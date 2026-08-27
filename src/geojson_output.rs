//! Writes waiting zones out as a client-facing GeoJSON `FeatureCollection`,
//! reprojected to WGS84 lon/lat, for a client that geofences against it
//! from its own GPS.
//!
//! Scope, deliberately: this emits geometry (`perimeter`, `stop_line`), the
//! id [`crate::zone_generator`] already assigns (`waiting_zone_id`), and
//! `modes` (see [`zone_modes`]). `intersection_id` and `heading` still
//! aren't modeled anywhere in this crate or in `sumo_types`, so they still
//! aren't invented here either — seeding a property with data nobody
//! computed would be worse than leaving it out.
//!
//! ## Reprojection
//!
//! A `.net.xml`'s coordinates are **not** lon/lat: `netconvert` projects the
//! source `.osm` into a local, usually metric CRS (declared as a PROJ4
//! string in `location/@projParameter`) and then offsets it
//! (`location/@netOffset`) so the network sits near the origin. GeoJSON — and
//! a client's GPS — assume WGS84 lon/lat, so every coordinate here is
//! reprojected: the `netOffset` translation is undone first, then
//! [`proj4rs`] inverts the projection. A network with no projection at all
//! (`projParameter="!"`, e.g. a synthetic/test network) has nothing to
//! invert — [`write`] fails outright rather than emit coordinates dressed up
//! as lon/lat that aren't.
//!
//! ## Geometry
//!
//! [`E3Detector`]'s gates carry a lane and a linear position on it, not a
//! polygon — so each lane in a zone contributes its own rectangle: the
//! lane's shape between the entry and exit stations, offset left/right by
//! half the lane's width.
//!
//! A zone spanning several lanes (one shared signal group) merges those
//! rectangles into a single polygon rather than emitting one per lane —
//! [`merged_zone_ring`] — because a general polygon union is real
//! computational geometry this module doesn't need to do to get there:
//! `zone_generator`'s own docs guarantee every lane in one zone is a lane of
//! the *same edge* (grouping is keyed on `(edge, directions)`), and SUMO
//! numbers an edge's lanes 0..N from right to left, so "merge N side-by-side
//! rectangles" reduces to "take the rightmost lane's own right edge and the
//! leftmost lane's own left edge, and join them at the ends" — no clipping,
//! no union algorithm, just picking which two of the already-computed
//! offset boundaries form the outside. [`merged_zone_ring`] verifies the
//! lane indices are actually contiguous before trusting that (a group
//! containing lane 0 and lane 2 but not lane 1 would otherwise silently
//! claim lane 1's own ground) — every zone `zone_generator` has ever
//! produced for real data satisfies this, so a violation prints an error
//! (rather than a network-halting `bail!`, matching `zone_generator`'s own
//! per-lane warnings for a different-but-similarly-shaped "shouldn't
//! happen, but don't crash the whole run over it" case) and merges anyway,
//! best-effort — a caller that sees the error on real output has a real
//! `.net.xml` shape this module's own assumption doesn't hold for, worth
//! looking at directly rather than papered over by a silent fallback.

use anstream::eprintln;
use anstyle::{AnsiColor, Style};
use anyhow::{Context, Result, bail};
use geojson::{Feature, FeatureCollection, Geometry, JsonObject, Position};
use proj4rs::proj::Proj;
use proj4rs::transform::transform;
use std::collections::{BTreeSet, HashMap, HashSet};
use std::path::Path;
use sumo_types::additional::domain::{DetectorId, E3Detector, LanePosition};
use sumo_types::domain::{EdgeFunction, EdgeId, Lane, LaneIndex, Location, Network, Point, Projection, Shape, VClass};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

/// Styles the "error:" prefix on [`merged_zone_ring`]'s own message the way
/// `cargo`/`rustc` style theirs — bold red — mirroring
/// `zone_generator::WARNING`'s own reasoning for its (yellow) warnings.
const ERROR: Style = AnsiColor::Red.on_default().bold();

/// The one CRS every coordinate in the output is reprojected into: WGS84
/// lon/lat, in that axis order (matching GeoJSON's `[lon, lat]`, not
/// `[lat, lon]`).
const WGS84_LONLAT: &str = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";

/// Below this, in metres, no real vehicle fits — real Barcelona
/// `netconvert` output still produces lanes this short anyway (a stub
/// split off right at a junction, sometimes 0.1–0.2m), and drawing one at
/// its own real length gives a paper-thin sliver that reads as broken
/// rather than as a real piece of road. [`padded_entry`] is the fix: pad
/// the *drawn* span out to at least this, extrapolating backward past the
/// lane's own real start (`trimmed_segments`'s own docs) — visual only,
/// the real `.net.xml` length and the E3Detector gate positions
/// SUMO/engine actually see are untouched.
const MIN_DRAWN_LANE_LENGTH_METERS: f64 = 5.0;

/// `entry`, pulled earlier if needed so `exit - entry` is at least
/// `target_length` — see [`MIN_DRAWN_LANE_LENGTH_METERS`]'s own docs for
/// why that's the target [`zone_feature`] asks for by default. Always pads
/// backward, never moves `exit`: `exit` is either a zone's own real stop
/// line or an extended ancestor's own real hand-off point into the next
/// lane down the chain (`zone_generator::extended_entry_lanes`'s own
/// docs) — both already correct, real boundaries, not something to shift.
///
/// `target_length` isn't always [`MIN_DRAWN_LANE_LENGTH_METERS`]: a
/// shorter target is [`resolve_overlaps`]'s own way of asking for as much
/// padding as still fits without reaching into a real neighbour — see its
/// own docs on why that's better than the all-or-nothing choice between
/// the full default and none at all.
///
/// Deliberately a straight-line extrapolation of `lane`'s own first
/// segment (`trimmed_segments`'s own docs), never a real neighbouring
/// lane's own shape, even when one exists right behind a fork or an
/// already-another-zone's-own signal (`zone_generator`'s own docs on why
/// extension itself won't walk through either): a lane on the *other*
/// side of either is, in general, ground a *different* zone's own polygon
/// already draws — borrowing it here to pad this one out doesn't just
/// overlap that zone's own area, it makes the two read as one continuous
/// shape on a map, which defeats the entire point of them being separate
/// zones. A straight line reaches into unclaimed space instead — a
/// weaker approximation of the real street's own curve over the few
/// metres it typically covers, but one that never fakes shared ground
/// with a real, distinct zone.
fn padded_entry(entry: Length, exit: Length, target_length: Length) -> Length {
    let span = exit - entry;
    if span < target_length { entry - (target_length - span) } else { entry }
}

/// Reprojects a network's local/projected coordinates to WGS84 lon/lat.
/// Built once per network — parsing `location/@projParameter` and the WGS84
/// target is the expensive part, not the transform itself — and reused for
/// every point.
struct Reprojector {
    from: Proj,
    to: Proj,
    net_offset: Point,
}

impl Reprojector {
    fn new(location: &Location) -> Result<Self> {
        let Projection::Proj4(proj_string) = &location.projection else {
            bail!(
                "network is not georeferenced (location/@projParameter is \"!\"): \
                 GeoJSON needs real-world coordinates, and an unprojected network has none to give"
            );
        };
        let from = Proj::from_proj_string(proj_string).with_context(|| {
            format!("invalid PROJ4 string in .net.xml location: {proj_string:?}")
        })?;
        let to =
            Proj::from_proj_string(WGS84_LONLAT).expect("WGS84_LONLAT is a valid PROJ4 string");
        Ok(Self {
            from,
            to,
            net_offset: location.net_offset,
        })
    }

    /// `point` is in the network's own coordinates, straight from a
    /// `.net.xml` shape. `netOffset` is undone before the inverse
    /// projection, mirroring how `netconvert` applied it going the other
    /// way when it built the network.
    fn to_lon_lat(&self, point: Point) -> Result<[f64; 2]> {
        let mut coords = (
            point.x - self.net_offset.x,
            point.y - self.net_offset.y,
            0.0,
        );
        transform(&self.from, &self.to, &mut coords)
            .with_context(|| format!("could not reproject network point {point:?} to WGS84"))?;
        Ok([coords.0.to_degrees(), coords.1.to_degrees()])
    }
}

/// `shape`'s own total arc length — for a single lane's shape, the same
/// figure `.net.xml`'s own `length` attribute already gives (so this is
/// never called for one), but [`chain_shape`]'s combined multi-lane shape
/// has no such attribute of its own to read, only the concatenated points
/// to measure.
fn shape_length(shape: &Shape) -> Length {
    Length::new::<meter>(
        shape.0.windows(2).map(|window| {
            let [a, b] = window else {
                unreachable!("windows(2) always yields length-2 slices")
            };
            (b.x - a.x).hypot(b.y - a.y)
        }).sum(),
    )
}

/// The point on `shape` at arc-length `distance` from its start (clamped to
/// the shape's own length), together with the unit tangent of the segment
/// it falls on — used to offset left/right by half a lane's width.
///
/// Falls back to an arbitrary tangent `(1.0, 0.0)` on a degenerate
/// (empty or single-point) shape rather than panicking: real `netconvert`
/// output always writes a proper polyline, so this only guards a malformed
/// `.net.xml`, not a case this crate is expected to handle precisely.
fn point_and_tangent_at(shape: &Shape, distance: Length) -> (Point, (f64, f64)) {
    let target = distance.get::<meter>().max(0.0);
    let mut travelled = 0.0;

    for window in shape.0.windows(2) {
        let [a, b] = window else {
            unreachable!("windows(2) always yields length-2 slices")
        };
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let segment_length = dx.hypot(dy);
        if segment_length > 0.0 && travelled + segment_length < target {
            travelled += segment_length;
            continue;
        }

        let t = if segment_length > 0.0 {
            (target - travelled) / segment_length
        } else {
            0.0
        };
        let point = Point {
            x: a.x + dx * t,
            y: a.y + dy * t,
            z: a.z + (b.z - a.z) * t,
        };
        let tangent = if segment_length > 0.0 {
            (dx / segment_length, dy / segment_length)
        } else {
            (1.0, 0.0)
        };
        return (point, tangent);
    }

    (shape.0.last().copied().unwrap_or_default(), (1.0, 0.0))
}

/// `position` resolved to a distance from the lane's start, given the
/// lane's own `length` (needed to resolve [`LanePosition::FromEnd`]).
fn distance_from_start(position: LanePosition, lane_length: Length) -> Length {
    match position {
        LanePosition::FromStart(distance) => distance,
        LanePosition::FromEnd(distance) => lane_length - distance,
    }
}

/// `point` offset perpendicular to `tangent` by `distance` — to the left of
/// the direction of travel when `distance` is positive, to the right when
/// negative. Only `x`/`y` move: `z` (elevation) isn't touched by a
/// perpendicular offset.
fn offset_perpendicular(point: Point, tangent: (f64, f64), distance: Length) -> Point {
    let (dx, dy) = tangent;
    let d = distance.get::<meter>();
    Point {
        x: point.x - dy * d,
        y: point.y + dx * d,
        z: point.z,
    }
}

/// `shape`'s own segments, each clipped to the arc-length span
/// `[entry, exit]`, as `(segment_start, segment_end, segment_tangent)` —
/// a segment fully outside the span is dropped, one straddling an edge of
/// it is cut down to the part inside. A lane's own shape is exactly this
/// same sequence of straight segments; a zone whose entry/exit don't
/// happen to land exactly on original vertices still needs the ones
/// *between* them to actually follow the bend, not just the two ends.
/// `entry` itself can be negative — see [`padded_entry`] — in which case
/// the span between it and the shape's own real start (0) is an
/// extrapolated straight continuation of the first real segment, not
/// anything from `shape` itself.
///
/// Deliberately per-*segment*, not per-vertex: an earlier version of this
/// returned one point per vertex, offset by the tangent of whichever
/// segment led into it, and joined consecutive offset points with a
/// straight edge — fine on a gentle curve, but at a sharp turn that edge
/// connects two points offset in genuinely different directions, and nothing
/// stops the straight line between them from cutting back across the
/// polygon's own interior (a self-intersecting "bowtie", visibly wrong on
/// a real Barcelona zone — see the module docs). Every segment offering
/// *both* its own ends, in its own single direction, means every polygon
/// edge is a real offset of a real segment — an interior vertex gets two
/// close-together offset points (a small bevel facet) rather than one
/// point serving two different directions, so consecutive edges can only
/// ever meet, never cross past each other.
fn trimmed_segments(shape: &Shape, entry: Length, exit: Length) -> Vec<(Point, Point, (f64, f64))> {
    let entry_m = entry.get::<meter>();
    let exit_m = exit.get::<meter>().max(entry_m);

    let mut segments = Vec::new();

    // `entry_m` negative means [`padded_entry`] pulled it before the
    // lane's own real start to pad out a too-short lane's drawn span (see
    // its own docs) — extrapolated here along the first real segment's own
    // tangent, the same "continue in a straight line" a driver already
    // effectively does arriving from whatever's further back. Real lanes
    // always have at least two shape points; a synthetic/malformed one
    // with fewer just skips this (nothing to extrapolate the direction
    // of), same as this function already did for `entry_m >= 0.0`.
    if entry_m < 0.0
        && let [first, second, ..] = shape.0.as_slice()
    {
        let (dx, dy) = (second.x - first.x, second.y - first.y);
        let segment_length = dx.hypot(dy);
        if segment_length > 0.0 {
            let tangent = (dx / segment_length, dy / segment_length);
            let extend_to = exit_m.min(0.0);
            if extend_to > entry_m {
                let point_at = |t: f64| Point {
                    x: first.x + tangent.0 * t,
                    y: first.y + tangent.1 * t,
                    z: first.z,
                };
                segments.push((point_at(entry_m), point_at(extend_to), tangent));
            }
        }
    }

    let mut travelled = 0.0;
    for window in shape.0.windows(2) {
        let [a, b] = window else {
            unreachable!("windows(2) always yields length-2 slices")
        };
        let segment_length = (b.x - a.x).hypot(b.y - a.y);
        if segment_length <= 0.0 {
            // A degenerate, zero-length segment contributes no distance
            // and has no direction of its own to offer as a tangent —
            // skip it rather than divide by zero.
            continue;
        }
        let (segment_start, segment_end) = (travelled, travelled + segment_length);
        travelled = segment_end;
        if segment_end <= entry_m || segment_start >= exit_m {
            continue; // entirely outside the span
        }

        let tangent = ((b.x - a.x) / segment_length, (b.y - a.y) / segment_length);
        let lerp = |t: f64| Point {
            x: a.x + (b.x - a.x) * t,
            y: a.y + (b.y - a.y) * t,
            z: a.z + (b.z - a.z) * t,
        };
        let t0 = (entry_m.max(segment_start) - segment_start) / segment_length;
        let t1 = (exit_m.min(segment_end) - segment_start) / segment_length;
        segments.push((lerp(t0), lerp(t1), tangent));

        if travelled >= exit_m {
            break;
        }
    }
    segments
}

/// One side of a zone's offset boundary — one lane's own shape between
/// `entry`/`exit` (arc-length from the lane's start — see
/// [`trimmed_segments`]), offset perpendicular by `half_width` (positive =
/// left of the segment's own direction of travel, negative = right — see
/// [`offset_perpendicular`]). For a straight lane this is still exactly the
/// two corners it always was — `trimmed_segments` returns one segment,
/// entry to exit — it only grows more corners where the lane itself bends.
/// Called twice per lane in [`merged_zone_ring`]'s single-lane case (once
/// each side) and once per side for its multi-lane case (each side then
/// coming from a *different* lane — the group's own leftmost and
/// rightmost).
///
/// `entry == exit` (or a shape with no segments at all inside the span) has
/// no segment to offer a direction, so [`point_and_tangent_at`] stands in
/// for a single degenerate point — the same "collapsed rectangle" a
/// zero-length zone always produced before this had segments to walk.
///
/// Bevel-joining every segment's own offset (above) stops a *sharp* turn
/// from producing a straight edge that cuts back across the polygon's own
/// interior, but it isn't sufficient on its own: a real Barcelona
/// walkingarea can be short and *wide* at once (one seen while fixing this
/// was 2.9m long, 8 shape points, 4m wide — the width bigger than the
/// whole path's own length), and offsetting by more than the path's local
/// turning radius folds the offset boundary back on itself regardless of
/// how the offset points are joined. [`remove_self_intersections`] (below)
/// cleans each side's own boundary of exactly that — a convex hull (tried
/// first) stays simple by construction, but a lane 100m+ long with a gentle
/// real bend hulls into a shape covering nearly 3x its own true area:
/// accurate for a short, sharp zigzag is not the same property as accurate
/// for a long, gentle curve, and this crate needs both.
fn offset_boundary(shape: &Shape, entry: Length, exit: Length, half_width: Length) -> Vec<Point> {
    let segments = trimmed_segments(shape, entry, exit);
    if segments.is_empty() {
        let (point, tangent) = point_and_tangent_at(shape, entry);
        return vec![offset_perpendicular(point, tangent, half_width)];
    }
    let points = segments
        .iter()
        .flat_map(|&(start, end, tangent)| {
            [
                offset_perpendicular(start, tangent, half_width),
                offset_perpendicular(end, tangent, half_width),
            ]
        })
        .collect();
    simplify_points(remove_self_intersections(points), SIMPLIFY_TOLERANCE_METERS)
}

/// Below this, in metres, [`simplify_points`] treats an intermediate point
/// as contributing nothing a straight line between its neighbours doesn't
/// already capture — chosen well under any real visual or geometric
/// significance (the same "a centimetre is noise" reasoning
/// [`OVERLAP_AREA_THRESHOLD_DEG2`]'s own docs use), comfortably above the
/// sub-millimetre float noise [`remove_self_intersections`]'s own line
/// intersections can introduce.
const SIMPLIFY_TOLERANCE_METERS: f64 = 0.05;

/// Ramer-Douglas-Peucker simplification of an *open* polyline: `points`
/// reduced to whichever subset a straight line between the two endpoints
/// can't already approximate within `tolerance` — recursively finding
/// whichever intermediate point strays furthest from that line, keeping
/// only it (and repeating on each half it splits the line into) when that
/// strays by more than `tolerance`, and dropping the entire interior
/// otherwise. Always keeps both endpoints, and never reorders anything —
/// the result is a subsequence of `points`, in the same order.
///
/// Why this exists: [`chain_shape`] concatenates however many real lanes'
/// own shapes a zone's extension walks through end to end, and a real
/// Barcelona chain often includes several sub-metre "connector" lanes (see
/// its own module docs) whose shape points land mere centimetres apart
/// once strung together — real detail `netconvert` recorded, but far finer
/// than this crate's own output needs to reproduce faithfully, and it only
/// bloats the GeoJSON a client downloads for no visual or geometric
/// benefit any of those extra points buy back.
fn simplify_points(points: Vec<Point>, tolerance: f64) -> Vec<Point> {
    if points.len() < 3 {
        return points;
    }

    let perpendicular_distance = |p: Point, a: Point, b: Point| -> f64 {
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let len = dx.hypot(dy);
        if len == 0.0 {
            return (p.x - a.x).hypot(p.y - a.y);
        }
        ((p.x - a.x) * dy - (p.y - a.y) * dx).abs() / len
    };

    let (first, last) = (points[0], points[points.len() - 1]);
    let (mut max_dist, mut farthest) = (0.0, 0);
    for (i, &point) in points.iter().enumerate().take(points.len() - 1).skip(1) {
        let dist = perpendicular_distance(point, first, last);
        if dist > max_dist {
            max_dist = dist;
            farthest = i;
        }
    }

    if max_dist > tolerance {
        let mut kept = simplify_points(points[..=farthest].to_vec(), tolerance);
        kept.pop(); // shared with the second half's own first point
        kept.extend(simplify_points(points[farthest..].to_vec(), tolerance));
        kept
    } else {
        vec![first, last]
    }
}

/// [`offset_boundary`] on each side of `shape` between `entry`/`exit`,
/// closed via [`close_ring`] — the rectangle-or-bent-quadrilateral one
/// lane's own shape contributes, or, when `shape` is several real lanes'
/// shapes already concatenated end to end ([`chain_shape`]), the single
/// seamless polygon a whole extended-ancestor *chain* contributes instead
/// of one independent rectangle per lane in it. A chain that's only one
/// lane long (no real predecessor within the zone) is simply the `shape`/
/// `half_width` of that one lane — there's no special one-lane case here,
/// only a one-lane [`chain_shape`] call.
fn shape_ring(shape: &Shape, entry: Length, exit: Length, half_width: Length, reproject: &Reprojector) -> Result<Vec<Position>> {
    let left = offset_boundary(shape, entry, exit, half_width);
    let right = offset_boundary(shape, entry, exit, -half_width);
    // A straight lane of this span and width would enclose exactly this —
    // real bends only ever pull the true area *below* it, never above, so
    // it's a sound floor for `close_ring`'s own degenerate-collapse check,
    // not just a rough guess.
    let naive_area_m2 = (exit - entry).get::<meter>().abs() * half_width.get::<meter>().abs() * 2.0;
    close_ring(left, right, naive_area_m2, reproject)
}

/// Below this fraction of `close_ring`'s own naive expected area, its
/// self-intersection cleanup is judged to have collapsed the ring rather
/// than merely trimmed it — see its own docs. A real bend can legitimately
/// give back a fair chunk of the naive rectangle (a tight U costs the most,
/// and even that rarely halves it), but never anywhere near this little:
/// the real Barcelona case this constant was chosen against (a 6.7m,
/// 3.2m-wide chain with two sharp direction changes packed into ~1.5m
/// segments) collapsed to 0.014% of its own naive area, several orders of
/// magnitude below any real curve's own cost.
const MIN_OFFSET_RING_AREA_FRACTION: f64 = 0.1;

/// `left` and `right` (each already [`offset_boundary`]-cleaned, running in
/// the same direction — entry to exit), closed into one reprojected linear
/// ring: `right` reversed so the two sides trace the perimeter
/// consistently, the ring closed (first point repeated last) *before* a
/// final self-intersection pass so that closing edge — as real as any
/// other — gets checked too, not just the ones the two sides already
/// contributed (the two sides can still cross *each other*, e.g. a U-shaped
/// path narrower than the offset folds one side past where the other now
/// ends), then reprojected, deduplicated (a tight bevel facet's two
/// corners — see [`offset_boundary`]'s own docs — can be distinct in local
/// metres but round to the exact same lon/lat once
/// [`Reprojector::to_lon_lat`] is done with them, leaving a zero-length
/// edge that's harmless in itself but can confuse a naive downstream
/// self-intersection check into a false positive on the vertex it shares),
/// and re-closed if any of that rounding or dedup disturbed the exact
/// repeat.
///
/// `naive_area_m2` (see [`shape_ring`]'s own docs) guards against
/// [`remove_self_intersections`]'s own documented weakness: a path whose
/// bends are tight relative to the offset width can trigger a cascade of
/// splices that eats into real, non-crossing area along with the actual
/// bowtie loop, rather than trimming just the loop itself — on one real
/// Barcelona chain (`1395130587_2`, entries backing a signal-controlled
/// zone) this collapsed a real ~20m² ribbon down to a 0.003m² sliver, a
/// visibly broken shape no legitimate bend produces. Checked, not assumed,
/// the same way [`clip_by_constraints`] checks its own direct clip before
/// trusting it: if the cleaned ring's area falls below
/// [`MIN_OFFSET_RING_AREA_FRACTION`] of `naive_area_m2`, this falls back to
/// the convex hull of the *uncleaned* combined boundary instead — guaranteed
/// simple by construction, at the cost of claiming a bit more than the
/// exact bent shape, exactly the tradeoff already made there.
fn close_ring(left: Vec<Point>, mut right: Vec<Point>, naive_area_m2: f64, reproject: &Reprojector) -> Result<Vec<Position>> {
    right.reverse();
    let mut ring_points: Vec<Point> = left.into_iter().chain(right).collect();
    if let Some(&first) = ring_points.first() {
        ring_points.push(first);
    }
    let cleaned = remove_self_intersections(ring_points.clone());

    let as_xy = |points: &[Point]| -> Vec<(f64, f64)> { points.iter().map(|p| (p.x, p.y)).collect() };
    let cleaned_area_m2 = signed_area(&as_xy(&cleaned)).abs();

    let final_points = if naive_area_m2 > 0.0 && cleaned_area_m2 < naive_area_m2 * MIN_OFFSET_RING_AREA_FRACTION {
        convex_hull(&as_xy(&ring_points)).into_iter().map(|(x, y)| Point { x, y, z: 0.0 }).collect()
    } else {
        cleaned
    };

    let mut ring = final_points
        .into_iter()
        .map(|corner| reproject.to_lon_lat(corner).map(Position::from))
        .collect::<Result<Vec<_>>>()?;
    ring.dedup();
    if ring.first() != ring.last()
        && let Some(first) = ring.first().cloned()
    {
        ring.push(first);
    }
    Ok(ring)
}

/// The single ring covering every lane in `lane_gates` — each `(lane, entry
/// distance, exit distance)`, as computed per-gate in [`zone_feature`] —
/// on the assumption, true for every zone `zone_generator` has ever
/// produced from real data, that they're physically contiguous (see the
/// module docs). `None` only for `lane_gates` empty, the same "nothing to
/// build" case [`overlapping_zone_ids`]'s own callers already tolerate for
/// a zone with no gates at all.
///
/// A `zone_id` whose lanes turn out *not* to be contiguous still gets a
/// ring back — merging its outermost two lanes regardless — but also an
/// `error:` line on stderr: the ring may be claiming ground belonging to a
/// lane sitting in between that isn't actually part of this zone (that
/// lane is presumably a different zone's), which is worth someone looking
/// at directly rather than either crashing the whole run over it or
/// papering over it with a silent per-lane fallback.
fn merged_zone_ring(
    zone_id: &DetectorId,
    lane_gates: &[(&Lane, Length, Length)],
    reproject: &Reprojector,
) -> Result<Option<Vec<Position>>> {
    if lane_gates.is_empty() {
        return Ok(None);
    }

    let mut sorted: Vec<(&Lane, Length, Length)> = lane_gates.to_vec();
    sorted.sort_by_key(|(lane, _, _)| lane.index.0);
    let contiguous = sorted
        .windows(2)
        .all(|pair| pair[1].0.index.0 == pair[0].0.index.0 + 1);
    if !contiguous {
        let indices: Vec<usize> = sorted.iter().map(|(lane, _, _)| lane.index.0).collect();
        eprintln!(
            "{ERROR}error:{ERROR:#} zone \"{zone_id}\" merges non-contiguous lane indices \
             {indices:?} — the merged polygon may wrongly claim ground belonging to a lane \
             in between that isn't actually part of this zone"
        );
    }

    // SUMO numbers an edge's lanes 0..N from right to left (see the module
    // docs), and `offset_perpendicular`'s own positive direction is left of
    // travel — so the rightmost (lowest-index) lane's *outer* edge is its
    // negative offset, and the leftmost (highest-index) lane's outer edge
    // is its positive one; the shared edges in between, where one lane's
    // own inner boundary would touch its neighbour's, are exactly what
    // merging is for skipping.
    let (right_lane, right_entry, right_exit) = *sorted.first().expect("checked non-empty above");
    let (left_lane, left_entry, left_exit) = *sorted.last().expect("checked non-empty above");

    let right = offset_boundary(
        &right_lane.shape,
        right_entry,
        right_exit,
        -(right_lane.width / 2.0),
    );
    let left = offset_boundary(
        &left_lane.shape,
        left_entry,
        left_exit,
        left_lane.width / 2.0,
    );
    // The smaller of the two lanes' own naive rectangles — a safe floor for
    // [`close_ring`]'s own degenerate-collapse check, since a contiguous
    // merge of both never claims less ground than either lane alone would
    // on its own (see [`shape_ring`]'s own docs on why this floor is sound
    // in the first place).
    let naive_area_m2 = ((right_exit - right_entry).get::<meter>().abs() * right_lane.width.get::<meter>().abs())
        .min((left_exit - left_entry).get::<meter>().abs() * left_lane.width.get::<meter>().abs());
    close_ring(left, right, naive_area_m2, reproject).map(Some)
}

/// Where segments `a`-`b` and `c`-`d` *properly* cross (interiors meet at a
/// single point, not merely an endpoint) — `None` if they don't, same
/// strict "properly" [`polygons_overlap`]'s own segment test uses and for
/// the same reason: two offset points meant to coincide (or nearly so)
/// must not register as a crossing to remove.
fn segment_intersection_point(a: Point, b: Point, c: Point, d: Point) -> Option<Point> {
    let (p1, p2, p3, p4) = ((a.x, a.y), (b.x, b.y), (c.x, c.y), (d.x, d.y));
    let d1 = orientation(p3, p4, p1);
    let d2 = orientation(p3, p4, p2);
    let d3 = orientation(p1, p2, p3);
    let d4 = orientation(p1, p2, p4);
    if (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0) {
        let (x, y) = line_intersection(p1, p2, p3, p4);
        Some(Point { x, y, z: 0.0 })
    } else {
        None
    }
}

/// `path` (an open polyline — the boundary offset to one side of a lane's
/// own centreline) with every self-intersecting loop cut out: whenever two
/// non-adjacent segments of `path` properly cross, the portion of the path
/// between them — the loop a too-sharp turn folded back on itself — is
/// replaced by the single point where they cross. Repeats until no
/// crossing remains (removing one loop can occasionally reveal another
/// behind it). `path.len()` is always small here (one lane's own shape
/// points, rarely more than a couple of dozen), so the naive rescan-after-
/// every-splice approach is in no danger of being a real cost.
///
/// This is what makes a lane's own offset boundary track a real bend
/// instead of ballooning into a convex hull around it (see
/// [`offset_boundary`]'s own docs) or, left unaddressed, folding into a
/// self-intersecting "bowtie" a client's point-in-polygon test can't reason
/// about.
fn remove_self_intersections(mut path: Vec<Point>) -> Vec<Point> {
    'restart: loop {
        for i in 0..path.len().saturating_sub(1) {
            for j in (i + 2)..path.len().saturating_sub(1) {
                if let Some(meeting_point) =
                    segment_intersection_point(path[i], path[i + 1], path[j], path[j + 1])
                {
                    path.splice((i + 1)..=j, std::iter::once(meeting_point));
                    continue 'restart;
                }
            }
        }
        return path;
    }
}

/// Twice the signed area of triangle `a`, `b`, `c` — positive when `c` is
/// left of the ray `a -> b`, negative when right, zero when collinear.
/// Named for what it's used for here, not for what it computes: the usual
/// name is "cross product of `b-a` and `c-a`".
fn orientation(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
}

/// The signed area enclosed by `ring` (the shoelace formula) — positive for
/// a counterclockwise winding, negative for clockwise. [`polygon_overlap_area`]
/// only cares about the unsigned area, but [`clip_to_convex`] needs to know
/// which way a ring winds to know which side of each edge is "inside".
fn signed_area(ring: &[(f64, f64)]) -> f64 {
    let n = ring.len();
    if n < 3 {
        return 0.0;
    }
    (0..n)
        .map(|i| {
            let (x1, y1) = ring[i];
            let (x2, y2) = ring[(i + 1) % n];
            x1 * y2 - x2 * y1
        })
        .sum::<f64>()
        / 2.0
}

/// Where segment `a`-`b` crosses line `c`-`d` (extended infinitely) —
/// only ever called from [`clip_to_convex`] on a pair its own orientation
/// tests already established actually crosses `c`-`d` somewhere on `a`-`b`
/// itself, so the division below is never by zero in practice (parallel
/// lines never reach an inside/outside disagreement to call this on).
fn line_intersection(a: (f64, f64), b: (f64, f64), c: (f64, f64), d: (f64, f64)) -> (f64, f64) {
    let (x1, y1, x2, y2) = (a.0, a.1, b.0, b.1);
    let (x3, y3, x4, y4) = (c.0, c.1, d.0, d.1);
    let denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
    let t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
}

/// One Sutherland-Hodgman clip pass: `subject` (a closed polygon, as plain
/// `(x, y)` points, no repeated closing vertex needed) cut down to
/// whichever side of the infinite line through `line_a`-`line_b` that
/// `is_inside` calls "in". The one primitive both [`clip_to_convex`] (one
/// call per edge of a convex clip polygon, "in" meaning the interior side
/// of that specific edge) and [`clip_half_plane`] (a single call, the
/// "clip polygon" being an infinite half-plane rather than a closed shape)
/// are built from — the algorithm itself doesn't care which.
fn clip_by_line(
    subject: &[(f64, f64)],
    line_a: (f64, f64),
    line_b: (f64, f64),
    is_inside: impl Fn((f64, f64)) -> bool,
) -> Vec<(f64, f64)> {
    let mut output = Vec::new();
    for j in 0..subject.len() {
        let current = subject[j];
        let previous = subject[(j + subject.len() - 1) % subject.len()];
        let (current_in, previous_in) = (is_inside(current), is_inside(previous));
        if current_in {
            if !previous_in {
                output.push(line_intersection(previous, current, line_a, line_b));
            }
            output.push(current);
        } else if previous_in {
            output.push(line_intersection(previous, current, line_a, line_b));
        }
    }
    output
}

/// Sutherland-Hodgman polygon clipping: `subject` cut down to the part of
/// it that also lies inside convex polygon `clip`. Requires `clip` to be
/// convex — true of the rectangle a *straight* lane's [`merged_zone_ring`]
/// produces, not guaranteed for one following a real bend (see
/// [`offset_boundary`]'s own docs); `subject` can be any simple polygon,
/// though here it's always one of those same rings. Winding-direction-
/// agnostic — [`signed_area`] on `clip` itself decides which side of each
/// of its own edges counts as "inside",
/// so `subject` and `clip` don't need to agree with each other's winding.
fn clip_to_convex(subject: &[(f64, f64)], clip: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let inside_sign = if signed_area(clip) >= 0.0 { 1.0 } else { -1.0 };
    let mut output = subject.to_vec();

    for i in 0..clip.len() {
        if output.is_empty() {
            break;
        }
        let edge_start = clip[i];
        let edge_end = clip[(i + 1) % clip.len()];
        let is_inside = |p: (f64, f64)| orientation(edge_start, edge_end, p) * inside_sign >= 0.0;
        output = clip_by_line(&output, edge_start, edge_end, is_inside);
    }
    output
}

/// `subject` cut down to the closed half-plane through `point_on_line`,
/// perpendicular to `normal`, on the side `normal` points *away* from
/// (i.e. keeps `p` where `(p - point_on_line) · normal <= 0`). Used by
/// [`bisect`] to split two overlapping rings along the perpendicular
/// bisector of their own centroids — a full clip polygon would be
/// meaningless there (there's no second, opposite edge to close it with),
/// which is why this exists as its own primitive rather than a two-point
/// `clip_to_convex` call.
fn clip_half_plane(subject: &[(f64, f64)], point_on_line: (f64, f64), normal: (f64, f64)) -> Vec<(f64, f64)> {
    let side = move |p: (f64, f64)| (p.0 - point_on_line.0) * normal.0 + (p.1 - point_on_line.1) * normal.1;
    // Any second point on the line works; rotating `normal` 90° gives one
    // for free without needing the line in any other form.
    let tangent = (-normal.1, normal.0);
    let line_b = (point_on_line.0 + tangent.0, point_on_line.1 + tangent.1);
    clip_by_line(subject, point_on_line, line_b, move |p| side(p) <= 0.0)
}

/// Whether `p` lies inside (or on the boundary of) triangle `a`-`b`-`c` —
/// true exactly when `p` is never strictly on opposite sides of two of the
/// triangle's own edges. [`triangulate`]'s own ear test is the only
/// caller: whether the candidate ear's triangle contains some *other*
/// vertex of the polygon, which would make clipping it off wrong.
fn point_in_triangle(p: (f64, f64), a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> bool {
    let (d1, d2, d3) = (orientation(a, b, p), orientation(b, c, p), orientation(c, a, p));
    let has_negative = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    let has_positive = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    !(has_negative && has_positive)
}

/// Ear-clipping triangulation of simple (non-self-intersecting) polygon
/// `points` (no closing repeat) into triangles whose union covers exactly
/// the same area, wound counterclockwise either way — needed because
/// [`clip_to_convex`] (the crate's only polygon-clipping primitive)
/// requires its own `clip` argument to be convex, which a real zone's own
/// ring often isn't once [`chain_shape`] and [`merged_zone_ring`] are both
/// in play: a bent extended-ancestor segment, or a multi-lane group
/// following a real curve. A ring like that used to make
/// [`polygon_overlap_area`] silently underestimate (down to `0.0`) a real,
/// substantial overlap whenever it happened to be passed as the *clip*
/// side — confirmed on real Barcelona data: two zones with a genuine few
/// square metres in common measured `0.0` one direction and multiple m²
/// the other, purely depending on which one `clip_to_convex` treated as
/// convex. Splitting into triangles — always convex, by construction —
/// turns "does this arbitrary pair of possibly non-convex rings overlap"
/// into a sum of convex-vs-convex checks `clip_to_convex` already handles
/// exactly, with no dependence on which side is which.
///
/// Repeatedly finds a convex vertex whose own triangle (with its two
/// neighbours) contains no other vertex of what's left — an "ear" — clips
/// it into the output and removes it, until three vertices remain. Bails
/// out (returning whatever's already triangulated) rather than looping
/// forever if no ear can be found, which real `netconvert`-derived
/// geometry should never trigger but a sufficiently degenerate/numerically
/// noisy ring in principle could.
fn triangulate(points: &[(f64, f64)]) -> Vec<[(f64, f64); 3]> {
    if points.len() < 3 {
        return Vec::new();
    }
    let mut remaining = points.to_vec();
    if signed_area(&remaining) < 0.0 {
        remaining.reverse(); // consistent winding for the orientation-sign ear test below
    }

    let mut triangles = Vec::new();
    while remaining.len() > 3 {
        let n = remaining.len();
        let ear = (0..n).find(|&i| {
            let (prev, cur, next) = (remaining[(i + n - 1) % n], remaining[i], remaining[(i + 1) % n]);
            orientation(prev, cur, next) > 0.0
                && !(0..n).any(|j| {
                    j != i
                        && j != (i + n - 1) % n
                        && j != (i + 1) % n
                        && point_in_triangle(remaining[j], prev, cur, next)
                })
        });
        let Some(i) = ear else { break };
        let (prev, cur, next) = (remaining[(i + n - 1) % n], remaining[i], remaining[(i + 1) % n]);
        triangles.push([prev, cur, next]);
        remaining.remove(i);
    }
    if remaining.len() == 3 {
        triangles.push([remaining[0], remaining[1], remaining[2]]);
    }
    triangles
}

/// `a`'s own bounding box, as `(min_x, min_y, max_x, max_y)` —
/// [`polygons_overlap`]'s own cheap pre-filter, so the two rings' own
/// triangulations ([`polygon_overlap_area`]) only ever get computed for
/// pairs that could plausibly overlap at all. Real Barcelona data is a
/// whole city's worth of zones, and the overwhelming majority of any two
/// picked at random are nowhere near each other.
fn bounding_box(ring: &[(f64, f64)]) -> (f64, f64, f64, f64) {
    ring.iter().fold((f64::MAX, f64::MAX, f64::MIN, f64::MIN), |(min_x, min_y, max_x, max_y), &(x, y)| {
        (min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y))
    })
}

/// The area two rings `a` and `b` (each closed, GeoJSON-style: first
/// position repeated last) actually have in common — [`triangulate`] on
/// each, then every one of `a`'s own triangles clipped against every one
/// of `b`'s via [`clip_to_convex`] (always safe: a triangle is always
/// convex) and summed. Robust the way a boundary-crossing/point-in-polygon
/// test isn't: two rectangles built to share an edge exactly can still,
/// after independently reprojecting each to WGS84 (see the module docs on
/// why that happens), disagree on that edge's coordinates by the last
/// floating-point digit — enough to flip a "is this point exactly on the
/// line" test either way, but never enough to give the *clipped* polygon
/// any real area. A [`Self::OVERLAP_AREA_THRESHOLD_DEG2`]-sized clip is
/// noise from exactly that; real zones sharing actual ground clip to areas
/// many orders of magnitude bigger (their overlapping rectangles are
/// metres wide, not nanometres).
fn polygon_overlap_area(a: &[Position], b: &[Position]) -> f64 {
    let to_points = |ring: &[Position]| -> Vec<(f64, f64)> {
        // Drop the closing repeated vertex neither `triangulate` nor
        // `clip_to_convex` needs.
        ring[..ring.len().saturating_sub(1)]
            .iter()
            .map(|p| (p[0], p[1]))
            .collect()
    };
    let (triangles_a, triangles_b) = (triangulate(&to_points(a)), triangulate(&to_points(b)));
    triangles_a
        .iter()
        .flat_map(|ta| triangles_b.iter().map(move |tb| signed_area(&clip_to_convex(ta, tb)).abs()))
        .sum()
}

/// The area-weighted centroid of the ground `a` and `b` actually have in
/// common — same triangulate-and-clip decomposition as
/// [`polygon_overlap_area`], but averaging each clipped piece's own
/// (vertex-average) centroid instead of just summing its area. `None` when
/// they don't overlap at all (nothing to weight an average by).
///
/// [`bisecting_half_plane`]'s own anchor point, instead of the two rings'
/// *own* centroids — see its docs for why that matters for a ring far
/// longer than it is wide (a real Barcelona ribbon 80m long, contested by
/// several separate neighbours clustered around one small bend): the
/// ring's own centroid sits wherever its *whole* length averages to, which
/// can be nowhere near where any neighbour actually reaches into it.
fn polygon_overlap_centroid(a: &[Position], b: &[Position]) -> Option<(f64, f64)> {
    let to_points = |ring: &[Position]| -> Vec<(f64, f64)> {
        ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect()
    };
    let (triangles_a, triangles_b) = (triangulate(&to_points(a)), triangulate(&to_points(b)));
    let (mut area_sum, mut cx, mut cy) = (0.0, 0.0, 0.0);
    for ta in &triangles_a {
        for tb in &triangles_b {
            let clipped = clip_to_convex(ta, tb);
            let area = signed_area(&clipped).abs();
            if area <= 0.0 {
                continue;
            }
            let n = clipped.len() as f64;
            let (sx, sy) = clipped.iter().fold((0.0, 0.0), |(sx, sy), &(x, y)| (sx + x, sy + y));
            cx += area * (sx / n);
            cy += area * (sy / n);
            area_sum += area;
        }
    }
    (area_sum > 0.0).then_some((cx / area_sum, cy / area_sum))
}

/// Below this, in square degrees, an overlap is floating-point noise from
/// independently reprojecting two rectangles that share an edge exactly in
/// the network's own local CRS — not real ground two zones both claim.
/// [`polygon_overlap_area`]'s own docs have the full reasoning for why a
/// threshold is needed here at all.
///
/// A degree of longitude at Barcelona's own latitude is close to 84km, a
/// degree of latitude close to 111km, so converting a *area* in deg² to
/// real m² multiplies by *both* — 84,000 × 111,000 ≈ 9.32 × 10⁹ m² per
/// deg². An earlier value here (1×10⁻¹⁰) only accounted for *one* of those
/// two factors, making the real tolerance it enforced 0.93 m² — most of a
/// square metre, and large enough that a real, visible overlap between two
/// real Barcelona zones (confirmed: 0.043 m², two zones sharing real
/// ground a GPS point could actually land in) silently passed as "noise".
///
/// Chosen backward from a *different* floor than plain reprojection
/// jitter, though: [`clip_by_constraints`]'s own hull fallback is
/// documented as claiming "a little more than `base`'s own real,
/// possibly-concave area" whenever a bisected ring isn't convex to start
/// with, and [`resolve_overlaps`] never revisits a ring pair once it's
/// been given a constraint (see its own docs), so that little bit of
/// hull-fallback slack is a real, reproducible residual on real Barcelona
/// data (measured: 0.00013 m² between two genuinely adjacent zones at a
/// contested corner) rather than something later rounds clean up. 1×10⁻¹³
/// deg² (÷9.32×10⁹ ≈ 9.3cm²) clears that residual with room to spare while
/// staying two full orders of magnitude below the 0.043 m² real overlap
/// this threshold exists to still catch — floating-point reprojection
/// error itself is many more orders of magnitude smaller than either
/// figure, so there's no risk of swinging too far and flagging genuine
/// noise as an overlap.
const OVERLAP_AREA_THRESHOLD_DEG2: f64 = 1e-13;

/// Whether the areas enclosed by closed rings `a` and `b` overlap by more
/// than [`OVERLAP_AREA_THRESHOLD_DEG2`] — see [`polygon_overlap_area`]'s
/// own docs for why a bare "do they share any ground at all" test isn't
/// the right question to ask of reprojected geometry.
///
/// Checks the two rings' own bounding boxes first, and skips straight to
/// `false` if they don't even overlap — [`polygon_overlap_area`]'s own
/// triangulate-and-clip cost is worth avoiding for the overwhelming
/// majority of pairs across a whole city that could never plausibly
/// overlap in the first place, and this is called for every pair
/// [`overlapping_ring_indices`]'s own `O(n²)` scan considers.
fn polygons_overlap(a: &[Position], b: &[Position]) -> bool {
    let to_points = |ring: &[Position]| -> Vec<(f64, f64)> {
        ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect()
    };
    let (pa, pb) = (to_points(a), to_points(b));
    let (a_min_x, a_min_y, a_max_x, a_max_y) = bounding_box(&pa);
    let (b_min_x, b_min_y, b_max_x, b_max_y) = bounding_box(&pb);
    if a_max_x < b_min_x || b_max_x < a_min_x || a_max_y < b_min_y || b_max_y < a_min_y {
        return false;
    }
    polygon_overlap_area(a, b) > OVERLAP_AREA_THRESHOLD_DEG2
}

/// Every ring `feature`'s own geometry carries — one per sub-polygon (see
/// [`zone_feature`]'s own docs on why a zone can have more than one, via
/// extended ancestor entries). Empty if `feature` has no geometry or isn't
/// a `MultiPolygon` — never true of anything [`zone_feature`] itself
/// builds, but this is also called from [`overlapping_zone_ids`] on a
/// `FeatureCollection` a caller could in principle have built some other
/// way.
fn feature_rings(feature: &Feature) -> Vec<&[Position]> {
    let Some(geojson::GeometryValue::MultiPolygon { coordinates }) =
        feature.geometry.as_ref().map(|g| &g.value)
    else {
        return Vec::new();
    };
    coordinates.iter().filter_map(|polygon| polygon.first().map(Vec::as_slice)).collect()
}

/// Every `(zone_i, ring_i, zone_j, ring_j)` — indices into `features` and,
/// within each, into [`feature_rings`] — whose rings overlap, restricted to
/// `candidates` when given. The shared core [`overlapping_zone_ids`] and
/// [`resolve_overlaps`] are both built on, so the two can never disagree
/// about what counts as an overlap.
///
/// Restricting to `candidates` is only ever safe because both fixes
/// [`resolve_overlaps`] applies — dropping padding, [`bisect`]ing — strictly
/// shrink a ring, never grow one: a pair not already flagged against the
/// *full*, unresolved set can never become one later, so once a first full
/// pass has found every zone that's party to *some* overlap, nothing
/// outside that set needs rechecking again.
fn overlapping_ring_indices(
    features: &[Feature],
    candidates: &BTreeSet<usize>,
) -> Vec<(usize, usize, usize, usize)> {
    let indices: Vec<usize> = candidates.iter().copied().collect();
    let rings: Vec<Vec<&[Position]>> = indices.iter().map(|&i| feature_rings(&features[i])).collect();

    let mut found = Vec::new();
    for a in 0..indices.len() {
        for b in (a + 1)..indices.len() {
            for (ri, ring_a) in rings[a].iter().enumerate() {
                for (rj, ring_b) in rings[b].iter().enumerate() {
                    if polygons_overlap(ring_a, ring_b) {
                        found.push((indices[a], ri, indices[b], rj));
                    }
                }
            }
        }
    }
    found
}

/// [`overlapping_ring_indices`], deduplicated down to the `(zone_i,
/// zone_j)` pairs it found at least one conflicting ring for — every ring
/// two given zones must be checked against every ring of the other's for a
/// zone that's still one of the multi-polygon shapes
/// [`zone_feature`]'s own docs describe, but a caller only choosing which
/// *zones* need a fallback (see [`resolve_overlaps`]'s own padding-drop
/// phase) doesn't need the ring-level detail. `subset` of `None` means
/// every zone.
fn overlapping_indices(features: &[Feature], subset: Option<&BTreeSet<usize>>) -> Vec<(usize, usize)> {
    let owned_full_set;
    let candidates = match subset {
        Some(set) => set,
        None => {
            owned_full_set = (0..features.len()).collect();
            &owned_full_set
        }
    };
    overlapping_ring_indices(features, candidates)
        .into_iter()
        .map(|(i, _, j, _)| (i, j))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect()
}

/// Every pair of *different* zones in `collection` whose own areas overlap
/// — each zone's `MultiPolygon` checked ring-by-ring against every other
/// zone's. A client geofences against this output by testing a GPS point
/// against each zone's polygon (see the module docs); an overlap here means
/// a single point can match two different zones at once, which is exactly
/// the ambiguity a client asking "which crossing am I waiting at" can't
/// resolve on its own — see `zone_generator::pedestrian_zones`'s own docs
/// for a concrete way this can happen (one physical corner feeding two
/// differently-signalled crossings). [`to_feature_collection`] itself
/// already runs this same check and resolves whatever it finds (see
/// [`resolve_overlaps`]), so this only ever finds something real on output
/// this crate produced when called directly on a hand-built collection —
/// exactly how this module's own tests use it to exercise the detector in
/// isolation.
///
/// Doesn't check a zone's own polygons against each other — every zone with
/// at least one gate has exactly one ([`merged_zone_ring`]), so there's
/// nothing of its own left to compare — only different zones competing for
/// the same ground.
pub fn overlapping_zone_ids(collection: &FeatureCollection) -> Vec<(String, String)> {
    let ids: Vec<Option<String>> = collection
        .features
        .iter()
        .map(|feature| {
            feature.property("waiting_zone_id").and_then(|v| v.as_str()).map(str::to_string)
        })
        .collect();

    overlapping_indices(&collection.features, None)
        .into_iter()
        .filter_map(|(i, j)| Some((ids[i].clone()?, ids[j].clone()?)))
        .collect()
}

/// [`clip_half_plane`]'s own `(point_on_line, normal)` pair, named at the
/// type level everywhere one is passed around or stored — a
/// [`resolve_overlaps`] constraint, before it's actually applied to
/// anything.
type HalfPlane = ((f64, f64), (f64, f64));

/// `ring`'s own centroid, in whatever 2D coordinate space `ring` is
/// already in (lon/lat here). Only ever used to pick a *direction* to
/// split two overlapping rings along ([`bisecting_half_plane`]), so the
/// small distortion of averaging lon/lat directly rather than in a true
/// metric CRS doesn't matter — the shapes involved span a few dozen
/// metres at most.
fn ring_centroid(ring: &[Position]) -> (f64, f64) {
    let points = &ring[..ring.len().saturating_sub(1)];
    let n = (points.len().max(1)) as f64;
    let (sx, sy) = points.iter().fold((0.0, 0.0), |(sx, sy), p| (sx + p[0], sy + p[1]));
    (sx / n, sy / n)
}

/// The half-plane (`clip_half_plane`'s own `(point_on_line, normal)` pair)
/// that keeps `ring_a`'s own side when splitting it apart from `ring_b`: the
/// normal points from `ring_a`'s own centroid toward `ring_b`'s (negate it
/// for the half that keeps `ring_b`'s own side instead, as
/// [`resolve_overlaps`] does) — deciding *which* side is whose still only
/// needs each ring's own overall shape, not where specifically they
/// overlap. *Where* the line sits is a different question, though:
/// anchored at [`polygon_overlap_centroid`] (the actual overlapping
/// ground's own centroid) when there is one, rather than at the two rings'
/// own midpoint. For a ring far longer than it is wide (a real Barcelona
/// ribbon 80m long, contested by several separate neighbours clustered
/// around one small bend) the ring's own centroid sits wherever its
/// *whole* length averages to, nowhere near where any neighbour actually
/// reaches into it — a cut line built from that midpoint doesn't pass
/// through the real conflict at all, and combining several such
/// off-target lines from multiple simultaneous neighbours can converge on
/// a small, entirely valid-looking (simple, even convex) result that isn't
/// near any of the real overlaps. Falls back to the two rings' own midpoint
/// when they don't actually overlap (`polygon_overlap_centroid` returning
/// `None`) — not expected of anything [`resolve_overlaps`] itself calls
/// this on, but a straight line still needs *a* point to anchor at.
///
/// A straight cut along the *same* line, applied to both sides, can never
/// leave the two results overlapping each other — they're the two closed
/// half-planes of one line — which is what makes this safe to apply
/// without knowing anything about *why* the two shapes reach into each
/// other: real lane width at a sharp fork, a short lane's own extended
/// entry, or any other cause dropping padding doesn't touch.
///
/// `None` when the two centroids (nearly) coincide: no direction to cut
/// along. Left for [`resolve_overlaps`]'s own caller to report rather than
/// cutting blindly — the same "worth a human looking at directly" choice
/// [`merged_zone_ring`]'s own non-contiguous case makes, and, like that
/// one, expected to be vanishingly rare on real `netconvert` output rather
/// than a case this crate is actually designed around.
fn bisecting_half_plane(ring_a: &[Position], ring_b: &[Position]) -> Option<HalfPlane> {
    let (ax, ay) = ring_centroid(ring_a);
    let (bx, by) = ring_centroid(ring_b);
    let normal_a_to_b = (bx - ax, by - ay);
    if normal_a_to_b.0.hypot(normal_a_to_b.1) < 1e-12 {
        return None;
    }
    let anchor = polygon_overlap_centroid(ring_a, ring_b).unwrap_or(((ax + bx) / 2.0, (ay + by) / 2.0));
    Some((anchor, normal_a_to_b))
}

/// `base` (already in `(x, y)` point form, no closing repeat), cut down by
/// every half-plane in `constraints` — one call per neighbour a ring is
/// currently found to overlap, all applied in one pass to the *same*
/// original `base` rather than to whatever the previous constraint left
/// behind.
///
/// That distinction is the whole reason this exists as a batch rather than
/// looping a single-constraint clip once per neighbour: intersecting a
/// convex `base` with `N` half-planes gives the same convex region whether
/// they're applied one at a time or all at once, *as long as every one of
/// them is measured against the same starting shape* — [`resolve_overlaps`]'s
/// own docs cover why re-deriving each new cut's own centroid from an
/// *already cut* shape doesn't have that property, and fragmented a zone
/// contested by several neighbours into a disconnected, multi-lobed mess
/// instead of the single clean intersection a busy corner's own waiting
/// area actually is.
///
/// How many times `ring`'s own boundary crosses the infinite line through
/// `point_on_line` perpendicular to `normal` — a vertex sitting exactly on
/// the line counts as *not* crossing there (an edge ending exactly on the
/// line still only flips sides once, at its other end), so this never
/// over-counts on the coincidental exact touches real reprojected geometry
/// occasionally produces. [`clip_by_constraints`]'s own gate on whether
/// [`clip_half_plane`]'s guarantee actually holds for a given `ring` and
/// cut — see its own docs.
fn boundary_crossings(ring: &[(f64, f64)], point_on_line: (f64, f64), normal: (f64, f64)) -> usize {
    let side = |p: (f64, f64)| (p.0 - point_on_line.0) * normal.0 + (p.1 - point_on_line.1) * normal.1;
    let n = ring.len();
    (0..n).filter(|&i| (side(ring[i]) > 0.0) != (side(ring[(i + 1) % n]) > 0.0)).count()
}

/// `clip_half_plane` is exact Sutherland-Hodgman clipping, which only
/// guarantees a correct single polygon back when the boundary it's cutting
/// crosses the clip line at most twice — true of a convex `base`, not
/// guaranteed of a real zone's own (a multi-lane `merged_zone_ring`
/// spanning a bend). Past that, the algorithm bridges what should be
/// separate pieces with a straight edge along the clip line — not just a
/// theoretical risk: on a real Barcelona ribbon whose own bend put a
/// contested neighbour's cut line across it four times rather than two,
/// this silently produced a small, entirely valid-looking (simple, even
/// convex) quadrilateral that was still *wrong* — a phantom sliver
/// bridging across the ribbon's own bend, covering neither real remaining
/// piece — passing both the area-inflation and self-intersection checks
/// this function used to rely on alone. `boundary_crossings` catches this
/// directly, the same guarantee [`clip_half_plane`]'s own docs name,
/// checked per constraint against `base` itself before trusting the direct
/// path at all, rather than only inspecting its output after the fact.
/// Clipping `base`'s own convex hull instead is guaranteed exact (a hull is
/// always convex by construction, so every cut against it crosses at most
/// twice) at the cost of claiming a little more than `base`'s own real,
/// possibly-concave area in trade.
fn clip_by_constraints(base: &[(f64, f64)], constraints: &[HalfPlane]) -> Vec<(f64, f64)> {
    let apply = |mut points: Vec<(f64, f64)>| -> Vec<(f64, f64)> {
        for &(point_on_line, normal) in constraints {
            if points.is_empty() {
                break;
            }
            points = clip_half_plane(&points, point_on_line, normal);
        }
        points
    };

    // A single constraint crossing `base`'s own boundary at most twice is
    // [`clip_half_plane`]'s own documented guarantee of a correct result,
    // true regardless of `base`'s own convexity. More than one constraint
    // at once is a different claim, though — this module's own docs argue
    // it's safe to measure every constraint against the same stable `base`
    // *because* "intersecting a convex base with N half-planes gives the
    // same convex region regardless of order" — a property that
    // specifically needs `base` convex to begin with. A `base` with even a
    // gentle real bend (not literally convex) can satisfy the two-crossing
    // rule for every constraint individually and still combine into a
    // small, entirely valid-looking (simple, even convex) result that
    // isn't really the intersection any of those neighbours actually
    // contest — a phantom sliver bridging across the bend instead of
    // either real remaining piece. Real Barcelona case this guards: an
    // 80.9m ribbon with one gentle bend, contested by three separate
    // neighbours near that same bend at once, collapsed to a disconnected
    // notch nowhere near any of the three real overlaps. A lone constraint
    // doesn't have another one to combine badly with, so it only needs the
    // ordinary crossing check.
    let safe_to_clip_directly = match constraints {
        [] => true,
        [(point_on_line, normal)] => boundary_crossings(base, *point_on_line, *normal) <= 2,
        _ => base.len() == convex_hull(base).len(),
    };

    if safe_to_clip_directly {
        let direct = apply(base.to_vec());
        let original_area = signed_area(base).abs();
        let direct_area = signed_area(&direct).abs();
        if direct_area <= original_area * 1.0001 && !polyline_self_intersects(&direct) {
            return direct;
        }
    }
    apply(convex_hull(base))
}

/// `points` (no closing repeat) as a closed GeoJSON ring — `None` if fewer
/// than 3 remain, meaning [`clip_by_constraints`] clipped it away almost
/// entirely.
fn close_points(points: Vec<(f64, f64)>) -> Option<Vec<Position>> {
    if points.len() < 3 {
        return None;
    }
    let mut ring: Vec<Position> = points.into_iter().map(|(x, y)| Position::from([x, y])).collect();
    let first = ring[0].clone();
    ring.push(first);
    Some(ring)
}

/// Whether closed polyline `points` (no repeated closing vertex needed —
/// unlike a GeoJSON [`Position`] ring) crosses itself anywhere — the same
/// "bowtie" check the test module's own `ring_self_intersects` runs on
/// finished [`Position`] rings, duplicated here rather than shared because
/// this one runs on plain tuples mid-computation, before there's a
/// [`Position`] ring to hand it at all. Used by [`clip_by_constraints`]'s
/// own safety check on a direct half-plane clip — see its own docs for why
/// that can fail on a non-convex ring.
fn polyline_self_intersects(points: &[(f64, f64)]) -> bool {
    let n = points.len();
    if n < 4 {
        return false;
    }
    let at = |i: usize| Point { x: points[i % n].0, y: points[i % n].1, z: 0.0 };
    for i in 0..n {
        for j in (i + 2)..n {
            if i == 0 && j == n - 1 {
                continue; // adjacent via the closing wrap-around
            }
            if segment_intersection_point(at(i), at(i + 1), at(j), at(j + 1)).is_some() {
                return true;
            }
        }
    }
    false
}

/// The convex hull of `points` (Andrew's monotone chain), wound
/// counterclockwise. [`clip_by_constraints`]'s own fallback clip subject
/// for a ring a direct [`clip_half_plane`] sequence can't be trusted on —
/// see its own docs.
fn convex_hull(points: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let mut sorted = points.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).expect("network coordinates are always finite"));
    sorted.dedup();
    if sorted.len() < 3 {
        return sorted;
    }

    let cross = |o: (f64, f64), a: (f64, f64), b: (f64, f64)| (a.0 - o.0) * (b.1 - o.1) - (a.1 - o.1) * (b.0 - o.0);
    let half = |points: &[(f64, f64)]| -> Vec<(f64, f64)> {
        let mut hull: Vec<(f64, f64)> = Vec::new();
        for &p in points {
            while hull.len() >= 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0 {
                hull.pop();
            }
            hull.push(p);
        }
        hull
    };

    let mut lower = half(&sorted);
    sorted.reverse();
    let mut upper = half(&sorted);
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// Overwrites the exterior ring of `feature`'s `polygon_index`-th
/// sub-polygon in place — a no-op if `feature` isn't a `MultiPolygon` or
/// doesn't have that many sub-polygons (never true of anything
/// [`resolve_overlaps`] itself calls this on).
fn replace_ring(feature: &mut Feature, polygon_index: usize, ring: Vec<Position>) {
    if let Some(geometry) = feature.geometry.as_mut()
        && let geojson::GeometryValue::MultiPolygon { coordinates } = &mut geometry.value
        && let Some(polygon) = coordinates.get_mut(polygon_index)
        && let Some(exterior) = polygon.get_mut(0)
    {
        *exterior = ring;
    }
}

/// Whether any ring of `a` overlaps any ring of `b` — [`overlapping_ring_indices`]'s
/// own pairwise test, applied to two `Feature`s directly rather than
/// indices into a shared slice; [`resolve_overlaps`]'s own padding-drop
/// phase uses this to check a candidate replacement (the unpadded variant
/// of one zone) against the *other* zone's current geometry without first
/// having to splice the candidate into the shared `features` slice just to
/// ask the question.
fn ring_lists_overlap(a: &Feature, b: &Feature) -> bool {
    let (rings_a, rings_b) = (feature_rings(a), feature_rings(b));
    rings_a.iter().any(|ring_a| rings_b.iter().any(|ring_b| polygons_overlap(ring_a, ring_b)))
}

/// Past this many outer rounds of [`resolve_overlaps`], give up expanding
/// its own candidate set and leave whatever's left overlapping rather than
/// looping forever — real Barcelona data converges in 2 rounds, so this is
/// headroom for a much messier network, not a figure anything is tuned
/// against.
const MAX_RESOLUTION_ROUNDS: u32 = 8;

/// How many bisection steps [`max_safe_pad`] runs — 10 halvings of
/// [`MIN_DRAWN_LANE_LENGTH_METERS`]'s own 5m bottom out under a
/// millimetre, far tighter than the overlap-area noise floor
/// ([`OVERLAP_AREA_THRESHOLD_DEG2`]) could even distinguish, so more
/// steps would only buy precision nothing downstream can tell apart from
/// what this already gives.
const PAD_SEARCH_STEPS: u32 = 10;

/// The most [`zone_feature`] can pad `zone`'s own entries out to, up to
/// `upper`, without the result overlapping `opposing` — binary search
/// rather than the all-or-nothing choice between `upper` and `0.0`:
/// dropping straight to zero the instant the full default overlaps
/// anything turns a real but merely-a-bit-too-short lane back into the
/// paper-thin sliver [`MIN_DRAWN_LANE_LENGTH_METERS`] exists to avoid,
/// when often only a metre or two of it was ever the problem.
///
/// Assumes overlap is monotonic in the pad amount — true by construction,
/// since [`padded_entry`] only ever extends an entry *backward* along a
/// fixed line as the target length grows, never sideways or in any other
/// direction that could newly clear an obstruction a smaller pad had
/// already reached. If even `0.0` overlaps `opposing`, this converges to
/// `0.0` and reports it rather than special-casing that check up front:
/// the padding isn't the problem in that case, which is exactly what
/// [`resolve_overlaps`]'s own bisecting phase is for.
fn max_safe_pad(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    reproject: &Reprojector,
    upper: f64,
    opposing: &Feature,
) -> Result<f64> {
    let overlaps_at = |pad: f64| -> Result<bool> {
        Ok(ring_lists_overlap(&zone_feature(zone, lanes, successors, reproject, pad)?, opposing))
    };
    if !overlaps_at(upper)? {
        return Ok(upper);
    }

    let (mut safe, mut unsafe_) = (0.0, upper);
    for _ in 0..PAD_SEARCH_STEPS {
        let mid = (safe + unsafe_) / 2.0;
        if overlaps_at(mid)? { unsafe_ = mid } else { safe = mid }
    }
    Ok(safe)
}

/// Fixes up `features` (already built by [`zone_feature`] with padding on,
/// one-to-one with `zones`) so no two different zones' polygons overlap —
/// see [`overlapping_zone_ids`]'s own docs for why that has to hold for the
/// client-facing output. A no-op, and cheap, for the overwhelming majority
/// of real networks where nothing overlaps in the first place: the one
/// full `O(n²)` pass below is exactly [`overlapping_zone_ids`]'s own cost,
/// paid once regardless of whether it finds anything.
///
/// Two independent fixes, tried in order:
///
/// 1. **Shrink padding.** [`padded_entry`]'s own straight-line
///    extrapolation is a heuristic, and a wrong one often enough in
///    practice to be worth checking rather than trusting outright — real
///    Barcelona data hits this for the majority of overlaps found while
///    writing this. Tried first because it only ever gives back slack
///    `zone_feature` added for looks, never ground the zone's own detector
///    gates actually claim. Not all-or-nothing: dropping straight to zero
///    the moment the full default overlaps anything turns a real but
///    short lane back into the paper-thin sliver [`MIN_DRAWN_LANE_LENGTH_METERS`]
///    exists to avoid, when often only a metre or two of the padding was
///    ever the problem. [`max_safe_pad`] binary-searches for the most
///    padding that still avoids the specific neighbour a zone was found
///    overlapping, and only bottoms out at zero when even that doesn't
///    help — a genuine, non-padding overlap for bisecting to handle
///    instead.
/// 2. **Bisect.** Whatever's left over is a genuine geometric adjacency —
///    real lane width, a sharp fork — that shrinking padding can't touch.
///
/// A ring contested by *several* neighbours at once (a busy corner where
/// three or four zones all reach for the same ground) collects one
/// [`bisecting_half_plane`] constraint per neighbour, but
/// [`clip_by_constraints`] applies all of them **in one pass, against the
/// ring's own original shape** — never one at a time against whatever the
/// previous cut left behind. The first version of this function did the
/// latter, and on a real Barcelona corner contested by seven different
/// neighbours it fragmented that one zone into a disconnected, multi-lobed
/// shred rather than the single clean intersection the corner's own
/// waiting area actually is: each successive cut re-derived its own
/// centroid from an already-mangled shape, so the cuts didn't compose into
/// anything coherent. Measuring every constraint against the same stable
/// base (`base_rings`, captured once per ring, right after its own
/// padding decision) avoids that entirely — intersecting a convex shape
/// with `N` half-planes is the same convex region regardless of what order
/// they're applied in, as long as all `N` are measured against that one
/// shape.
fn resolve_overlaps(
    zones: &[E3Detector],
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    reproject: &Reprojector,
    features: &mut [Feature],
) -> Result<()> {
    // Every zone's current padding target, in metres — absent means still
    // at the full [`MIN_DRAWN_LANE_LENGTH_METERS`] default, never having
    // needed shrinking. [`max_safe_pad`] only ever lowers an entry, never
    // raises one back up: a zone that no longer overlaps anything once its
    // padding is smaller stays that size rather than creeping back toward
    // the look-nicer default, since re-growing it could just reopen the
    // same overlap on a later round.
    let mut pad_meters: HashMap<usize, f64> = HashMap::new();

    // Every ring's own stable base for bisecting (see this function's own
    // docs) — captured the first time that ring is seen, i.e. right after
    // its zone's padding decision above, before any bisecting has touched
    // it — and the accumulated constraints clipping it down, keyed by
    // *which other ring* each one came from so the same neighbour is never
    // double-counted across rounds.
    let mut base_rings: HashMap<(usize, usize), Vec<(f64, f64)>> = HashMap::new();
    let mut constraints: HashMap<(usize, usize), HashMap<(usize, usize), HalfPlane>> = HashMap::new();
    let mut warned_coincident: HashSet<((usize, usize), (usize, usize))> = HashSet::new();

    // The outer round exists because neither fix below is proven to
    // *strictly* shrink a ring in every case `clip_half_plane` can face —
    // see `clip_by_constraints`'s own docs on the one shape of non-convex
    // ring even its convex-hull fallback can't fully rule out reaching
    // slightly further than the original. Rather than try to prove that
    // bound tighter, this just checks: a full, unrestricted
    // `overlapping_indices` after each round is exactly as authoritative
    // as `overlapping_zone_ids` itself, so if anything — including a zone
    // neither fix above ever touched — still overlaps, or newly does, it's
    // picked up here and folded into the next round's own candidate set.
    // Converges as long as *some* pair keeps shrinking each round; a
    // genuinely stuck pair (two coincident centroids, say) just stops
    // making progress and the loop exits below rather than spinning on it
    // forever.
    for _round in 0..MAX_RESOLUTION_ROUNDS {
        let remaining = overlapping_indices(features, None);
        if remaining.is_empty() {
            return Ok(());
        }
        let candidates: BTreeSet<usize> = remaining.iter().flat_map(|&(i, j)| [i, j]).collect();

        // Every zone whose padding actually changed *this round* —
        // tracked separately from `pad_meters` itself (which only says the
        // *current* target, not whether it just moved) because a zone can
        // carry bisect constraints from an *earlier* round, computed and
        // applied against whatever its shape was back then. Shrinking
        // padding here replaces `features[i]` outright with the newly
        // rebuilt geometry, which would silently erase those earlier clips
        // if nothing forced them to be re-applied — the bug that left two
        // real Barcelona pairs overlapping the first time this function
        // ran this fixture: their shared zone got its padding changed
        // *after* already being bisected once, and the stale `base_rings`
        // entry from that earlier round never got refreshed.
        let mut pad_changed: HashSet<usize> = HashSet::new();

        loop {
            let overlaps = overlapping_indices(features, Some(&candidates));
            if overlaps.is_empty() {
                break;
            }
            let mut progressed = false;
            for (i, j) in overlaps {
                let current = pad_meters.get(&i).copied().unwrap_or(MIN_DRAWN_LANE_LENGTH_METERS);
                if current > 0.0 {
                    let best = max_safe_pad(&zones[i], lanes, successors, reproject, current, &features[j])?;
                    if best < current {
                        features[i] = zone_feature(&zones[i], lanes, successors, reproject, best)?;
                        pad_meters.insert(i, best);
                        pad_changed.insert(i);
                        progressed = true;
                        continue;
                    }
                }
                let current = pad_meters.get(&j).copied().unwrap_or(MIN_DRAWN_LANE_LENGTH_METERS);
                if current > 0.0 {
                    let best = max_safe_pad(&zones[j], lanes, successors, reproject, current, &features[i])?;
                    if best < current {
                        features[j] = zone_feature(&zones[j], lanes, successors, reproject, best)?;
                        pad_meters.insert(j, best);
                        pad_changed.insert(j);
                        progressed = true;
                    }
                }
            }
            if !progressed {
                break;
            }
        }

        // Invalidate any stale base captured for a zone before its padding
        // just changed above, and force every ring of its that already
        // carries a bisect constraint back into `touched` below so that
        // constraint gets re-applied against the fresh base — otherwise it
        // simply vanishes along with the stale base it was computed
        // against.
        let mut forced_touch: BTreeSet<(usize, usize)> = BTreeSet::new();
        for &i in &pad_changed {
            for (_, r) in constraints.keys().filter(|&&(zone, _)| zone == i).copied().collect::<Vec<_>>() {
                base_rings.remove(&(i, r));
                forced_touch.insert((i, r));
            }
        }

        for &i in &candidates {
            for (r, ring) in feature_rings(&features[i]).into_iter().enumerate() {
                base_rings.entry((i, r)).or_insert_with(|| {
                    ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect()
                });
            }
        }

        let mut touched: BTreeSet<(usize, usize)> = forced_touch;
        for (zi, ri, zj, rj) in overlapping_ring_indices(features, &candidates) {
            if constraints.get(&(zi, ri)).is_some_and(|by| by.contains_key(&(zj, rj))) {
                continue; // already constrained against this exact neighbour
            }
            let ring_a = feature_rings(&features[zi])[ri];
            let ring_b = feature_rings(&features[zj])[rj];
            let Some((point, normal)) = bisecting_half_plane(ring_a, ring_b) else {
                if warned_coincident.insert(((zi, ri), (zj, rj))) {
                    eprintln!(
                        "{ERROR}error:{ERROR:#} zones {:?} and {:?} overlap with (nearly) \
                         coincident centroids — no direction to split them along, left \
                         overlapping",
                        features[zi].property("waiting_zone_id"),
                        features[zj].property("waiting_zone_id"),
                    );
                }
                continue;
            };
            constraints.entry((zi, ri)).or_default().insert((zj, rj), (point, normal));
            constraints.entry((zj, rj)).or_default().insert((zi, ri), (point, (-normal.0, -normal.1)));
            touched.insert((zi, ri));
            touched.insert((zj, rj));
        }

        if touched.is_empty() {
            break; // nothing left that a new constraint could resolve
        }

        for (zone, ring) in touched {
            let cuts: Vec<HalfPlane> = constraints[&(zone, ring)].values().copied().collect();
            let clipped = clip_by_constraints(&base_rings[&(zone, ring)], &cuts);
            match close_points(clipped) {
                Some(new_ring) => replace_ring(&mut features[zone], ring, new_ring),
                None => eprintln!(
                    "{ERROR}error:{ERROR:#} zone {:?}'s own waiting area was clipped away \
                     entirely by {} contesting neighbour(s) — left as it was before this round",
                    features[zone].property("waiting_zone_id"),
                    cuts.len(),
                ),
            }
        }
    }

    for (i, j) in overlapping_indices(features, None) {
        eprintln!(
            "{ERROR}error:{ERROR:#} zones {:?} and {:?} still overlap after \
             {MAX_RESOLUTION_ROUNDS} resolution rounds — left as-is",
            features[i].property("waiting_zone_id"),
            features[j].property("waiting_zone_id"),
        );
    }
    Ok(())
}

/// The average of `points`, in the network's own local coordinates — a
/// single representative point for a zone's `stop_line`, even when the zone
/// spans several lanes (and so several individual stop lines).
fn centroid(points: &[Point]) -> Point {
    let count = points.len().max(1) as f64;
    let sum = points.iter().fold(Point::default(), |acc, p| Point {
        x: acc.x + p.x,
        y: acc.y + p.y,
        z: acc.z + p.z,
    });
    Point {
        x: sum.x / count,
        y: sum.y / count,
        z: sum.z / count,
    }
}

/// `zone`'s own `modes` property: which of `connection.md`'s four declared
/// travel modes (`CAR`, `MOTORCYCLE`, `BICYCLE`, `ON_FOOT` —
/// `events::enums::Mode`'s own wire vocabulary; spelled out as bare
/// `&'static str`s here rather than adding a dependency on `events` from a
/// crate that otherwise knows nothing about the MQTT/session protocol) may
/// actually wait here.
///
/// Deliberately coarser than [`sumo_types::domain::VClass`]'s own
/// fine-grained vClass distinctions (`Passenger` vs. `Motorcycle` vs. `Bus`
/// vs. `Taxi`, ...): a `.net.xml`'s `allow`/`disallow` can in principle
/// restrict a lane to any one of those, but the road infrastructure it's
/// actually modelling never creates a *waiting-zone* this fine — a real
/// junction's own queue is either pedestrians, a dedicated bike lane, or
/// everyone else driving, never a separate car-only or bus-only queue.
/// Encoding vClass fidelity this crate has no real distinction to spend it
/// on would produce a `modes` property that looks precise but claims a
/// granularity the underlying data was never modelling in the first place —
/// this only ever returns one of three shapes:
///
/// - `["ON_FOOT"]` for a pedestrian zone ([`E3Detector::detect_persons`]
///   non-empty — `zone_generator::pedestrian_zones`'s own marker, checked
///   directly rather than re-parsing it back out of the id).
/// - `["BICYCLE"]` for a zone whose every lane is a *dedicated* bike lane —
///   permits [`VClass::Bicycle`] but not [`VClass::Passenger`]. Checked
///   against `zone.exits`' own lanes — never `zone.entries`, which can
///   include an extended ancestor several hops back on a *different* edge
///   (`zone_generator::extended_entry_lanes`'s own docs) — on the same
///   "exits are always the group's own controlled lane, entries aren't
///   guaranteed to be" principle `zone_generator::pedestrian_zones`'s own
///   docs already lean on. *Every* lane of the zone has to qualify, not
///   just one: a zone can span several parallel lanes of one movement
///   (`merged_zone_ring`'s own docs), and a car lane sitting right next to
///   a bike lane in the same group is still a car lane sharing that
///   physical queue, not a dedicated bike facility as a whole.
/// - `["CAR", "MOTORCYCLE"]` for every other vehicle zone — the two are
///   never modeled apart, because nothing in this crate's data ever
///   separates them either: real Barcelona `.net.xml` output never
///   restricts a lane to one but not the other, so there is no
///   infrastructure-backed distinction to draw. A lane whose own vClass
///   permissions happen to exclude ordinary cars too (e.g. a bus-only
///   lane) still counts as this generic vehicle case rather than admitting
///   nothing at all — see the module docs above.
fn zone_modes(zone: &E3Detector, lanes: &HashMap<&str, &Lane>) -> Vec<&'static str> {
    if !zone.detect_persons.is_empty() {
        return vec!["ON_FOOT"];
    }

    let zone_lanes: Vec<&Lane> = zone
        .exits
        .iter()
        .filter_map(|exit| lanes.get(exit.lane.0.as_str()).copied())
        .collect();
    let dedicated_bike_lane = !zone_lanes.is_empty()
        && zone_lanes
            .iter()
            .all(|lane| lane.permits(VClass::Bicycle) && !lane.permits(VClass::Passenger));

    if dedicated_bike_lane {
        vec!["BICYCLE"]
    } else {
        vec!["CAR", "MOTORCYCLE"]
    }
}

/// For every real lane with exactly one real (non-internal) outgoing
/// connection, that connection's own destination lane and `via` (if any) —
/// independently re-derived from `network`'s own connections here rather
/// than reusing `zone_generator`'s own private connectivity graph, so this
/// module doesn't need to depend on that crate's internals for it.
///
/// Never ambiguous for a lane [`zone_feature`] actually looks this up
/// for: every extended-ancestor entry it ever sees is one
/// `zone_generator::extended_entry_lanes` already proved has exactly one
/// real successor, by this exact "single outgoing connection" rule —
/// re-deriving it from the same `.net.xml` data can only ever agree with
/// what that walk found.
fn single_successors(network: &Network) -> HashMap<&str, (&str, Option<&str>)> {
    let internal_edges: HashSet<&EdgeId> = network
        .edges
        .iter()
        .filter(|edge| edge.function == EdgeFunction::Internal)
        .map(|edge| &edge.id)
        .collect();
    let lane_id_by_edge_and_index: HashMap<(&EdgeId, LaneIndex), &str> = network
        .edges
        .iter()
        .flat_map(|edge| edge.lanes.iter().map(move |lane| ((&edge.id, lane.index), lane.id.0.as_str())))
        .collect();

    let mut successors_by_from: HashMap<&str, Vec<(&str, Option<&str>)>> = HashMap::new();
    for connection in &network.connections {
        if internal_edges.contains(&connection.from_edge) || internal_edges.contains(&connection.to_edge) {
            continue;
        }
        let (Some(&from_lane), Some(&to_lane)) = (
            lane_id_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane)),
            lane_id_by_edge_and_index.get(&(&connection.to_edge, connection.to_lane)),
        ) else {
            continue;
        };
        let via = connection.via.as_ref().map(|via| via.0.as_str());
        successors_by_from.entry(from_lane).or_default().push((to_lane, via));
    }

    successors_by_from
        .into_iter()
        .filter_map(|(from, successors)| {
            let distinct: HashSet<&str> = successors.iter().map(|&(lane, _)| lane).collect();
            (distinct.len() == 1).then(|| (from, successors[0]))
        })
        .collect()
}

/// One extended-ancestor *segment*'s own combined shape: `start`'s real
/// lane, followed by every further lane and `via` [`single_successors`]
/// finds walking forward from it, up to (but never including) whichever
/// comes first of: the first lane that isn't itself one of
/// `ancestor_lanes` (the zone's own controlled lane, where
/// [`merged_zone_ring`] takes over instead), or another real merge point
/// (`in_degree(successor) != 1` — more than one ancestor feeding forward
/// into it, or, for a `start` that's itself past the last real fork,
/// none at all). The final `via` bridging into whichever one stops the
/// walk *is* included, so the segment's own polygon reaches exactly to
/// where the next one begins, with no gap between them, matching every
/// other hop along the way.
///
/// This is what turns what used to be one independent rectangle per
/// extended-ancestor lane into a single seamless polygon (via
/// [`shape_ring`]) spanning the whole segment — real Barcelona data has a
/// visible gap between two such rectangles at a bend often enough that
/// this exists: `offset_boundary`/`trimmed_segments` already handle a
/// single lane's own multi-point bend correctly, and a run of lanes
/// physically connected end to end is no different, once their shapes are
/// concatenated into one.
///
/// Stopping at a merge point rather than walking through it isn't just
/// about avoiding redundant work: a zone with two branches merging (a real
/// street `Y` — see this module's own overlap-resolution history) used to
/// have *both* branches' own segments walk all the way to the core,
/// meaning the shared ground downstream of the merge got drawn twice, once
/// per branch. Leaflet's own default SVG fill rule for a `MultiPolygon`
/// (`evenodd`) treats a point enclosed by an *even* number of the
/// feature's own rings as outside — so that doubly-covered stretch
/// rendered as a hole, a real, visible defect this crate's own overlap
/// checker had no way to catch (it only ever compares *different* zones,
/// never a zone's own rings against each other — see
/// [`overlapping_zone_ids`]'s own docs). Stopping every segment at the
/// next merge point instead means the shared ground is drawn by exactly
/// one segment, never more.
///
/// `visited` guards the same pathological cycle
/// `zone_generator::extended_entry_lanes`'s own `visited` set guards
/// against; a normal segment never revisits a lane.
fn chain_shape<'a>(
    start: &'a str,
    ancestor_lanes: &BTreeSet<&str>,
    in_degree: &HashMap<&str, usize>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    lanes: &HashMap<&str, &'a Lane>,
) -> Option<(Vec<&'a Lane>, Shape)> {
    let mut chain_lanes: Vec<&Lane> = Vec::new();
    let mut points: Vec<Point> = Vec::new();
    let mut visited: HashSet<&str> = HashSet::new();
    let mut current = start;

    loop {
        if !visited.insert(current) {
            break;
        }
        let Some(&lane) = lanes.get(current) else { break };
        chain_lanes.push(lane);
        points.extend(lane.shape.0.iter().copied());

        let Some(&(successor, via)) = successors.get(current) else { break };
        if let Some(via_id) = via
            && let Some(&via_lane) = lanes.get(via_id)
        {
            points.extend(via_lane.shape.0.iter().copied());
        }
        // Stop at the zone's own controlled lane (not an ancestor at all)
        // *or* at another real merge point (`in_degree != 1`) -- that one
        // belongs to its own segment, built once from its own call here,
        // not duplicated into this one too. See this function's own docs
        // on why duplicating it used to be actively wrong, not just
        // wasteful.
        if !ancestor_lanes.contains(successor) || in_degree.get(successor).copied().unwrap_or(0) != 1 {
            break;
        }
        current = successor;
    }

    (points.len() >= 2).then_some((chain_lanes, Shape(points)))
}

/// Builds `zone`'s `Feature`: a `MultiPolygon` geometry and a
/// `waiting_zone_id`/`stop_line`/`modes` triple of properties.
///
/// The geometry is no longer always exactly one ring:
/// `zone_generator::extended_entry_lanes` can add entry gates on ancestor
/// lanes several hops back from the zone's own controlled lane(s) (see its
/// own docs — "maximizing a waiting zone's own physical size"), and those
/// ancestors are, in general, lanes of a *different* edge —
/// [`merged_zone_ring`]'s own leftmost/rightmost-lane trick only holds for
/// lanes of the *same* edge, so it can't be trusted to merge across that
/// boundary. `entries` and `exits` are no longer 1:1 for the same reason
/// (extension only adds entries, never exits) — matched back up here by
/// lane: an entry whose lane is one of `zone`'s own exits is "core"
/// (possibly `max_zone_length`-capped, but otherwise unchanged from before
/// extension) and merges into one ring with the others like it; every
/// other entry is an extended ancestor, grouped into whichever
/// [`chain_shape`] it belongs to (leaf-to-core order, following
/// `successors`) and drawn as *that chain's own* single seamless polygon
/// rather than one independent rectangle per lane — a chain that turns out
/// to be one lane long (no real predecessor within this zone at all) still
/// gets its own polygon; there's just nothing else to concatenate onto it.
///
/// `pad_meters` is the target [`padded_entry`] pads every entry span out
/// to — a whole chain's own *total* length, not each lane in it
/// separately: [`MIN_DRAWN_LANE_LENGTH_METERS`] builds the normal, looks-
/// nicer-on-a-map shape [`to_feature_collection`] uses by default, `0.0`
/// builds the same zone at its real, unpadded size, and anything in
/// between is [`resolve_overlaps`]'s own way of asking for as much padding
/// as still fits without reaching into a neighbour — see its own docs for
/// why that's better than jumping straight to `0.0` the first time the
/// full default overlaps something.
fn zone_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    reproject: &Reprojector,
    pad_meters: f64,
) -> Result<Feature> {
    let resolve = |lane_ref: &sumo_types::additional::domain::LaneRef| {
        lanes.get(lane_ref.0.as_str()).copied().with_context(|| {
            format!(
                "zone {:?} references lane {:?}, which isn't in the network",
                zone.id, lane_ref
            )
        })
    };
    let target_length = Length::new::<meter>(pad_meters);
    let entry_span = |entry: Length, exit: Length| padded_entry(entry, exit, target_length);

    let mut exit_position_by_lane: HashMap<&str, Length> = HashMap::with_capacity(zone.exits.len());
    let mut stop_points = Vec::with_capacity(zone.exits.len());
    for exit in &zone.exits {
        let lane = resolve(&exit.lane)?;
        let exit_distance = distance_from_start(exit.position, lane.length);
        exit_position_by_lane.insert(exit.lane.0.as_str(), exit_distance);
        stop_points.push(point_and_tangent_at(&lane.shape, exit_distance).0);
    }

    let mut core_gates = Vec::with_capacity(zone.exits.len());
    let mut ancestor_lanes: BTreeSet<&str> = BTreeSet::new();
    for entry in &zone.entries {
        let lane = resolve(&entry.lane)?;
        let entry_distance = distance_from_start(entry.position, lane.length);

        if let Some(&exit_distance) = exit_position_by_lane.get(entry.lane.0.as_str()) {
            let entry_distance = entry_span(entry_distance, exit_distance);
            core_gates.push((lane, entry_distance, exit_distance));
        } else if !entry.lane.0.starts_with(':') {
            // `zone_generator::extended_entry_lanes` adds every hop's own
            // bridging `via` as a *separate* flat entry too (real
            // `.net.xml` internal lanes always start with `:`, SUMO's own
            // convention) -- correct for the E3Detector itself, which
            // needs every gate listed regardless of what it bridges, but
            // not a leaf to build a *chain* from here: `chain_shape`
            // already finds and includes each hop's own via by walking
            // `successors`, so treating a via lane as its own ancestor
            // too would draw the exact same ground twice, as its own
            // redundant sliver alongside the real chain that already
            // covers it.
            ancestor_lanes.insert(entry.lane.0.as_str());
        }
    }
    // A `BTreeSet`, not a `HashSet`: `segment_starts` below iterates this
    // to decide the *order* polygons are pushed, which fixes each ring's
    // index within the finished `MultiPolygon` — `resolve_overlaps` keys
    // its own per-round state off those indices, so a `HashSet`'s
    // run-to-run-random iteration order would make which specific pair (if
    // any) is still overlapping after `MAX_RESOLUTION_ROUNDS` change
    // between otherwise-identical runs on the same input.

    let mut polygons = merged_zone_ring(&zone.id, &core_gates, reproject)?
        .map(|ring| vec![vec![ring]])
        .unwrap_or_default();

    // How many *other* ancestors feed forward into each ancestor lane —
    // `chain_shape`'s own segmentation depends on this, not just on
    // finding leaves: a lane with more than one real predecessor (a real
    // street merge, `resolve_overlaps`'s own module docs) has to start its
    // *own* segment rather than being swept into either predecessor's,
    // exactly as much as a leaf (nothing feeding into it at all) does —
    // see `chain_shape`'s own docs for why building it a second time, once
    // per predecessor, was actively wrong rather than merely redundant.
    let mut in_degree: HashMap<&str, usize> = HashMap::new();
    for &lane in &ancestor_lanes {
        if let Some(&(successor, _)) = successors.get(lane)
            && ancestor_lanes.contains(successor)
        {
            *in_degree.entry(successor).or_insert(0) += 1;
        }
    }
    let segment_starts =
        ancestor_lanes.iter().filter(|lane| in_degree.get(*lane).copied().unwrap_or(0) != 1);
    for &start in segment_starts {
        let Some((chain_lanes, shape)) = chain_shape(start, &ancestor_lanes, &in_degree, successors, lanes)
        else {
            continue;
        };
        let total_length = shape_length(&shape);
        let entry_distance = entry_span(Length::new::<meter>(0.0), total_length);
        let half_width = chain_lanes[0].width / 2.0;
        polygons.push(vec![shape_ring(&shape, entry_distance, total_length, half_width, reproject)?]);
    }

    let stop_line = reproject.to_lon_lat(centroid(&stop_points))?;

    let mut properties = JsonObject::new();
    properties.insert("waiting_zone_id".to_string(), zone.id.0.clone().into());
    properties.insert("stop_line".to_string(), serde_json::json!(stop_line));
    properties.insert(
        "modes".to_string(),
        serde_json::json!(zone_modes(zone, lanes)),
    );

    let mut feature = Feature::from(Geometry::new_multi_polygon(polygons));
    feature.properties = Some(properties);
    Ok(feature)
}

/// Converts `zones` (as generated by [`crate::zone_generator`] from
/// `network`) into a GeoJSON `FeatureCollection`, one feature per zone —
/// guaranteed free of the overlaps [`overlapping_zone_ids`] checks for
/// (see [`resolve_overlaps`]), not just built and left for a caller to
/// check separately: the client-facing pipeline (`write`, below) has
/// exactly one chance to get this right before it ships.
///
/// Fails if `network` isn't georeferenced (see the module docs) or if a
/// zone references a lane `network` doesn't have — the latter would mean
/// `zones` wasn't actually generated from this `network`.
pub fn to_feature_collection(network: &Network, zones: &[E3Detector]) -> Result<FeatureCollection> {
    let reproject = Reprojector::new(&network.location)?;
    let lanes: HashMap<&str, &Lane> = network
        .edges
        .iter()
        .flat_map(|edge| &edge.lanes)
        .map(|lane| (lane.id.0.as_str(), lane))
        .collect();
    let successors = single_successors(network);

    let mut features = zones
        .iter()
        .map(|zone| zone_feature(zone, &lanes, &successors, &reproject, MIN_DRAWN_LANE_LENGTH_METERS))
        .collect::<Result<Vec<_>>>()?;

    resolve_overlaps(zones, &lanes, &successors, &reproject, &mut features)?;

    Ok(FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    })
}

/// Writes `zones` to `path` as a complete GeoJSON `FeatureCollection`.
pub fn write(path: &Path, network: &Network, zones: &[E3Detector]) -> Result<()> {
    let collection = to_feature_collection(network, zones)?;
    let json = serde_json::to_string_pretty(&collection)
        .context("could not serialize waiting zones as GeoJSON")?;
    std::fs::write(path, json).with_context(|| format!("could not write output file: {path:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sumo_types::additional::domain::{DetectorGate, DetectorId, LaneRef};
    use sumo_types::domain::{
        Boundary, Connection, ConnectionDirection, Edge, EdgeFunction, EdgeId, LaneId, LaneIndex, LinkState,
    };
    use sumo_types::uom::si::velocity::meter_per_second;

    /// A straight, north-pointing lane 20m long, its shape running from
    /// `(0, 0)` to `(0, 20)` in a UTM-like local CRS.
    fn straight_lane(id: &str, width_m: f64) -> Lane {
        parallel_lane(id, width_m, 0.0)
    }

    /// [`straight_lane`], shifted `x_offset_m` east — two of these `x_offset_m`
    /// apart produce two zones whose rectangles overlap when
    /// `x_offset_m < width_m` (their half-widths reach past the gap between
    /// centrelines) and don't when `x_offset_m` is comfortably larger.
    fn parallel_lane(id: &str, width_m: f64, x_offset_m: f64) -> Lane {
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(10.0),
            length: Length::new::<meter>(20.0),
            width: Length::new::<meter>(width_m),
            end_offset: Length::new::<meter>(0.0),
            shape: Shape(vec![
                Point {
                    x: x_offset_m,
                    y: 0.0,
                    z: 0.0,
                },
                Point {
                    x: x_offset_m,
                    y: 20.0,
                    z: 0.0,
                },
            ]),
            allow: vec![],
            disallow: vec![],
        }
    }

    /// [`parallel_lane`], with `index` set instead of the default 0 —
    /// needed for [`merged_zone_ring`]'s own tests, which care which lane is
    /// physically left/right of which, not just how far apart they are.
    fn indexed_parallel_lane(id: &str, index: usize, width_m: f64, x_offset_m: f64) -> Lane {
        Lane {
            index: LaneIndex(index),
            ..parallel_lane(id, width_m, x_offset_m)
        }
    }

    /// A straight, north-pointing lane `length_m` long — [`padded_entry`]'s
    /// own tests need a length shorter than [`MIN_DRAWN_LANE_LENGTH_METERS`],
    /// unlike every other lane helper here, which is always the fixed 20m.
    fn short_lane(id: &str, length_m: f64, width_m: f64) -> Lane {
        Lane {
            length: Length::new::<meter>(length_m),
            shape: Shape(vec![
                Point { x: 0.0, y: 0.0, z: 0.0 },
                Point { x: 0.0, y: length_m, z: 0.0 },
            ]),
            ..parallel_lane(id, width_m, 0.0)
        }
    }

    fn utm_31n_network(lanes: Vec<Lane>) -> Network {
        Network {
            location: Location {
                // Chosen so the local shape above (x in [0, 0], y in [0,
                // 20]) sits near Barcelona rather than off the coast of
                // Ghana (UTM 31N's origin): offset applied on top of a real
                // UTM 31N easting/northing for a point close to it.
                net_offset: Point {
                    x: -430_000.0,
                    y: -4_582_000.0,
                    z: 0.0,
                },
                converted_boundary: Boundary::default(),
                original_boundary: Boundary::default(),
                projection: Projection::Proj4(
                    "+proj=utm +zone=31 +ellps=WGS84 +datum=WGS84 +units=m +no_defs".into(),
                ),
            },
            edges: vec![Edge {
                id: EdgeId("e0".into()),
                function: EdgeFunction::Normal,
                from: None,
                to: None,
                name: None,
                priority: None,
                length: None,
                shape: None,
                spread_type: None,
                lanes,
            }],
            junctions: vec![],
            connections: vec![],
            roundabouts: vec![],
            traffic_light_programs: vec![],
        }
    }

    /// [`utm_31n_network`], but with `edges` (each `(edge_id, lanes)`)
    /// instead of a single fixed `"e0"` — needed for
    /// [`zone_feature`]'s own tests, where an extended ancestor entry
    /// (see its own docs) has to be a lane of a genuinely different edge
    /// from the zone's own controlled one.
    fn utm_31n_network_multi_edge(edges: Vec<(&str, Vec<Lane>)>) -> Network {
        Network {
            edges: edges
                .into_iter()
                .map(|(id, lanes)| Edge {
                    id: EdgeId(id.into()),
                    function: EdgeFunction::Normal,
                    from: None,
                    to: None,
                    name: None,
                    priority: None,
                    length: None,
                    shape: None,
                    spread_type: None,
                    lanes,
                })
                .collect(),
            ..utm_31n_network(vec![])
        }
    }

    /// A real, single-lane (`fromLane`/`toLane` both `0`) `<connection>`
    /// from `from_edge` to `to_edge`, optionally bridged by an internal
    /// `via` lane — [`single_successors`]'s own tests need real
    /// connections to re-derive chain adjacency from, unlike every other
    /// fixture here, which leaves `Network::connections` empty since
    /// nothing before this needed it.
    fn plain_connection(from_edge: &str, to_edge: &str, via: Option<&str>) -> Connection {
        Connection {
            from_edge: EdgeId(from_edge.into()),
            to_edge: EdgeId(to_edge.into()),
            from_lane: LaneIndex(0),
            to_lane: LaneIndex(0),
            direction: ConnectionDirection::Straight,
            state: LinkState::Major,
            via: via.map(|via| LaneId(via.into())),
            traffic_light: None,
            link_index: None,
            pass: false,
            keep_clear: true,
        }
    }

    fn gate(lane: &str, position: LanePosition) -> DetectorGate {
        DetectorGate {
            lane: LaneRef(lane.into()),
            position,
            friendly_position: Some(true),
        }
    }

    fn zone(id: &str, lane: &str) -> E3Detector {
        zone_spanning(id, lane, Length::new::<meter>(20.0))
    }

    /// [`zone`], with the exit gate at `exit` metres from the lane's start
    /// instead of the fixed 20m [`straight_lane`]'s own length happens to
    /// match — for a lane whose real length isn't 20m.
    fn zone_spanning(id: &str, lane: &str, exit: Length) -> E3Detector {
        E3Detector {
            id: DetectorId(id.into()),
            entries: vec![gate(
                lane,
                LanePosition::FromStart(Length::new::<meter>(0.0)),
            )],
            exits: vec![gate(lane, LanePosition::FromStart(exit))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        }
    }

    /// [`zone`], but spanning every lane in `lane_ids` at once — a zone
    /// whose waiting area covers more than one lane of the same edge, the
    /// shape [`merged_zone_ring`] exists for. Every lane's own entry/exit
    /// is the fixed `[0, 20]`m span [`straight_lane`]/[`parallel_lane`]
    /// both use.
    fn zone_multi(id: &str, lane_ids: &[&str]) -> E3Detector {
        E3Detector {
            id: DetectorId(id.into()),
            entries: lane_ids
                .iter()
                .map(|lane| gate(lane, LanePosition::FromStart(Length::new::<meter>(0.0))))
                .collect(),
            exits: lane_ids
                .iter()
                .map(|lane| gate(lane, LanePosition::FromStart(Length::new::<meter>(20.0))))
                .collect(),
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        }
    }

    #[test]
    fn rejects_an_unprojected_network() {
        let network = Network {
            location: Location::default(), // Projection::None
            ..utm_31n_network(vec![])
        };

        let err = to_feature_collection(&network, &[]).unwrap_err();
        assert!(err.to_string().contains("not georeferenced"), "{err}");
    }

    #[test]
    fn reprojects_the_lane_shape_into_a_plausible_lon_lat_box() {
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(collection.features.len(), 1);

        let feature = &collection.features[0];
        assert_eq!(
            feature.property("waiting_zone_id").unwrap(),
            &serde_json::json!("j0_0")
        );

        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &feature.geometry.as_ref().unwrap().value
        else {
            panic!(
                "expected a MultiPolygon geometry, got {:?}",
                feature.geometry
            );
        };
        assert_eq!(coordinates.len(), 1, "one lane -> one polygon");
        let ring = &coordinates[0][0];
        assert_eq!(ring.first(), ring.last(), "a linear ring must close");

        // Barcelona-ish: UTM 31N puts it around 2°E, 41°N.
        for position in ring {
            assert!(
                (1.0..3.0).contains(&position[0]),
                "lon {} out of range",
                position[0]
            );
            assert!(
                (40.0..42.0).contains(&position[1]),
                "lat {} out of range",
                position[1]
            );
        }

        let stop_line = feature.property("stop_line").unwrap().as_array().unwrap();
        assert!((1.0..3.0).contains(&stop_line[0].as_f64().unwrap()));
        assert!((40.0..42.0).contains(&stop_line[1].as_f64().unwrap()));
    }

    /// The `modes` a `Feature`'s own `"modes"` property lists, as plain
    /// `&str`s — shared by every `zone_modes` test below so each one reads
    /// as a one-line assertion instead of repeating the same unwrap chain.
    fn modes_of(feature: &Feature) -> Vec<String> {
        feature
            .property("modes")
            .unwrap()
            .as_array()
            .unwrap()
            .iter()
            .map(|m| m.as_str().unwrap().to_string())
            .collect()
    }

    #[test]
    fn a_default_lane_with_no_allow_or_disallow_is_a_generic_vehicle_zone() {
        // Permits everything, including bicycles -- but not a *dedicated*
        // bike lane (it permits cars too), so this is still the generic
        // CAR/MOTORCYCLE case, not BICYCLE.
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_dedicated_bike_lane_is_bicycle_only() {
        let lane = Lane {
            allow: vec!["bicycle".to_string()],
            ..straight_lane("e0_0", 3.2)
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["BICYCLE"]);
    }

    #[test]
    fn a_bus_only_lane_still_counts_as_a_generic_vehicle_zone() {
        // The infrastructure a `.net.xml` encodes never creates a
        // waiting-zone-level distinction this fine (see `zone_modes`'s own
        // docs) -- a bus-only restriction doesn't turn this into a third
        // kind of waiting zone or an empty admission, it's still the
        // generic driving case.
        let lane = Lane {
            allow: vec!["bus".to_string()],
            ..straight_lane("e0_0", 3.2)
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_lane_disallowing_bicycles_is_still_a_generic_vehicle_zone() {
        let lane = Lane {
            disallow: vec!["bicycle".to_string()],
            ..straight_lane("e0_0", 3.2)
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_pedestrian_zone_is_always_on_foot_regardless_of_its_lanes_own_vclass() {
        // `zone_modes` checks `detect_persons`, not the lane's own
        // permissions, for a pedestrian zone -- real walkingarea lanes
        // always carry `allow="pedestrian"` anyway (`sumo_types` doesn't
        // model anything finer there), but this pins that the *zone's* own
        // marker decides, not a lane lookup that would happen to agree.
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let ped_zone = E3Detector {
            detect_persons: vec!["walk".to_string()],
            ..zone("j0_0", "e0_0")
        };

        let collection = to_feature_collection(&network, &[ped_zone]).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["ON_FOOT"]);
    }

    #[test]
    fn a_multi_lane_zone_with_only_one_dedicated_bike_lane_is_still_a_generic_vehicle_zone() {
        // `e0_1` alone is a dedicated bike lane, but `e0_0` shares the same
        // physical queue and admits ordinary traffic too -- the group as a
        // whole isn't a dedicated bike facility, so this is CAR/MOTORCYCLE,
        // not BICYCLE. *Every* lane in the group has to be bike-dedicated
        // for the zone to count as one -- see the next test.
        let network = utm_31n_network(vec![
            indexed_parallel_lane("e0_0", 0, 3.2, 0.0),
            Lane {
                allow: vec!["bicycle".to_string()],
                ..indexed_parallel_lane("e0_1", 1, 3.2, 3.2)
            },
        ]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_1"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_multi_lane_zone_where_every_lane_is_a_dedicated_bike_lane_is_bicycle_only() {
        let bike_lane = |id: &str, index: usize, x_offset_m: f64| Lane {
            allow: vec!["bicycle".to_string()],
            ..indexed_parallel_lane(id, index, 3.2, x_offset_m)
        };
        let network = utm_31n_network(vec![bike_lane("e0_0", 0, 0.0), bike_lane("e0_1", 1, 3.2)]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_1"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["BICYCLE"]);
    }

    #[test]
    fn offsets_the_ring_by_roughly_half_the_lane_width() {
        // A lane running due north: the perpendicular offset is purely in
        // x (easting), so converting the resulting lon/lat difference back
        // to metres at this latitude should land close to `width / 2` on
        // each side -- not an exact match (that needs the local metres/degree
        // scale factor), but within the same order of magnitude for a
        // narrow lane.
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        // The lane runs due north, so the perpendicular offset is purely
        // in x (easting): the ring's own widest point-to-point longitude
        // spread is the full lane width, not just half of it — not tied to
        // any particular vertex's own index (self-intersection removal can
        // reorder/collapse points), just the shape's own extent.
        let lons = ring.iter().map(|p| p[0]);
        let lon_span = lons.clone().fold(f64::MIN, f64::max) - lons.fold(f64::MAX, f64::min);
        assert!(
            lon_span > 0.00001,
            "expected a visible offset, got {lon_span}"
        );
        assert!(
            lon_span < 0.001,
            "offset implausibly large for a 3.2m lane: {lon_span}"
        );
    }

    #[test]
    fn pads_a_lane_shorter_than_a_car_out_to_a_reasonable_minimum_length() {
        // e0 alone, 1m long -- shorter than any real vehicle, and with
        // nothing else in the network at all (no connections, so nothing
        // for `zone_generator::extended_entry_lanes` to have walked
        // through even if this fixture went through it). `padded_entry`
        // still pads it out by extrapolating its own first segment
        // backward in a straight line -- see its own docs for why that,
        // not a real neighbouring lane's own shape, is deliberate: a real
        // predecessor lane is, in general, ground a *different* zone's own
        // polygon already draws.
        let network = utm_31n_network(vec![short_lane("e0_0", 1.0, 3.2)]);
        let zones = vec![zone_spanning("j0_0", "e0_0", Length::new::<meter>(1.0))];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];

        // e0 runs due north, so its own length shows up as latitude span;
        // a bare 1m lane would span roughly 1m worth of latitude, nowhere
        // near MIN_DRAWN_LANE_LENGTH_METERS's own 5m.
        let lats = ring.iter().map(|p| p[1]);
        let lat_span_m =
            (lats.clone().fold(f64::MIN, f64::max) - lats.fold(f64::MAX, f64::min)) * 111_320.0;
        assert!(
            lat_span_m > MIN_DRAWN_LANE_LENGTH_METERS - 0.5,
            "expected the padded span to reach close to {MIN_DRAWN_LANE_LENGTH_METERS}m, \
             got {lat_span_m:.2}m"
        );
    }

    #[test]
    fn flags_two_zones_whose_rectangles_actually_overlap() {
        // 3.2m-wide lanes, centrelines 2m apart: each rectangle reaches
        // 1.6m either side of its own centreline, so they cover a shared
        // 1.6+1.6-2=1.2m-wide strip down the middle.
        //
        // Built directly via `zone_feature`, not `to_feature_collection`:
        // that function now resolves an overlap like this one itself (see
        // `resolve_overlaps`), so going through it here would test the
        // resolver, not the detector this test actually means to check in
        // isolation -- `to_feature_collection_bisects_...`, below, is the
        // one that exercises the resolver on this exact fixture.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 2.0),
        ]);
        let lanes: HashMap<&str, &Lane> =
            network.edges.iter().flat_map(|edge| &edge.lanes).map(|lane| (lane.id.0.as_str(), lane)).collect();
        let successors = single_successors(&network);
        let reproject = Reprojector::new(&network.location).unwrap();
        let collection = FeatureCollection {
            bbox: None,
            features: vec![
                zone_feature(&zone("j0_0", "e0_0"), &lanes, &successors, &reproject, MIN_DRAWN_LANE_LENGTH_METERS)
                    .unwrap(),
                zone_feature(&zone("j0_1", "e0_1"), &lanes, &successors, &reproject, MIN_DRAWN_LANE_LENGTH_METERS)
                    .unwrap(),
            ],
            foreign_members: None,
        };

        let overlaps = overlapping_zone_ids(&collection);
        assert_eq!(
            overlaps,
            vec![("j0_0".to_string(), "j0_1".to_string())],
            "these two rectangles are built to overlap by construction"
        );
    }

    #[test]
    fn to_feature_collection_shrinks_padding_just_enough_to_clear_a_real_neighbour() {
        // `e0_0` is a 2m-long north-pointing stub at x=0 -- short enough
        // that `padded_entry` extrapolates its southern end back to y=-3
        // (see `pads_a_lane_shorter_than_a_car_out_to_a_reasonable_minimum_length`).
        // `e1_0` is a real, full-length east-west lane whose own northern
        // edge sits at y=-0.4 (half its 3.2m width above its y=-2
        // centreline): unpadded, the two don't come anywhere near each
        // other (`e0_0`'s real span is y in [0, 2]), but the *full*
        // default pad (reaching to y=-3) overlaps it by a wide margin.
        // Anywhere in between -- down to y=-0.4, a pad of ~2.4m -- is
        // exactly as safe as no padding at all, which is what
        // `max_safe_pad`'s own binary search should find instead of
        // dropping straight to zero. With `OVERLAP_AREA_THRESHOLD_DEG2`
        // now correctly sized (its own docs on the earlier, ~9300x too
        // permissive value), the search converges tightly to that real
        // ~2.4m geometric boundary rather than drifting well past it.
        let lane_a = short_lane("e0_0", 2.0, 3.2);
        let lane_b = Lane {
            shape: Shape(vec![
                Point { x: -5.0, y: -2.0, z: 0.0 },
                Point { x: 5.0, y: -2.0, z: 0.0 },
            ]),
            ..short_lane("e1_0", 10.0, 3.2)
        };
        let network = utm_31n_network_multi_edge(vec![("e0", vec![lane_a]), ("e1", vec![lane_b])]);
        let zones = vec![
            zone_spanning("j0_0", "e0_0", Length::new::<meter>(2.0)),
            zone_spanning("j0_1", "e1_0", Length::new::<meter>(10.0)),
        ];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(
            overlapping_zone_ids(&collection),
            Vec::new(),
            "shrinking the stub's own padding should have been enough to clear e1_0"
        );

        let stub = collection.features.iter().find(|f| f.property("waiting_zone_id").unwrap() == "j0_0").unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } = &stub.geometry.as_ref().unwrap().value else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        let lats = ring.iter().map(|p| p[1]);
        let lat_span_m = (lats.clone().fold(f64::MIN, f64::max) - lats.fold(f64::MAX, f64::min)) * 111_320.0;
        assert!(
            lat_span_m > 2.0,
            "expected some padding to survive (more than e0_0's own real 2m span), got {lat_span_m:.2}m"
        );
        assert!(
            lat_span_m < 2.45,
            "expected convergence tight against the real ~2.4m geometric boundary, not \
             drifting toward the full 5m default, got {lat_span_m:.2}m"
        );
    }

    #[test]
    fn to_feature_collection_bisects_a_genuine_geometric_overlap_padding_cant_explain() {
        // The exact fixture `flags_two_zones_whose_rectangles_actually_overlap` uses to
        // prove the *detector* catches a real overlap -- both lanes are a full 20m
        // long, well past `MIN_DRAWN_LANE_LENGTH_METERS`, so dropping padding can't
        // do anything here at all (both zones are already unpadded); only bisecting
        // can, and this proves `to_feature_collection` reaches for it.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 2.0),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(
            overlapping_zone_ids(&collection),
            Vec::new(),
            "the two rings should have been bisected apart"
        );
        for feature in &collection.features {
            let geojson::GeometryValue::MultiPolygon { coordinates } =
                &feature.geometry.as_ref().unwrap().value
            else {
                panic!("expected a MultiPolygon geometry");
            };
            assert!(
                !coordinates.is_empty(),
                "{:?} lost all of its geometry to the cut",
                feature.property("waiting_zone_id")
            );
        }
    }

    #[test]
    fn to_feature_collection_clips_a_ring_contested_by_two_neighbours_in_one_pass() {
        // Three parallel 3.2m lanes 2m apart: `e0_1` (the middle one)
        // overlaps *both* `e0_0` and `e0_2` (each 2m away, well inside the
        // combined 3.2m of half-widths), while `e0_0` and `e0_2` (4m apart)
        // don't overlap each other at all. `e0_1`'s own zone therefore
        // needs two simultaneous constraints applied to its *one* original
        // shape -- not two sequential cuts against whatever the first one
        // left behind, which is exactly the bug `resolve_overlaps`'s own
        // docs describe fragmenting a real Barcelona corner contested by
        // several neighbours into a disconnected shred.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 2.0),
            parallel_lane("e0_2", 3.2, 4.0),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1"), zone("j0_2", "e0_2")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(overlapping_zone_ids(&collection), Vec::new());

        let middle = collection
            .features
            .iter()
            .find(|f| f.property("waiting_zone_id").unwrap() == "j0_1")
            .unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } = &middle.geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(coordinates.len(), 1, "j0_1 should still be one simple, connected shape");
        assert!(
            !ring_self_intersects(&coordinates[0][0]),
            "clipping against two neighbours in one pass should stay a single simple \
             polygon, not a self-intersecting shred: {:?}",
            coordinates[0][0]
        );
    }

    /// A straight lane of `width_m`, running from `from` to `to` in the
    /// network's own local coordinates — [`single_successors`]'s and
    /// [`chain_shape`]'s own tests need real, differently-positioned
    /// connected segments, unlike [`parallel_lane`]'s fixed north-south
    /// 20m shape.
    fn segment_lane(id: &str, width_m: f64, from: (f64, f64), to: (f64, f64)) -> Lane {
        let length = (to.0 - from.0).hypot(to.1 - from.1);
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(10.0),
            length: Length::new::<meter>(length),
            width: Length::new::<meter>(width_m),
            end_offset: Length::new::<meter>(0.0),
            shape: Shape(vec![
                Point { x: from.0, y: from.1, z: 0.0 },
                Point { x: to.0, y: to.1, z: 0.0 },
            ]),
            allow: vec![],
            disallow: vec![],
        }
    }

    #[test]
    fn extended_ancestor_chain_becomes_one_seamless_polygon_not_one_rectangle_per_lane() {
        // Three real, connected 20m lanes end to end: e2 (furthest back)
        // into e1 into e0 (the zone's own controlled lane). Both e1 and e2
        // are extended-ancestor entries, exactly as
        // `zone_generator::extended_entry_lanes` would produce for a real
        // fork-free chain like this -- before `chain_shape` existed, each
        // drew its own independent 20m rectangle, with a real Barcelona
        // gap between them often enough to be worth fixing; now the two
        // should concatenate into one seamless 40m polygon instead.
        let lane_e0 = segment_lane("e0_0", 3.2, (0.0, 0.0), (0.0, 20.0));
        let lane_e1 = segment_lane("e1_0", 3.2, (0.0, -20.0), (0.0, 0.0));
        let lane_e2 = segment_lane("e2_0", 3.2, (0.0, -40.0), (0.0, -20.0));
        let network = Network {
            connections: vec![plain_connection("e2", "e1", None), plain_connection("e1", "e0", None)],
            ..utm_31n_network_multi_edge(vec![
                ("e0", vec![lane_e0]),
                ("e1", vec![lane_e1]),
                ("e2", vec![lane_e2]),
            ])
        };

        let zone = E3Detector {
            id: DetectorId("j0_0".into()),
            entries: vec![
                gate("e0_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e1_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e2_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
            ],
            exits: vec![gate("e0_0", LanePosition::FromStart(Length::new::<meter>(20.0)))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        };

        let collection = to_feature_collection(&network, &[zone]).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            2,
            "one polygon for the core (e0) and one *seamless* chain polygon for e1+e2 \
             concatenated, not three independent rectangles"
        );

        let chain_ring = &coordinates[1][0];
        assert!(!ring_self_intersects(chain_ring), "{chain_ring:?}");
        let lats = chain_ring.iter().map(|p| p[1]);
        let lat_span_m = (lats.clone().fold(f64::MIN, f64::max) - lats.fold(f64::MAX, f64::min)) * 111_320.0;
        assert!(
            lat_span_m > 35.0,
            "expected the concatenated e1+e2 chain to span close to their combined 40m, \
             got {lat_span_m:.1}m"
        );
    }

    #[test]
    fn two_leaves_merging_into_a_shared_ancestor_each_get_their_own_non_overlapping_segment() {
        // A real Y-merge: `e_leaf_a` and `e_leaf_b` are two independent
        // real streets that both feed into `e_mid`, which then feeds into
        // the zone's own core (`e0`) -- the exact shape a real Barcelona
        // corner hit (see `chain_shape`'s own module docs). `e_mid` has two
        // real predecessors (`in_degree` 2), so it has to start its *own*
        // segment rather than being drawn a second time by each branch:
        // Leaflet's own default `evenodd` fill rule turns ground covered by
        // two of a zone's own rings into a rendered hole, a real, visible
        // defect the first version of this drew on real Barcelona data.
        let lane_e0 = segment_lane("e0_0", 3.2, (0.0, 0.0), (0.0, 20.0));
        let lane_mid = segment_lane("e_mid_0", 3.2, (0.0, -20.0), (0.0, 0.0));
        let lane_leaf_a = segment_lane("e_leaf_a_0", 3.2, (-20.0, -40.0), (0.0, -20.0));
        let lane_leaf_b = segment_lane("e_leaf_b_0", 3.2, (20.0, -40.0), (0.0, -20.0));
        let network = Network {
            connections: vec![
                plain_connection("e_leaf_a", "e_mid", None),
                plain_connection("e_leaf_b", "e_mid", None),
                plain_connection("e_mid", "e0", None),
            ],
            ..utm_31n_network_multi_edge(vec![
                ("e0", vec![lane_e0]),
                ("e_mid", vec![lane_mid]),
                ("e_leaf_a", vec![lane_leaf_a]),
                ("e_leaf_b", vec![lane_leaf_b]),
            ])
        };

        let zone = E3Detector {
            id: DetectorId("j0_0".into()),
            entries: vec![
                gate("e0_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e_mid_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e_leaf_a_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e_leaf_b_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
            ],
            exits: vec![gate("e0_0", LanePosition::FromStart(Length::new::<meter>(20.0)))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        };

        let collection = to_feature_collection(&network, &[zone]).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            4,
            "one core polygon, one segment per leaf branch (a, b) covering only its own \
             unique ground, and one more for e_mid's own shared segment -- never e_mid \
             drawn twice"
        );
        for polygon in coordinates {
            assert!(!ring_self_intersects(&polygon[0]), "{:?}", polygon[0]);
        }
        // A tiny sliver where two straight-rectangle approximations meet
        // at a sharp real angle (this fixture's own `via`-less corners are
        // sharper than most real ones, which usually have a real via lane
        // smoothing the transition) is an accepted, pre-existing
        // limitation of approximating a bend with rectangles at all --
        // `resolve_overlaps` exists to clean up exactly this kind of thing
        // *between* zones already. What this test actually checks is that
        // `e_mid`'s own ~64m² of real ground isn't duplicated wholesale
        // into two segments the way it used to be: a genuine duplicate
        // would show up several orders of magnitude past a sharp corner's
        // own sliver.
        const MAX_CORNER_OVERLAP_DEG2: f64 = 1e-9;
        for i in 0..coordinates.len() {
            for j in (i + 1)..coordinates.len() {
                let area = polygon_overlap_area(&coordinates[i][0], &coordinates[j][0]);
                assert!(
                    area < MAX_CORNER_OVERLAP_DEG2,
                    "segments {i} and {j} share {area} deg² -- too large to be a sharp \
                     corner's own sliver, this looks like e_mid's own ground duplicated \
                     wholesale into both segments again"
                );
            }
        }
    }

    #[test]
    fn does_not_flag_two_zones_that_share_an_edge_exactly() {
        // Centrelines exactly `width` apart: the rectangles' edges coincide
        // (the line straight between the two centrelines) without any
        // actual area in common — adjacent, not overlapping. This is
        // deliberately the *exact* zero-gap case, not a hairline gap: real
        // adjacent lanes on the same road share their boundary exactly, in
        // the network's own local CRS, and reprojecting each rectangle to
        // WGS84 independently (see the module docs) can make that shared
        // edge land on slightly different coordinates for each — a real
        // scenario `polygon_overlap_area`'s own docs cover, which this
        // proves doesn't false-positive.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 3.2),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(
            overlapping_zone_ids(&collection),
            Vec::new(),
            "these rectangles are built to share an edge exactly, not to overlap"
        );
    }

    #[test]
    fn does_not_flag_two_well_separated_zones() {
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 50.0),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(overlapping_zone_ids(&collection), Vec::new());
    }

    /// Whether any two non-adjacent edges of closed ring `ring` (GeoJSON
    /// style: first position repeated last) properly cross — a simple
    /// segment-crossing scan, same idea as [`segments_properly_cross`] via
    /// [`orientation`] but over one ring's own edges instead of two
    /// different rings'. A self-intersecting ("bowtie") polygon is invalid
    /// GeoJSON a client's point-in-polygon test can't reason about at all.
    fn ring_self_intersects(ring: &[Position]) -> bool {
        let n = ring.len().saturating_sub(1); // last position repeats the first
        let edge = |i: usize| ((ring[i][0], ring[i][1]), (ring[i + 1][0], ring[i + 1][1]));
        for i in 0..n {
            for j in (i + 2)..n {
                if i == 0 && j == n - 1 {
                    continue; // adjacent via the closing wrap-around
                }
                let (p1, p2) = edge(i);
                let (p3, p4) = edge(j);
                let d1 = orientation(p3, p4, p1);
                let d2 = orientation(p3, p4, p2);
                let d3 = orientation(p1, p2, p3);
                let d4 = orientation(p1, p2, p4);
                if (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0) {
                    return true;
                }
            }
        }
        false
    }

    /// A short, sharply zigzagging, *wide* lane — mirroring a real
    /// Barcelona walkingarea found while fixing this (2.9m long over 8
    /// shape points, 4m wide: the width bigger than the whole path). A
    /// naive per-vertex or per-segment offset self-intersects on a shape
    /// like this — the width exceeds the path's own local turning radius —
    /// which is exactly why [`offset_boundary`] runs
    /// [`remove_self_intersections`] on each side; see its own docs.
    fn zigzag_lane(id: &str, width_m: f64) -> Lane {
        let shape = Shape(vec![
            Point { x: 0.0, y: 0.0, z: 0.0 },
            Point { x: -2.0, y: 1.0, z: 0.0 },
            Point { x: -1.5, y: 2.0, z: 0.0 },
            Point { x: 1.0, y: 3.5, z: 0.0 },
            Point { x: 2.0, y: 2.5, z: 0.0 },
            Point { x: 0.5, y: 1.0, z: 0.0 },
            Point { x: 1.8, y: -0.5, z: 0.0 },
        ]);
        let length: f64 = shape
            .0
            .windows(2)
            .map(|w| {
                let [a, b] = w else { unreachable!() };
                (b.x - a.x).hypot(b.y - a.y)
            })
            .sum();
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(2.0),
            length: Length::new::<meter>(length),
            width: Length::new::<meter>(width_m),
            end_offset: Length::new::<meter>(0.0),
            shape,
            allow: vec![],
            disallow: vec![],
        }
    }

    #[test]
    fn merges_two_adjacent_lanes_of_the_same_zone_into_a_single_polygon() {
        // Same setup `flags_two_zones_whose_rectangles_actually_overlap`
        // uses for two *different* zones, but as one zone spanning both
        // lanes instead — the real shape this exists for (e.g. two
        // adjacent Barcelona vehicle lanes both going straight).
        let network = utm_31n_network(vec![
            indexed_parallel_lane("e0_0", 0, 3.2, 0.0),
            indexed_parallel_lane("e0_1", 1, 3.2, 3.2),
        ]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_1"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            1,
            "two physically contiguous lanes of the same zone should merge into one \
             polygon, not the seam-prone one-rectangle-per-lane MultiPolygon"
        );
        assert!(!ring_self_intersects(&coordinates[0][0]));
    }

    #[test]
    fn draws_an_extended_ancestor_entry_as_its_own_polygon_alongside_the_core_one() {
        // e0 is the zone's own controlled lane (has an exit); e1 is an
        // ancestor `zone_generator::extended_entry_lanes` walked back onto
        // -- present only as an entry, on a *different* edge, so it can't
        // merge with e0's own polygon (see `zone_feature`'s own docs).
        let network = utm_31n_network_multi_edge(vec![
            ("e0", vec![indexed_parallel_lane("e0_0", 0, 3.2, 0.0)]),
            ("e1", vec![indexed_parallel_lane("e1_0", 0, 3.2, 50.0)]),
        ]);
        let zone = E3Detector {
            id: DetectorId("j0_0".into()),
            entries: vec![
                gate("e0_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e1_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
            ],
            exits: vec![gate("e0_0", LanePosition::FromStart(Length::new::<meter>(20.0)))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        };

        let collection = to_feature_collection(&network, &[zone]).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            2,
            "one merged/core polygon for e0 (the zone's own controlled lane) plus one \
             independent polygon for the extended ancestor e1 -- not merged together, \
             since they're lanes of different edges"
        );
        for polygon in coordinates {
            assert!(!ring_self_intersects(&polygon[0]));
        }
    }

    #[test]
    fn merges_anyway_when_the_zones_lanes_are_not_contiguous() {
        // Lane 1 sits physically between lanes 0 and 2, but this zone only
        // claims 0 and 2 (as if lane 1 belonged to some other movement) --
        // this is exactly the shape `merged_zone_ring`'s own docs say should
        // never happen for real `zone_generator` output, so there's no
        // fallback to fall back to any more: it still merges the two lanes
        // it does have (and, not asserted here since it just goes to
        // stderr, prints an error about it) rather than refusing to produce
        // a zone at all.
        let network = utm_31n_network(vec![
            indexed_parallel_lane("e0_0", 0, 3.2, 0.0),
            indexed_parallel_lane("e0_1", 1, 3.2, 3.2),
            indexed_parallel_lane("e0_2", 2, 3.2, 6.4),
        ]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_2"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(coordinates.len(), 1, "still one merged polygon, not a crash or a gap");
    }

    #[test]
    fn offset_boundary_never_self_intersects_on_a_short_wide_zigzag() {
        let lane = zigzag_lane("e0_0", 4.0);
        let length = lane.length;
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone_spanning("j0_0", "e0_0", length)];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        assert!(
            !ring_self_intersects(ring),
            "offset_boundary produced a self-intersecting polygon for a short, wide, \
             sharply zigzagging lane: {ring:?}"
        );
    }

    #[test]
    fn shape_ring_does_not_collapse_to_a_sliver_on_a_short_lane_with_tight_bends() {
        // A real Barcelona lane (`1395130587_2`, found while fixing this)
        // shifted to the origin: five segments, none longer than ~1.8m, with
        // two real direction changes -- short enough, and tight enough
        // relative to a 3.2m-wide lane, that `remove_self_intersections`'s
        // splice-based cleanup used to cut away real, non-crossing area
        // along with the actual self-crossing loop, collapsing a ~21.6m²
        // ribbon down to 0.003m² (a few points a few centimetres apart).
        // `close_ring`'s own naive-area fallback (see its docs) exists
        // specifically to catch this.
        let shape = Shape(vec![
            Point { x: 0.0, y: 0.0, z: 0.0 },
            Point { x: -0.14, y: -0.14, z: 0.0 },
            Point { x: -0.95, y: -1.73, z: 0.0 },
            Point { x: -0.77, y: -3.19, z: 0.0 },
            Point { x: -0.57, y: -4.67, z: 0.0 },
            Point { x: -1.35, y: -6.29, z: 0.0 },
        ]);
        let length: f64 = shape
            .0
            .windows(2)
            .map(|w| {
                let [a, b] = w else { unreachable!() };
                (b.x - a.x).hypot(b.y - a.y)
            })
            .sum();
        let lane = Lane {
            id: LaneId("e0_0".into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(2.0),
            length: Length::new::<meter>(length),
            width: Length::new::<meter>(3.2),
            end_offset: Length::new::<meter>(0.0),
            shape,
            allow: vec![],
            disallow: vec![],
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone_spanning("j0_0", "e0_0", Length::new::<meter>(length))];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        assert!(!ring_self_intersects(ring), "collapsed ring should still be simple: {ring:?}");

        let points: Vec<(f64, f64)> = ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
        let naive_area_m2 = length * 3.2;
        assert!(
            area_m2 > naive_area_m2 * 0.5,
            "ring area {area_m2:.3}m2 is far below the naive expectation of \
             {naive_area_m2:.3}m2 for a {length:.2}m lane 3.2m wide -- \
             remove_self_intersections likely collapsed it again: {ring:?}"
        );
    }

    #[test]
    fn simplify_points_drops_nearly_collinear_intermediate_points() {
        // A straight line along x, with several intermediate points sitting
        // within a millimetre of it -- exactly what a chain of several
        // sub-metre real connector lanes strung together tends to produce
        // (`chain_shape`'s own module docs).
        let points = vec![
            Point { x: 0.0, y: 0.0, z: 0.0 },
            Point { x: 1.0, y: 0.0005, z: 0.0 },
            Point { x: 2.0, y: -0.0003, z: 0.0 },
            Point { x: 3.0, y: 0.0002, z: 0.0 },
            Point { x: 10.0, y: 0.0, z: 0.0 },
        ];
        let simplified = simplify_points(points, SIMPLIFY_TOLERANCE_METERS);
        assert_eq!(
            simplified.len(),
            2,
            "every intermediate point is within a millimetre of the straight line \
             between the endpoints, well under the 5cm tolerance -- expected just the \
             two endpoints to survive"
        );
    }

    #[test]
    fn simplify_points_keeps_a_real_corner() {
        // A genuine right-angle turn, 5m off the straight line between the
        // endpoints -- two orders of magnitude past the 5cm tolerance, so
        // the corner itself has to survive simplification.
        let points = vec![
            Point { x: 0.0, y: 0.0, z: 0.0 },
            Point { x: 5.0, y: 5.0, z: 0.0 },
            Point { x: 10.0, y: 0.0, z: 0.0 },
        ];
        let simplified = simplify_points(points.clone(), SIMPLIFY_TOLERANCE_METERS);
        assert_eq!(simplified, points, "a genuine corner should never be simplified away");
    }

    #[test]
    fn simplify_points_never_drops_below_two_points() {
        assert_eq!(simplify_points(vec![], 0.05).len(), 0);
        let one = vec![Point { x: 0.0, y: 0.0, z: 0.0 }];
        assert_eq!(simplify_points(one.clone(), 0.05), one);
    }

    #[test]
    fn triangulate_sums_to_the_original_polygons_own_area() {
        // An L-shape (non-convex): a 4x4 square with its own top-right 3x3
        // corner removed, real area 16 - 9 = 7. Whatever `triangulate`
        // splits it into should sum back to exactly that, regardless of
        // how many triangles it takes.
        let l_shape = vec![(0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (1.0, 1.0), (1.0, 4.0), (0.0, 4.0)];
        let triangles = triangulate(&l_shape);
        let total_area: f64 = triangles.iter().map(|t| signed_area(t).abs()).sum();
        assert!(
            (total_area - 7.0).abs() < 1e-9,
            "expected the L-shape's own real area (7.0) preserved across triangulation, got \
             {total_area} from {} triangles",
            triangles.len()
        );
    }

    #[test]
    fn polygon_overlap_area_handles_a_non_convex_clip_correctly() {
        // `clip_to_convex` alone assumes its own `clip` argument is
        // convex -- passing a non-convex ring like this L-shape as the
        // clip side used to give `0.0` for a real overlap, entirely
        // depending on which of `polygon_overlap_area`'s two arguments it
        // landed on: confirmed on real Barcelona zones sharing several
        // real square metres that measured `0.0` clipped one way and
        // multiple m² the other.
        let l_shape: Vec<Position> = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 1.0),
            (1.0, 1.0),
            (1.0, 4.0),
            (0.0, 4.0),
            (0.0, 0.0),
        ]
        .into_iter()
        .map(|(x, y)| Position::from([x, y]))
        .collect();

        // A 1x1 square entirely inside the L's own vertical leg (x in
        // [0,1], y in [1,2]) -- real, unambiguous overlap of exactly 1.0.
        let square: Vec<Position> = [(0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0), (0.0, 1.0)]
            .into_iter()
            .map(|(x, y)| Position::from([x, y]))
            .collect();

        let forward = polygon_overlap_area(&square, &l_shape);
        let backward = polygon_overlap_area(&l_shape, &square);
        assert!(
            (forward - 1.0).abs() < 1e-9,
            "expected the square's own full 1.0 area, entirely inside the L, got {forward}"
        );
        assert!(
            (backward - 1.0).abs() < 1e-9,
            "the same overlap measured with the arguments swapped should agree, got {backward}"
        );
    }

    /// Below this, in real m², a ring can't plausibly be a real waiting
    /// area — not even standing room for a single pedestrian — so it has to
    /// be a construction artifact rather than real ground `zone_generator`
    /// actually meant to claim. Chosen with real headroom below the
    /// smallest *legitimate* ring in real Barcelona data (a genuinely short
    /// zone at ~0.22m², well above this) but orders of magnitude above the
    /// sub-0.06m² slivers `close_ring`'s own naive-area fallback (see its
    /// docs) was added to eliminate.
    const MIN_PLAUSIBLE_RING_AREA_M2: f64 = 0.05;

    /// Below this, in degrees, two consecutive ring edges are judged a
    /// spike — a needle-thin notch a client's own rendering (and a human
    /// looking at the map) reads as visibly wrong — rather than a real
    /// corner a waiting area's own shape can legitimately have. A waiting
    /// zone's boundary is either a straight lane edge (interior angle
    /// 180°, dead flat) or a bevel facet where `offset_boundary` turns a
    /// bend (see its own docs) — both stay well clear of 80° on any real
    /// street geometry; a survivor below it is exactly the shape of defect
    /// this crate's own bug history is full of (a self-intersection
    /// `remove_self_intersections` failed to fully clean up, a
    /// `clip_by_constraints` phantom sliver, ...), not a legitimate acute
    /// corner this crate has any reason to draw.
    const MIN_INTERIOR_ANGLE_DEGREES: f64 = 80.0;

    /// Below this, in degrees (roughly a centimetre at Barcelona's own
    /// latitude — see [`OVERLAP_AREA_THRESHOLD_DEG2`]'s own deg-to-metre
    /// conversion), an edge is too short for its direction to mean
    /// anything: a bevel facet's own two corners (`offset_boundary`'s own
    /// docs) can land this close together, and the angle either one of
    /// them forms with its *other*, real-length neighbour is noise, not a
    /// spike — skipped by [`interior_angles_degrees`] rather than reported
    /// as a false `0.0`.
    const MIN_EDGE_LENGTH_DEGREES: f64 = 1e-7;

    /// The interior angle, in degrees, at every vertex of `points` (a
    /// ring's own distinct vertices — no repeated closing point) whose two
    /// neighbouring edges are both at least [`MIN_EDGE_LENGTH_DEGREES`]
    /// long: the angle between the two segments meeting there, `180°` for
    /// a vertex a straight line already passes straight through, sliding
    /// down toward `0°` as the two segments fold back on each other into a
    /// spike. Undirected — computed from the two edge vectors pointing
    /// *away* from the vertex, so it doesn't need the ring's own winding
    /// direction or which side is "inside".
    fn interior_angles_degrees(points: &[(f64, f64)]) -> Vec<f64> {
        let n = points.len();
        (0..n)
            .filter_map(|i| {
                let (prev, cur, next) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
                let (ax, ay) = (prev.0 - cur.0, prev.1 - cur.1);
                let (bx, by) = (next.0 - cur.0, next.1 - cur.1);
                let (la, lb) = (ax.hypot(ay), bx.hypot(by));
                if la < MIN_EDGE_LENGTH_DEGREES || lb < MIN_EDGE_LENGTH_DEGREES {
                    return None;
                }
                let cos_angle = ((ax * bx + ay * by) / (la * lb)).clamp(-1.0, 1.0);
                Some(cos_angle.acos().to_degrees())
            })
            .collect()
    }

    #[test]
    fn every_real_barcelona_zone_ring_is_simple_and_has_a_plausible_shape() {
        // An end-to-end coherence sweep over real Barcelona output, not
        // just the specific fixtures above: every ring this crate would
        // actually ship has to be a simple polygon (no self-crossing a
        // client's point-in-polygon test can't reason about) enclosing a
        // physically plausible amount of ground with no spike in its own
        // outline, not just *some* of the narrower failure modes those
        // fixtures each target individually.
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None);
        let collection = to_feature_collection(&network, &zones).expect("building collection");

        let mut failures = Vec::new();
        for feature in &collection.features {
            let id = feature.property("waiting_zone_id").unwrap().as_str().unwrap();
            for (ri, ring) in feature_rings(feature).into_iter().enumerate() {
                let points: Vec<(f64, f64)> =
                    ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
                if polyline_self_intersects(&points) {
                    failures.push(format!("{id} ring {ri} self-intersects: {points:?}"));
                    continue;
                }
                let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
                if area_m2 < MIN_PLAUSIBLE_RING_AREA_M2 {
                    failures.push(format!(
                        "{id} ring {ri} has an implausibly small area of {area_m2:.6}m2 \
                         (below {MIN_PLAUSIBLE_RING_AREA_M2}m2): {points:?}"
                    ));
                }
                for (vi, angle) in interior_angles_degrees(&points).into_iter().enumerate() {
                    if angle < MIN_INTERIOR_ANGLE_DEGREES {
                        failures.push(format!(
                            "{id} ring {ri} vertex {vi} has a {angle:.1}° interior angle \
                             (below {MIN_INTERIOR_ANGLE_DEGREES}°) — looks like a spike: {points:?}"
                        ));
                    }
                }
            }
        }

        assert!(
            failures.is_empty(),
            "{} incoherent ring(s) found among real Barcelona zones:\n{}",
            failures.len(),
            failures.join("\n"),
        );
    }

    #[test]
    fn bisecting_half_plane_anchors_on_the_actual_overlap_not_the_two_rings_own_centroids() {
        // A long, straight zone (its own centroid at x=50) whose real
        // conflict is entirely at its far end (x in [95, 100]) -- the same
        // shape of problem as a real Barcelona bug this guards against (an
        // 80.9m ribbon contested by neighbours clustered around one small
        // bend near its own far end), simplified to a straight rectangle so
        // this fix (anchoring on the real overlap) isn't entangled with the
        // *other* one bent rings specifically need
        // (`clip_by_constraints`'s own multi-constraint gate, see its
        // docs). The old behaviour anchored at the two rings' own centroid
        // midpoint (x=75) — nowhere near the real conflict at x≈97.5 either
        // — and combining several such off-target cuts from multiple
        // simultaneous neighbours was what collapsed the real ribbon down
        // to a small, valid-looking (simple, even convex) phantom sliver
        // instead of correctly trimming just its far end.
        let ring_a: Vec<Position> =
            [(0.0, 0.0), (100.0, 0.0), (100.0, 10.0), (0.0, 10.0), (0.0, 0.0)].into_iter().map(Position::from).collect();
        let ring_b: Vec<Position> =
            [(95.0, 3.0), (105.0, 3.0), (105.0, 7.0), (95.0, 7.0), (95.0, 3.0)].into_iter().map(Position::from).collect();

        let (anchor, _normal) = bisecting_half_plane(&ring_a, &ring_b).expect("distinct centroids");
        assert!(
            anchor.0 > 90.0,
            "anchor should sit near the real overlap around x=97.5 (the far end of a \
             100-long ribbon), not at x=75 (the two rings' own centroid midpoint) or \
             x=50 (ring_a's own centroid): got {anchor:?}"
        );
    }

    #[test]
    fn a_long_bent_ribbon_keeps_most_of_its_own_area_when_only_one_end_is_contested() {
        // The real Barcelona ribbon this bug was found on: `-27641458#20_1`
        // is an 80.9m lane with one gentle bend, contested near that bend
        // by three separate neighbouring zones at once. Before anchoring
        // `bisecting_half_plane` on the real overlap instead of the two
        // rings' own centroids, and gating `clip_by_constraints`'s
        // multi-constraint direct path on `base`'s own exact convexity
        // (see both their own docs), this collapsed to a ~53m²
        // disconnected notch nowhere near any of the three real overlaps;
        // correctly, it should keep the great majority of the ribbon's own
        // ~260m² and lose only a small piece near the bend.
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None);
        let collection = to_feature_collection(&network, &zones).expect("building collection");

        let feature = collection
            .features
            .iter()
            .find(|f| f.property("waiting_zone_id").unwrap().as_str().unwrap() == "-27641458#20_straight")
            .expect("zone found");
        let rings = feature_rings(feature);
        assert_eq!(rings.len(), 1, "this zone is a single lane's own core ring, no ancestor extension");
        let points: Vec<(f64, f64)> = rings[0][..rings[0].len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
        assert!(
            area_m2 > 130.0,
            "expected to keep most of this 80.9m-lane ribbon's own ~260m², losing only a \
             small piece near its bend to the neighbours contesting it there -- got \
             {area_m2:.1}m2, suspiciously close to the old collapsed-notch bug's ~53m²: \
             {points:?}"
        );
    }

}

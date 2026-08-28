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
//! polygon — so each lane in a zone contributes its own buffered shape: the
//! lane's own centreline between the entry and exit stations, offset
//! left/right by half the lane's width. Building and combining those
//! shapes is real computational geometry (a bend can make one lane's own
//! buffer self-overlap; several lanes, or an extended-ancestor chain on a
//! different edge again, can need merging into one seamless shape; two
//! different zones' own shapes can need separating so a client's point can
//! never match both) — geometry this module deliberately does *not*
//! hand-roll an offset/clip/triangulate pipeline for, using
//! [`geo::BooleanOps`] instead:
//!
//! - A lane's own buffer ([`buffer_shape`]) is the union of one convex
//!   quadrilateral per segment of its (entry/exit-trimmed) shape — always a
//!   valid simple polygon regardless of how sharply the lane bends, because
//!   `union` resolves however the quads along it overlap or leave a wedge
//!   correctly by construction, not by a hand-written self-intersection
//!   splice with its own convex-hull fallback for when that splice cut too
//!   much away.
//! - A zone spanning several lanes, or an extended-ancestor chain touching
//!   the core (see [`chain_shape`]'s own docs), is the union of each
//!   piece's own buffer ([`zone_polygon`]) — correct regardless of lane
//!   order or index contiguity, not dependent (as an earlier version of
//!   this module was) on SUMO happening to number a zone's own lanes
//!   contiguously.
//! - Two zones' polygons overlapping is `intersection`, and separating them
//!   is `difference` against a cutting half-plane ([`resolve_overlaps`]) —
//!   exact for arbitrary, even non-convex, even multi-part polygons, which
//!   is what actually lets a busy corner contested by several neighbours at
//!   once be cut in one pass without an earlier version's own bespoke
//!   convexity bookkeeping for when that stopped being safe.
//!
//! Every polygon above is built and resolved in the network's own local
//! (projected, metric) coordinates — [`Reprojector::to_lon_lat`] is only
//! ever applied once, to a ring's finished points, when building the
//! output `Feature` ([`build_feature`]). Comparing or clipping in metres
//! throughout means an overlap threshold here is a plain area in m² (see
//! [`OVERLAP_AREA_THRESHOLD_M2`]), not a deg² figure that has to correct
//! for a degree of longitude and a degree of latitude covering different
//! real distances.

use anstream::eprintln;
use anstyle::{AnsiColor, Style};
use anyhow::{Context, Result, bail};
use geo::algorithm::area::Area;
use geo::{
    BooleanOps, BoundingRect, Contains, Coord, LineString, MapCoords, MultiPolygon, Point as GeoPoint,
    Polygon as GeoPolygon, Simplify,
};
use geojson::{Feature, FeatureCollection, Geometry, JsonObject, Position};
use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, LineJoin, StrokeStyle};
use proj4rs::proj::Proj;
use proj4rs::transform::transform;
use std::collections::{BTreeSet, HashMap, HashSet};
use std::path::Path;
use sumo_types::additional::domain::{E3Detector, LanePosition};
use sumo_types::domain::{EdgeFunction, EdgeId, Lane, LaneIndex, Location, Network, Point, Projection, Shape, VClass};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

/// Styles the "error:" prefix on [`resolve_overlaps`]'s own messages the
/// way `cargo`/`rustc` style theirs — bold red — mirroring
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

/// `shape`'s own centreline between `entry`/`exit`, as the ordered sequence
/// of distinct points [`trimmed_segments`] walks it through — [`buffer_shape`]'s
/// own input path, reconstructed from segment endpoints rather than exposing
/// `trimmed_segments`'s own `(start, end, tangent)` triples directly, since
/// a real stroke-offset call needs the path as a plain polyline, not
/// pre-split into segments (a real join, at each interior point, is exactly
/// what a per-segment split would throw away).
fn trimmed_path_points(shape: &Shape, entry: Length, exit: Length) -> Vec<Point> {
    let segments = trimmed_segments(shape, entry, exit);
    let mut points = Vec::with_capacity(segments.len() + 1);
    for (i, &(start, end, _)) in segments.iter().enumerate() {
        if i == 0 {
            points.push(start);
        }
        points.push(end);
    }
    points
}

/// Below this, in radians, [`LineJoin::Round`]'s own arc at a bend is
/// approximated by a single straight segment rather than subdividing it
/// further — the `L / R` ratio [`i_overlay`]'s own docs define it as (max
/// segment length over arc radius). Small enough that the arc reads as
/// smooth rather than faceted on any real lane width, comfortably below
/// the 80° [`MIN_INTERIOR_ANGLE_DEGREES`] this crate's own coherence tests
/// require of *every* vertex: a round join never contributes a sharp
/// vertex of its own by construction, however sharply the underlying lane
/// bends, which is the whole reason this replaces a straight (mitre- or
/// bevel-style) join for this module's purposes — see [`buffer_shape`]'s
/// own docs.
const ROUND_JOIN_SEGMENT_ANGLE_RADIANS: f64 = 0.3;

/// `shapes` (whatever [`i_overlay::mesh::stroke::offset::StrokeOffset::stroke`]
/// returned), converted into a [`MultiPolygon`] in the same local
/// coordinates: one [`GeoPolygon`] per shape, its own first contour as the
/// exterior ring and any further contours as holes (`i_overlay`'s own docs:
/// "outer boundary paths have a counterclockwise order, and holes have a
/// clockwise order" — the same convention `geo` itself uses, so nothing
/// needs correcting here). Every contour is re-closed (its own first point
/// repeated last) if it doesn't already come that way, matching what every
/// other ring this module builds needs: `i_overlay`'s own stroke output
/// doesn't repeat it, `geo::LineString`'s own polygon-membership convention
/// doesn't strictly require it, but this module's downstream GeoJSON output
/// does.
fn shapes_to_multipolygon(shapes: i_overlay::i_shape::base::data::Shapes<[f64; 2]>) -> MultiPolygon<f64> {
    let close = |points: Vec<[f64; 2]>| -> LineString<f64> {
        let mut coords: Vec<Coord<f64>> = points.into_iter().map(|[x, y]| Coord { x, y }).collect();
        if coords.first().is_some() && coords.first() != coords.last() {
            coords.push(coords[0]);
        }
        LineString::new(coords)
    };
    MultiPolygon::new(
        shapes
            .into_iter()
            .filter_map(|mut contours| {
                if contours.is_empty() {
                    return None;
                }
                let exterior = close(contours.remove(0));
                let interiors = contours.into_iter().map(close).collect();
                Some(GeoPolygon::new(exterior, interiors))
            })
            .collect(),
    )
}

/// `shape`'s own boundary between `entry`/`exit`, offset `half_width` to
/// each side of its centreline, as a single (possibly multi-part)
/// [`MultiPolygon`] in the network's own local coordinates — a real stroke
/// offset ([`i_overlay::mesh::stroke::offset::StrokeOffset::stroke`]), not
/// an approximation of one.
///
/// An earlier version of this built the same shape as the union of one
/// straight quadrilateral per [`trimmed_segments`] entry — a valid simple
/// polygon on its own, correctly merged with its neighbours by
/// [`geo::BooleanOps::union`] regardless of how the quads along a bend
/// overlapped or left a wedge between them. That held up for a gentle
/// bend, but not for real Barcelona data at the sharp end: a chain of many
/// very short real segments (sub-metre "connector" lanes strung
/// end-to-end, or a lane split at every OSM node along a tight turn) union
/// a correspondingly large number of thin, near-degenerate quads, and confirmed
/// on real output, that could still come out self-intersecting or spiked —
/// not because any *one* union step was wrong, but because assembling a
/// bend out of many flat-sided pieces has no join of its own at all: each
/// quad's own flat end meets its neighbour's at whatever angle the path
/// bends through, with nothing smoothing the transition, unlike a real
/// stroke algorithm's own explicit join. [`LineJoin::Round`] (see
/// [`ROUND_JOIN_SEGMENT_ANGLE_RADIANS`]'s own docs) gives every bend an
/// actual join instead — a smooth arc that can never itself be sharper
/// than the coherence tests require, and is computed once for the whole
/// path rather than reconstructed from however many tiny pieces happen to
/// approximate it.
///
/// [`LineCap::Butt`] at the *start* always — a flat, perpendicular cut —
/// matches this module's own entry gates exactly: a waiting zone's own
/// boundary is the lane's real width at the exact station a detector gate
/// names, not padded out further by a rounded or squared cap.
///
/// `end_cap` is a parameter, not always `Butt` too, for exactly one reason:
/// [`zone_polygon`]'s own ancestor-chain case unions this shape's own
/// output into the zone's core polygon afterward, and the two only ever
/// *touch* at a single shared point (`chain_shape`'s own docs — the
/// chain's own last point is, by construction, exactly the core's own
/// first), never genuinely overlap. Two polygons capped flat right at that
/// shared point, with even a slightly different tangent direction on each
/// side (an ordinary street doesn't bend in a perfectly straight line
/// through a real junction), don't butt cleanly — confirmed on real
/// Barcelona data: [`geo::BooleanOps::union`] of two such "kissing" shapes
/// can pinch at the seam rather than join into one clean boundary.
/// [`LineCap::Square`] at the chain's own connecting end extends the
/// buffer straight past that point by `half_width`, forcing a real overlap
/// with the core's own buffer along the chain's own tangent — measurably
/// better on real Barcelona data than leaving both ends `Butt` (the worst
/// offender at this specific seam, a 346°-ish reflex spike, is gone), but
/// not a complete fix: a couple of vertices right around the same seam can
/// still land just under [`MIN_INTERIOR_ANGLE_DEGREES`] on real data,
/// still under investigation — see this crate's own follow-up notes rather
/// than treating the seam as fully solved.
///
/// Empty when `entry`/`exit` don't actually span any of `shape`'s own
/// segments — a genuinely zero-length zone, which has no ground of its own
/// to draw.
fn buffer_shape(shape: &Shape, entry: Length, exit: Length, half_width: Length, end_cap: LineCap<[f64; 2], f64>) -> MultiPolygon<f64> {
    let points = trimmed_path_points(shape, entry, exit);
    if points.len() < 2 {
        return MultiPolygon::new(Vec::new());
    }
    let path: Vec<[f64; 2]> = points.iter().map(|p| [p.x, p.y]).collect();
    let style = StrokeStyle::new(2.0 * half_width.get::<meter>())
        .line_join(LineJoin::Round(ROUND_JOIN_SEGMENT_ANGLE_RADIANS))
        .start_cap(LineCap::Butt)
        .end_cap(end_cap);
    shapes_to_multipolygon(path.stroke(style, false))
}

/// The single polygon covering every lane in `lane_gates` — each `(lane,
/// entry distance, exit distance)`, as computed per-gate in
/// [`zone_polygon`] — as the union of each lane's own [`buffer_shape`].
/// Correct regardless of how many lanes a zone has, what order they're in,
/// or whether their own indices happen to be contiguous: an earlier
/// version of this instead picked the group's own leftmost and rightmost
/// lane and joined them directly, on the assumption (true for every zone
/// `zone_generator` has ever produced from real data, but never actually
/// guaranteed by its own model) that SUMO numbers a same-edge zone's lanes
/// contiguously — a `union` isn't assuming anything about the lanes'
/// arrangement in the first place, so there's nothing left for that
/// assumption to be wrong about.
fn merged_core_polygon(lane_gates: &[(&Lane, Length, Length)]) -> MultiPolygon<f64> {
    lane_gates
        .iter()
        .map(|&(lane, entry, exit)| buffer_shape(&lane.shape, entry, exit, lane.width / 2.0, LineCap::Butt))
        .reduce(|acc, polygon| acc.union(&polygon))
        .unwrap_or_else(|| MultiPolygon::new(Vec::new()))
}

/// A walkingarea lane's own `shape`, used directly as a closed polygon —
/// unlike every other kind of lane this crate ever buffers, a
/// `function="walkingarea"` edge's own `<lane>` "shape" isn't a centreline
/// netconvert expects offset by half the lane's width: it's already the
/// outline of the (2D) walkable area netconvert itself computed. Confirmed
/// on real Barcelona data across a random sample of walkingareas: a real
/// lane's own shape arc-length always matches its `length` attribute
/// exactly (`length` *is* "distance travelled along the shape" for one),
/// but a walkingarea's own shape traces 3-10x its own `length`, with the
/// shape's first and last points typically close together rather than the
/// far-apart ends of an open path — the signature of an already-closed
/// outline, not a line to walk along. Treating it as a centreline anyway
/// (this crate's own earlier behaviour) measures "distance `length` along
/// the outline" — a number with no relationship to any real position on
/// it — trims the outline down to a meaningless arbitrary arc, and
/// stroke-buffers that a second time on top: exactly the shape of defect
/// (self-crossing zigzags, extreme reflex angles) real pedestrian zones
/// turned up disproportionately often once this crate started checking for
/// either.
///
/// No entry/exit trimming, unlike [`buffer_shape`]: `zone_generator::pedestrian_zones`
/// never extends a pedestrian zone's own entry backward (a fork-free
/// stretch of sidewalk isn't "committed to this crossing" the way a
/// fork-free stretch of road is — see its own docs), so a pedestrian
/// zone's own entry and exit already span the lane's whole nominal
/// `[0, length]` by construction; with `length` itself not meaning a real
/// position on this particular kind of shape, there's no trim left to
/// apply that would mean anything anyway.
fn pedestrian_lane_polygon(lane: &Lane) -> MultiPolygon<f64> {
    let mut coords: Vec<Coord<f64>> = lane.shape.0.iter().map(|p| Coord { x: p.x, y: p.y }).collect();
    if coords.len() < 3 {
        return MultiPolygon::new(Vec::new());
    }
    if coords.first() != coords.last() {
        coords.push(coords[0]);
    }
    let raw = MultiPolygon::new(vec![GeoPolygon::new(LineString::new(coords), Vec::new())]);
    // Real Barcelona walkingarea outlines aren't always simple polygons on
    // their own — confirmed on real data, some self-touch or self-cross
    // exactly like the hand-built shapes this crate used to have to guard
    // against elsewhere. A self-union routes this through `geo::BooleanOps`'s
    // own exact machinery, which `i_overlay` (the crate behind it)
    // documents as accepting self-intersecting input directly, so whatever
    // netconvert's own output may already have wrong comes back out clean
    // — unlike the same trick tried earlier on already-processed output of
    // this crate's own (see `snap_coords`'s own docs on why that specific
    // case made things worse instead), this is the *first* thing done to
    // raw, external input, not a repair layered on top of several other
    // transforms already in play.
    raw.union(&raw)
}

/// The single polygon covering every lane in `lanes` — a pedestrian zone's
/// own counterpart to [`merged_core_polygon`], built from each lane's own
/// [`pedestrian_lane_polygon`] instead of a stroke-buffered centreline.
fn merged_pedestrian_polygon(lanes: &[&Lane]) -> MultiPolygon<f64> {
    lanes
        .iter()
        .map(|lane| pedestrian_lane_polygon(lane))
        .reduce(|acc, polygon| acc.union(&polygon))
        .unwrap_or_else(|| MultiPolygon::new(Vec::new()))
}

/// Every ring `feature`'s own geometry carries — its exterior ring only, one
/// per sub-polygon (see [`zone_polygon`]'s own docs on why a zone can have
/// more than one, via extended ancestor entries); a hole (an interior ring)
/// never legitimately occurs in anything this module builds — buffering and
/// merging same-direction road/sidewalk ribbons has no way to enclose empty
/// space — so this doesn't look for one. Empty if `feature` has no geometry
/// or isn't a `MultiPolygon` — never true of anything [`build_feature`]
/// itself builds, but this is also called from [`overlapping_zone_ids`] on a
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

/// `feature`'s own geometry, rebuilt as a [`MultiPolygon`] in whatever 2D
/// coordinate space its positions are already in — [`overlapping_zone_ids`]'s
/// own way of getting back to real polygons (and so real, exact
/// [`geo::BooleanOps::intersection`]) from already-built GeoJSON, the one
/// place in this module that still has to work from a `Feature` rather than
/// the local-coordinate `MultiPolygon`s the rest of the pipeline carries
/// straight through (see the module docs). Only the exterior ring of each
/// sub-polygon — see [`feature_rings`]'s own docs on why a hole never
/// legitimately occurs here.
fn feature_multipolygon(feature: &Feature) -> MultiPolygon<f64> {
    let rings = feature_rings(feature).into_iter().map(|ring| {
        GeoPolygon::new(LineString::new(ring.iter().map(|p| Coord { x: p[0], y: p[1] }).collect()), Vec::new())
    });
    MultiPolygon::new(rings.collect())
}

/// Same reasoning as [`OVERLAP_AREA_THRESHOLD_M2`] (see its own docs), but
/// converted for [`overlapping_zone_ids`]'s own lon/lat input: the rest of
/// this module works in the network's own local, metric coordinates
/// throughout (see the module docs), so a plain m² threshold is enough
/// there, but this function's own public contract is a `FeatureCollection`
/// already reprojected to WGS84. A degree of longitude at Barcelona's own
/// latitude is close to 84km, a degree of latitude close to 111km, so
/// converting an *area* threshold from m² to deg² divides by both.
const OVERLAP_AREA_THRESHOLD_DEG2: f64 = OVERLAP_AREA_THRESHOLD_M2 / (84_000.0 * 111_000.0);

/// Every pair of *different* zones in `collection` whose own areas overlap
/// by more than [`OVERLAP_AREA_THRESHOLD_DEG2`]. A client geofences against
/// this output by testing a GPS point against each zone's polygon (see the
/// module docs); an overlap here means a single point can match two
/// different zones at once, which is exactly the ambiguity a client asking
/// "which crossing am I waiting at" can't resolve on its own — see
/// `zone_generator::pedestrian_zones`'s own docs for a concrete way this can
/// happen (one physical corner feeding two differently-signalled
/// crossings). [`to_feature_collection`] itself already runs this same
/// check and resolves whatever it finds (see [`resolve_overlaps`]), so this
/// only ever finds something real on output this crate produced when called
/// directly on a hand-built collection — exactly how this module's own
/// tests use it to exercise the detector in isolation.
pub fn overlapping_zone_ids(collection: &FeatureCollection) -> Vec<(String, String)> {
    let ids: Vec<Option<String>> = collection
        .features
        .iter()
        .map(|feature| feature.property("waiting_zone_id").and_then(|v| v.as_str()).map(str::to_string))
        .collect();
    let polygons: Vec<MultiPolygon<f64>> = collection.features.iter().map(feature_multipolygon).collect();

    let mut found = Vec::new();
    for i in 0..polygons.len() {
        for j in (i + 1)..polygons.len() {
            if polygons[i].intersection(&polygons[j]).unsigned_area() > OVERLAP_AREA_THRESHOLD_DEG2
                && let (Some(a), Some(b)) = (&ids[i], &ids[j])
            {
                found.push((a.clone(), b.clone()));
            }
        }
    }
    found
}

/// Below this, in real m², an overlap between two zones' polygons is
/// floating-point noise rather than real, shared ground — two rectangles
/// independently built to share an edge exactly can still disagree on that
/// edge's last few floating-point digits once a bend or a union has moved
/// their vertices around. Every polygon this threshold compares is still in
/// the network's own local, metric coordinates (see the module docs), so
/// this is a plain area in m², not a degree-based figure that would also
/// have to correct for a degree of longitude and a degree of latitude
/// covering different real distances — the whole reason an earlier version
/// of this comparison (before reprojection moved to the very end of the
/// pipeline) needed that correction at all.
const OVERLAP_AREA_THRESHOLD_M2: f64 = 0.01;

fn overlap_area_m2(a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> f64 {
    let boxes_overlap = match (a.bounding_rect(), b.bounding_rect()) {
        (Some(ra), Some(rb)) => {
            ra.min().x <= rb.max().x && rb.min().x <= ra.max().x && ra.min().y <= rb.max().y && rb.min().y <= ra.max().y
        }
        _ => false,
    };
    // `intersection` is the expensive part; most pairs across a whole city
    // are nowhere near each other, so a bounding-box check first is worth
    // it the same way it was for the pipeline this replaces.
    if boxes_overlap { a.intersection(b).unsigned_area() } else { 0.0 }
}

/// `polygon` — the result of cutting *one* originally-connected part of a
/// zone's own `MultiPolygon` (see [`resolve_overlaps`]'s own docs on why
/// it's called per part, never on a zone's whole `MultiPolygon` at once) —
/// reduced to just the piece of that result still connected to `reference`
/// (its own zone's `stop_line_point`, in the same local coordinates). A
/// no-op when `polygon` already has at most one part.
///
/// A cut is exact set subtraction: it removes precisely the real, disputed
/// overlap and nothing else, which is correct as far as it goes, but says
/// nothing about *where in the shape* that overlap happened to fall. A
/// neighbour whose own overlap lands in the *middle* of a long
/// single-part chain — not at either end — cuts that one part into two
/// separate pieces, one still attached to the zone's own stop line and one
/// stranded further back with no way back to it: confirmed on real
/// Barcelona data as a real, visible defect (a "hole" with ground
/// continuing on the far side of it), not the intended "the zone gives up
/// disputed ground and stops there" a client geofencing against this
/// output needs. A piece stranded past the collision this way is
/// unreachable from the zone's own controlled stop line without crossing
/// the very ground just excluded, so it isn't real waiting-area ground for
/// this zone any more, whether or not the neighbour claims it either —
/// dropping it is what "stops at the collision" actually means at the
/// polygon level.
fn keep_part_near(polygon: MultiPolygon<f64>, reference: Coord<f64>) -> MultiPolygon<f64> {
    if polygon.0.len() <= 1 {
        return polygon;
    }
    let reference_point = GeoPoint::from(reference);
    // Prefer the part that actually contains the stop line first: distance
    // alone can be fooled by a tiny sliver that happens to sit closer to the
    // reference point than the real, substantially larger remaining piece
    // (seen on 50926861#0_straight, where a ~0.28 m^2 scrap sat nearer the
    // stop line than the zone's real remaining ground). Falling back to the
    // largest part by area is a much more robust proxy for "the real
    // surviving ground" than raw nearest-by-distance.
    let containing = polygon
        .0
        .iter()
        .find(|part| part.contains(&reference_point))
        .cloned();
    let chosen = containing.or_else(|| {
        polygon
            .0
            .into_iter()
            .max_by(|a, b| a.unsigned_area().total_cmp(&b.unsigned_area()))
    });
    MultiPolygon::new(chosen.into_iter().collect())
}

/// Below this, in real m², a polygon part left over from a
/// [`resolve_overlaps`] cut is floating-point noise rather than real ground
/// worth keeping — an edge cut almost exactly along an existing vertex can
/// leave a sliver a fraction of a millimetre wide. [`overlap_area_m2`]'s own
/// threshold guards the same kind of noise at the *detection* end (deciding
/// whether two zones overlap at all); this is the same idea applied to a
/// cut's own *output*, and matters for more than tidiness: a real Barcelona
/// pedestrian zone, cut repeatedly by several neighbours across a few
/// rounds, accumulated slivers this thin, and [`geo::BooleanOps`]'s own
/// fixed-point core measurably slows down (confirmed: a single
/// `intersection` against one of these went from microseconds to hundreds
/// of milliseconds) — and can hit an internal precision assertion outright
/// — on geometry this degenerate. Dropping them immediately after every cut
/// keeps every polygon this module carries forward numerically
/// well-conditioned, not just visually clean.
const MIN_KEPT_PART_AREA_M2: f64 = 0.01;

fn drop_slivers(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(polygon.0.into_iter().filter(|part| part.unsigned_area() > MIN_KEPT_PART_AREA_M2).collect())
}

/// Below this, in metres, two vertices are the same point as far as this
/// module's own output is concerned — `snap_coords` rounds every coordinate
/// to the nearest multiple of it. [`geo::BooleanOps`] itself already snaps
/// to *some* fixed-point grid internally (that's what makes it exact), but
/// two vertices that were meant to coincide (the same real corner, computed
/// two different ways — e.g. by each side of a shared cut, independently,
/// in [`resolve_overlaps`]) can still land a float epsilon apart, which the
/// *next* round's own [`overlap_area_m2`] check reads as a fresh, genuine
/// sliver of overlap to resolve all over again — confirmed on real
/// Barcelona data: a small cluster of zones kept "resolving" a hairline
/// overlap only for the next round to find a new one in its place, each
/// round's own polygon a little more complex than the last from the
/// accumulated noise, until a later round's `intersection`/`difference`
/// call measurably slowed down (see [`MIN_KEPT_PART_AREA_M2`]'s own docs on
/// the same failure shape). Snapping every polygon this module carries
/// forward onto a shared grid closes that loop: the same real corner
/// reached two different ways always lands on exactly the same point, so
/// there's no float-epsilon gap left for a later round to rediscover as
/// new.
///
/// Finer converges *better*, not worse, right down to the smallest grid
/// that still absorbs real float noise: a coarser grid doesn't just fail to
/// help, it makes its own new problem, since every round's own rounding is
/// itself a source of positional disagreement between two sides that were
/// exactly coincident before it — confirmed by trying coarser values on the
/// same real data (1mm and 1cm both left more pairs unresolved after
/// [`MAX_RESOLUTION_ROUNDS`] than 0.1mm did, 1cm markedly more than 1mm).
/// 0.1mm is comfortably above where real float noise from `geo`'s own
/// fixed-point core lives (empirically sub-micrometre) and comfortably
/// below anything that could ever be a real, intended geometric feature of
/// a road or sidewalk — real Barcelona data converges in a single round at
/// this value, with nothing left over for [`resolve_overlaps`]'s own
/// "still overlap after N rounds" fallback to ever report.
const SNAP_GRID_METERS: f64 = 0.0001;

/// `snap_coords` itself, plus the cleanup its own rounding makes necessary:
/// two consecutive vertices that were merely *close* before snapping can
/// become *identical* after it, leaving a zero-length edge — harmless in
/// itself, but a client reading two coincident points as a real edge sees a
/// spurious 0° interior angle where there's no actual spike, or, if enough
/// of them line up, a self-touch a bowtie check flags as a crossing.
/// Deduplicating consecutive coordinates removes the zero-length edge along
/// with it, closing the gap between "snapped to the same point" and "reads
/// as a real corner".
///
/// Unlike every other transform in this module, [`geo::MapCoords`] is a raw
/// per-coordinate rewrite with no simplicity guarantee of its own —
/// rounding two vertices independently can, in principle, drag one edge
/// across another that wasn't crossing before. A tempting fix is routing
/// the result back through [`geo::BooleanOps::union`] with itself (`i_overlay`,
/// the crate backing it, documents accepting self-intersecting input
/// directly) — but that's not the free repair it looks like: tried on real
/// Barcelona data, it introduced far more damage than it fixed, corrupting
/// hundreds of zones that were never self-intersecting in the first place
/// (a self-union isn't a documented no-op for already-simple, multi-part
/// input, and evidently isn't one in practice either). Left as plain
/// snap-and-dedup instead — leaves a small, known number of real Barcelona
/// rings on very tightly-bent short chains still capable of a hairline
/// self-touch, which is a real, narrower gap this module doesn't yet close
/// (see the crate's own follow-up notes), but doesn't risk the geometry
/// that was already correct to chase it.
fn snap_coords(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    let snap = |v: f64| (v / SNAP_GRID_METERS).round() * SNAP_GRID_METERS;
    polygon.map_coords(|c| Coord { x: snap(c.x), y: snap(c.y) })
}

/// Every `(i, j)` index pair into `polygons` whose polygons overlap by more
/// than [`OVERLAP_AREA_THRESHOLD_M2`] — the local-coordinate counterpart of
/// [`overlapping_zone_ids`], used internally by [`resolve_overlaps`], which
/// needs indices to mutate rather than the ids that function's own public
/// contract returns.
fn overlapping_pairs(polygons: &[MultiPolygon<f64>]) -> Vec<(usize, usize)> {
    let mut found = Vec::new();
    for i in 0..polygons.len() {
        for j in (i + 1)..polygons.len() {
            if overlap_area_m2(&polygons[i], &polygons[j]) > OVERLAP_AREA_THRESHOLD_M2 {
                found.push((i, j));
            }
        }
    }
    found
}

/// Every `(i, j)` pair in `pairs` whose polygons still overlap, checked
/// against `polygons` as it currently stands — the same test
/// [`overlapping_pairs`] runs, just against an already-known candidate list
/// instead of scanning every possible pair in `polygons` again. Both fixes
/// [`resolve_overlaps`] applies — shrinking padding, cutting away a real
/// overlap — only ever shrink a polygon, never grow one, so a pair that
/// wasn't found overlapping in a first full [`overlapping_pairs`] scan can
/// never become one later: restricting every later round to the candidates
/// that first scan already found is safe, and turns what would otherwise be
/// a fresh `O(n²)` scan of the *entire* network on every round into one
/// proportional only to however many zones are actually contested — a
/// small fraction of a whole city's worth of zones in practice.
fn still_overlapping(polygons: &[MultiPolygon<f64>], pairs: &[(usize, usize)]) -> Vec<(usize, usize)> {
    pairs
        .iter()
        .copied()
        .filter(|&(i, j)| overlap_area_m2(&polygons[i], &polygons[j]) > OVERLAP_AREA_THRESHOLD_M2)
        .collect()
}

/// The most [`zone_polygon`] can pad `zone`'s own entries out to, up to
/// `upper`, without the result overlapping `opposing` — binary search
/// rather than the all-or-nothing choice between `upper` and `0.0`:
/// dropping straight to zero the instant the full default overlaps
/// anything turns a real but merely-a-bit-too-short lane back into the
/// paper-thin sliver [`MIN_DRAWN_LANE_LENGTH_METERS`] exists to avoid, when
/// often only a metre or two of it was ever the problem.
///
/// Assumes overlap is monotonic in the pad amount — true by construction,
/// since [`padded_entry`] only ever extends an entry *backward* along a
/// fixed line as the target length grows, never sideways or in any other
/// direction that could newly clear an obstruction a smaller pad had
/// already reached. If even `0.0` overlaps `opposing`, this converges to
/// `0.0` and reports it rather than special-casing that check up front: the
/// padding isn't the problem in that case, which is exactly what
/// [`resolve_overlaps`]'s own cutting phase is for.
fn max_safe_pad(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    upper: f64,
    opposing: &MultiPolygon<f64>,
) -> Result<f64> {
    let overlaps_at = |pad: f64| -> Result<bool> {
        Ok(overlap_area_m2(&zone_polygon(zone, lanes, successors, pad)?, opposing) > OVERLAP_AREA_THRESHOLD_M2)
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

/// How many bisection steps [`max_safe_pad`] runs — 10 halvings of
/// [`MIN_DRAWN_LANE_LENGTH_METERS`]'s own 5m bottom out under a millimetre,
/// far tighter than [`OVERLAP_AREA_THRESHOLD_M2`] could even distinguish,
/// so more steps would only buy precision nothing downstream can tell
/// apart from what this already gives.
const PAD_SEARCH_STEPS: u32 = 10;

/// Past this many outer rounds of [`resolve_overlaps`], give up and leave
/// whatever's left overlapping rather than looping forever — real Barcelona
/// data converges in 2 rounds, so this is headroom for a much messier
/// network, not a figure anything is tuned against.
const MAX_RESOLUTION_ROUNDS: u32 = 8;

/// Fixes up `polygons` (already built by [`zone_polygon`] with padding on,
/// one-to-one with `zones`, in the network's own local coordinates) so no
/// two different zones' polygons overlap — see [`overlapping_zone_ids`]'s
/// own docs for why that has to hold for the client-facing output. A no-op,
/// and cheap, for the overwhelming majority of real networks where nothing
/// overlaps in the first place: the one real `O(n²)` scan below is exactly
/// [`overlapping_zone_ids`]'s own cost, paid once regardless of whether it
/// finds anything ([`still_overlapping`]'s own docs cover why every later
/// round's own check is cheap instead of repeating it).
///
/// Two independent fixes, tried in order:
///
/// 1. **Shrink padding.** [`padded_entry`]'s own straight-line
///    extrapolation is a heuristic, and a wrong one often enough in
///    practice to be worth checking rather than trusting outright.
///    [`max_safe_pad`] binary-searches for the most padding that still
///    avoids the specific neighbour a zone was found overlapping, and only
///    bottoms out at zero when even that doesn't help — a genuine,
///    non-padding overlap for the cutting phase to handle instead.
/// 2. **Cut.** Whatever's left over is a genuine geometric adjacency — real
///    lane width, a sharp fork — that shrinking padding can't touch. Both
///    sides simply give back the ground they actually contest: `overlap =
///    polygons[i].intersection(&polygons[j])` is real, disputed ground
///    neither side's own detector gates are entitled to claim ambiguously,
///    and `polygons[i].difference(&overlap)` removes it from both — after
///    which neither can possibly still intersect the other, because
///    whatever they used to share is now excluded from both by
///    construction (`(A - C) ∩ B ⊆ (A ∩ B) - (C ∩ B) = C - C = ∅` when `C =
///    A ∩ B`). No direction to compute, no extent to size, and no failure
///    case where a cut can't be found — an earlier version of this function
///    instead built a half-plane cut from the two shapes' own centroids,
///    and hit real trouble when a zone's own combined shape (core plus
///    however many extended-ancestor chain parts) was far bigger than the
///    specific ground actually contested: the centroid the direction was
///    built from sat nowhere near the real conflict (confirmed on real
///    Barcelona data: a bent 80m ribbon's own cut removed over 80% of it in
///    one shot, instead of the small local wedge the real overlap called
///    for). Subtracting the literal overlap has no such failure mode to
///    guard against, because it was never asked a directional question, or
///    given a whole zone's worth of irrelevant shape to be misled by, in
///    the first place.
///
/// A polygon contested by *several* neighbours at once still needs every
/// one of those cuts computed against `polygons` exactly as this round
/// found it, before any of the round's own cuts are applied: even though
/// `intersection`/`difference` themselves don't have an ordering
/// precondition, computing a *second* neighbour's own overlap against a
/// polygon *already* shrunk by a first cut would silently understate it.
/// The round loop's own top is already that stable snapshot, so there's
/// nothing further to track.
fn resolve_overlaps(
    zones: &[E3Detector],
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    stop_points: &[Coord<f64>],
    polygons: &mut [MultiPolygon<f64>],
) -> Result<()> {
    // Every zone pair that could possibly still need fixing, for the rest
    // of this function's own life — one real `O(n²)` scan across the whole
    // network, never repeated (see [`still_overlapping`]'s own docs for why
    // that's sound: both fixes below only ever shrink a polygon, so a pair
    // that isn't here yet can never become one later).
    let candidates = overlapping_pairs(polygons);
    if candidates.is_empty() {
        return Ok(());
    }

    // Every zone's current padding target, in metres — absent means still
    // at the full [`MIN_DRAWN_LANE_LENGTH_METERS`] default, never having
    // needed shrinking. [`max_safe_pad`] only ever lowers an entry, never
    // raises one back up: a zone that no longer overlaps anything once its
    // padding is smaller stays that size rather than creeping back toward
    // the look-nicer default, since re-growing it could just reopen the
    // same overlap on a later round.
    let mut pad_meters: HashMap<usize, f64> = HashMap::new();

    for _round in 0..MAX_RESOLUTION_ROUNDS {
        let overlaps = still_overlapping(polygons, &candidates);
        if overlaps.is_empty() {
            return Ok(());
        }

        let mut progressed = false;
        for &(i, j) in &overlaps {
            for &(shrink, opposing) in &[(i, j), (j, i)] {
                let current = pad_meters.get(&shrink).copied().unwrap_or(MIN_DRAWN_LANE_LENGTH_METERS);
                if current <= 0.0 {
                    continue;
                }
                let best = max_safe_pad(&zones[shrink], lanes, successors, current, &polygons[opposing])?;
                if best < current {
                    polygons[shrink] = zone_polygon(&zones[shrink], lanes, successors, best)?;
                    pad_meters.insert(shrink, best);
                    progressed = true;
                }
            }
        }

        // Every cut this round needs, computed against `polygons` exactly
        // as this round found it (before any of them are applied — see
        // this function's own docs on why a cut derived from an
        // already-cut shape isn't the same cut any more), collected per
        // zone so one contested by several neighbours at once gets every
        // one of its own overlaps subtracted together rather than one at a
        // time against a moving target. The cut itself is just the real
        // overlap — see this function's own docs for why that's enough on
        // its own, with no direction to compute or extent to size.
        let mut cuts: HashMap<usize, Vec<GeoPolygon<f64>>> = HashMap::new();
        for (i, j) in still_overlapping(polygons, &candidates) {
            let overlap = polygons[i].intersection(&polygons[j]);
            cuts.entry(i).or_default().extend(overlap.0.iter().cloned());
            cuts.entry(j).or_default().extend(overlap.0);
            progressed = true;
        }
        for (zone, overlaps) in cuts {
            // Unioned first, not handed to `difference` as a raw, possibly
            // self-overlapping list: two different neighbours' own overlap
            // regions can themselves touch or overlap (adjacent contested
            // ground at a busy corner), and a subtrahend `difference` never
            // otherwise gets to clean up itself can carry that same
            // near-degenerate geometry straight into the harder, two-input
            // operation.
            let subtrahend = overlaps
                .into_iter()
                .map(|part| MultiPolygon::new(vec![part]))
                .reduce(|acc, part| acc.union(&part))
                .unwrap_or_else(|| MultiPolygon::new(Vec::new()));
            // Cut (and `keep_part_near`-filtered — see its own docs) *per
            // part*, not the zone's own whole `MultiPolygon` in one go: a
            // zone can legitimately already have more than one disjoint
            // part before this round ever touches it (the core plus each
            // extended-ancestor chain, none of which necessarily overlap
            // each other — `zone_polygon`'s own docs), and a neighbour's
            // own overlap this round is essentially never with *every* one
            // of those parts at once. Cutting the whole `MultiPolygon`
            // as one `difference` call already leaves an untouched part
            // untouched (`difference` only ever removes what's actually in
            // the subtrahend) — the bug was applying `keep_part_near`
            // *after* that, over the combined result: confirmed on real
            // Barcelona data, a zone with several genuinely separate,
            // untouched chain parts had every one of them but the single
            // closest-to-the-stop-line discarded the moment *any* one part
            // needed cutting, not just whichever fragment the cut itself
            // stranded. Splitting the loop by part keeps that distinction:
            // an untouched part is pushed straight through, only a part
            // that actually intersects `subtrahend` is cut and filtered.
            let mut new_parts = Vec::with_capacity(polygons[zone].0.len());
            for part in &polygons[zone].0 {
                let part_polygon = MultiPolygon::new(vec![part.clone()]);
                if overlap_area_m2(&part_polygon, &subtrahend) <= OVERLAP_AREA_THRESHOLD_M2 {
                    new_parts.push(part.clone());
                    continue;
                }
                let cut = part_polygon.difference(&subtrahend);
                // Deliberately no `simplify` here, unlike [`zone_polygon`]'s
                // own build: Douglas-Peucker approximates each side of a cut
                // independently, and two zones' own polygons — both cut
                // along the exact same shared boundary this round — can
                // drift centimetres apart from each other once each one's
                // own simplification picks a different nearby point to
                // keep. That reopens a real, sizeable "overlap" (or gap)
                // for the *next* round to rediscover: confirmed on real
                // Barcelona data, simplifying every round left *more*
                // pairs unresolved after 8 rounds, not fewer. Left
                // unsimplified here, a cut zone's own polygon can carry a
                // few more vertices than [`zone_polygon`]'s own output
                // normally would — a minor cost against never disturbing
                // the exact non-overlap this round's own cut just
                // established.
                //
                // Also deliberately no `snap_coords`: that was
                // load-bearing against the former quad-union
                // `buffer_shape`'s own float noise (see its own docs), but
                // with a real stroke offset behind every polygon this
                // function starts from, rounding every vertex onto a
                // shared grid on top of that no longer measurably helps
                // convergence and net loses ground on the
                // self-intersection coherence check (confirmed by trying
                // both ways on real Barcelona data) — [`geo::MapCoords`]'s
                // own lack of a simplicity guarantee (see [`snap_coords`]'s
                // own docs) is a real cost with nothing left here to buy
                // back.
                let cleaned =
                    keep_part_near(drop_slivers(cut), stop_points[zone]);
                new_parts.extend(cleaned.0);
            }
            polygons[zone] = MultiPolygon::new(new_parts);
        }

        if !progressed {
            break; // nothing left that another round could resolve
        }
    }

    for (i, j) in still_overlapping(polygons, &candidates) {
        eprintln!(
            "{ERROR}error:{ERROR:#} zones {:?} and {:?} still overlap after \
             {MAX_RESOLUTION_ROUNDS} resolution rounds — left as-is",
            zones[i].id, zones[j].id,
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
///   (`merged_core_polygon`'s own docs), and a car lane sitting right next to
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
/// [`merged_core_polygon`] takes over instead), or another real merge point
/// (`in_degree(successor) != 1` — more than one ancestor feeding forward
/// into it, or, for a `start` that's itself past the last real fork,
/// none at all). The final `via` bridging into whichever one stops the
/// walk *is* included, so the segment's own polygon reaches exactly to
/// where the next one begins, with no gap between them, matching every
/// other hop along the way.
///
/// This is what turns what used to be one independent rectangle per
/// extended-ancestor lane into a single seamless polygon (via
/// [`buffer_shape`]) spanning the whole segment — real Barcelona data has a
/// visible gap between two such rectangles at a bend often enough that
/// this exists: [`trimmed_segments`] already handles a single lane's own
/// multi-point bend correctly, and a run of lanes physically connected end
/// to end is no different, once their shapes are concatenated into one.
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
/// The third element of the returned tuple is the real lane id the walk
/// stopped *at* — the zone's own core lane when this chain feeds straight
/// into one, `None` at a genuine dead end. [`zone_polygon`] uses it to
/// buffer a chain and the single core gate it feeds as one continuous path
/// instead of two separately-capped shapes unioned together — see its own
/// docs on why that seam used to spike.
fn chain_shape<'a>(
    start: &'a str,
    ancestor_lanes: &BTreeSet<&str>,
    in_degree: &HashMap<&str, usize>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    lanes: &HashMap<&str, &'a Lane>,
) -> Option<(Vec<&'a Lane>, Shape, Option<String>)> {
    let mut chain_lanes: Vec<&Lane> = Vec::new();
    let mut points: Vec<Point> = Vec::new();
    let mut visited: HashSet<&str> = HashSet::new();
    let mut current = start;
    let mut terminal_successor: Option<String> = None;

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
        terminal_successor = Some(successor.to_string());
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

    (points.len() >= 2).then_some((chain_lanes, Shape(points), terminal_successor))
}

/// Below this, in metres, [`geo::Simplify`] treats an intermediate ring
/// vertex as contributing nothing a straight line between its neighbours
/// doesn't already capture — chosen well under any real visual or
/// geometric significance (the same "a centimetre is noise" reasoning
/// [`OVERLAP_AREA_THRESHOLD_M2`]'s own docs use). Applied once, to
/// [`zone_polygon`]'s own finished polygon: [`chain_shape`] concatenates
/// however many real lanes a zone's extension walks through end to end,
/// and a real Barcelona chain often includes several sub-metre "connector"
/// lanes (see its own module docs) whose shape points land mere
/// centimetres apart once strung together — real detail `netconvert`
/// recorded, but far finer than this crate's own output needs to
/// reproduce faithfully, and it only bloats the GeoJSON a client downloads
/// for no visual or geometric benefit any of those extra points buy back.
const SIMPLIFY_TOLERANCE_METERS: f64 = 0.05;

/// `zone`'s own waiting-area polygon, in the network's own local
/// coordinates: the core group's own [`merged_core_polygon`], unioned with
/// one polygon per extended-ancestor chain (see [`chain_shape`]'s own
/// docs). `entries` and `exits` aren't 1:1 (extension only adds entries,
/// never exits) — matched back up here by lane: an entry whose lane is one
/// of `zone`'s own exits is "core" (possibly `pad_meters`-extended, but
/// otherwise the zone's own controlled lane) and feeds
/// [`merged_core_polygon`]; every other entry is an extended ancestor,
/// grouped into whichever [`chain_shape`] it belongs to and unioned in
/// separately. A chain that happens to touch or overlap the core (or
/// another chain) merges into one seamless shape automatically — `union`
/// already gives [`merged_core_polygon`] that guarantee, so there's
/// nothing extra to arrange for it here.
///
/// `pad_meters` is the target [`padded_entry`] pads every entry span out
/// to — a whole chain's own *total* length, not each lane in it
/// separately: [`MIN_DRAWN_LANE_LENGTH_METERS`] builds the normal, looks-
/// nicer-on-a-map shape [`to_feature_collection`] uses by default, `0.0`
/// builds the same zone at its real, unpadded size, and anything in
/// between is [`resolve_overlaps`]'s own way of asking for as much padding
/// as still fits without reaching into a neighbour.
fn zone_polygon(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    pad_meters: f64,
) -> Result<MultiPolygon<f64>> {
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
    for exit in &zone.exits {
        let lane = resolve(&exit.lane)?;
        exit_position_by_lane.insert(exit.lane.0.as_str(), distance_from_start(exit.position, lane.length));
    }

    // A pedestrian zone's own core lanes are walkingareas — see
    // [`pedestrian_lane_polygon`]'s own docs for why those need their
    // `shape` used directly as a polygon rather than measured for an
    // entry/exit distance and stroke-buffered like every other lane's.
    let is_pedestrian = !zone.detect_persons.is_empty();

    let mut core_gates = Vec::with_capacity(zone.exits.len());
    let mut pedestrian_core_lanes = Vec::with_capacity(zone.exits.len());
    let mut ancestor_lanes: BTreeSet<&str> = BTreeSet::new();
    for entry in &zone.entries {
        let lane = resolve(&entry.lane)?;

        if let Some(&exit_distance) = exit_position_by_lane.get(entry.lane.0.as_str()) {
            if is_pedestrian {
                pedestrian_core_lanes.push(lane);
            } else {
                let entry_distance = distance_from_start(entry.position, lane.length);
                core_gates.push((lane, entry_span(entry_distance, exit_distance), exit_distance));
            }
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
            // covers it. Never populated for a pedestrian zone in
            // practice — `zone_generator::pedestrian_zones` never extends
            // backward — so the ancestor-chain code below is a no-op for
            // one, not a second, competing way to handle the same lanes.
            ancestor_lanes.insert(entry.lane.0.as_str());
        }
    }

    // A core gate fed by exactly one ancestor lane can be absorbed into
    // that chain's own buffer below — one continuous path, stroked once,
    // rather than two separately-capped shapes unioned together at a seam
    // (see the `absorbed_core_gates` loop's own docs for why that seam
    // used to spike). A gate fed by more than one ancestor (a real merge
    // right at the stop line) stays out of this — there's no single chain
    // to absorb it into unambiguously — and falls back to
    // `merged_core_polygon` exactly as before.
    let core_gate_index_by_lane: HashMap<&str, usize> =
        core_gates.iter().enumerate().map(|(i, &(lane, _, _))| (lane.id.0.as_str(), i)).collect();
    let mut core_predecessor_count: HashMap<&str, usize> = HashMap::new();
    for &lane in &ancestor_lanes {
        if let Some(&(successor, _)) = successors.get(lane)
            && core_gate_index_by_lane.contains_key(successor)
        {
            *core_predecessor_count.entry(successor).or_insert(0) += 1;
        }
    }
    let absorbed_core_gates: HashSet<usize> = core_gate_index_by_lane
        .iter()
        .filter(|&(&lane_id, _)| core_predecessor_count.get(lane_id).copied() == Some(1))
        .map(|(_, &i)| i)
        .collect();

    let mut polygon = if is_pedestrian {
        merged_pedestrian_polygon(&pedestrian_core_lanes)
    } else {
        let unabsorbed_core_gates: Vec<_> = core_gates
            .iter()
            .enumerate()
            .filter(|(i, _)| !absorbed_core_gates.contains(i))
            .map(|(_, &gate)| gate)
            .collect();
        merged_core_polygon(&unabsorbed_core_gates)
    };

    // How many *other* ancestors feed forward into each ancestor lane —
    // `chain_shape`'s own segmentation depends on this, not just on
    // finding leaves: a lane with more than one real predecessor (a real
    // street merge) has to start its *own* segment rather than being swept
    // into either predecessor's, exactly as much as a leaf (nothing
    // feeding into it at all) does — see `chain_shape`'s own docs for why
    // building it a second time, once per predecessor, was actively wrong
    // rather than merely redundant.
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
        let Some((chain_lanes, shape, terminal_successor)) =
            chain_shape(start, &ancestor_lanes, &in_degree, successors, lanes)
        else {
            continue;
        };

        // This chain feeds straight into a core gate only it supplies —
        // buffer chain and core as one continuous path instead of
        // unioning two separately-built shapes. `buffer_shape`'s own
        // internal round joins then smooth out the chain's own bends
        // exactly as they would within a single lane's shape, and the
        // real stop line (the core gate's own exit) gets a flat, `Butt`
        // cap with nothing left to union against it and pinch or
        // overshoot past it — the seam `LineCap::Square` below only ever
        // patched, not fixed (see `buffer_shape`'s own docs on that seam).
        if let Some(gate_idx) =
            terminal_successor.as_deref().and_then(|id| core_gate_index_by_lane.get(id)).copied()
            && absorbed_core_gates.contains(&gate_idx)
        {
            let (core_lane, _core_entry, core_exit_distance) = core_gates[gate_idx];
            let chain_length = shape_length(&shape);
            let mut combined_points = shape.0.clone();
            combined_points.extend(core_lane.shape.0.iter().copied());
            let combined_shape = Shape(combined_points);
            let total_exit = chain_length + core_exit_distance;
            let entry_distance = entry_span(Length::new::<meter>(0.0), total_exit);
            let half_width = core_lane.width / 2.0;
            polygon =
                polygon.union(&buffer_shape(&combined_shape, entry_distance, total_exit, half_width, LineCap::Butt));
            continue;
        }

        let total_length = shape_length(&shape);
        let entry_distance = entry_span(Length::new::<meter>(0.0), total_length);
        let half_width = chain_lanes[0].width / 2.0;
        // `Round`, not `Butt`, at this chain's own connecting end — see
        // `buffer_shape`'s own docs for why a flat cap right where this
        // unions into the core can pinch instead of joining cleanly. Only
        // reached when the gate above didn't already absorb this chain
        // (a real merge right at the stop line, more than one chain
        // feeding the same gate).
        let end_cap = LineCap::Square;
        polygon = polygon.union(&buffer_shape(&shape, entry_distance, total_length, half_width, end_cap));
    }

    // `simplify` is aimed at a long extended-ancestor chain's own sub-metre
    // "connector" lane noise (see `SIMPLIFY_TOLERANCE_METERS`'s own docs);
    // a walkingarea's own shape is already a compact, few-metre outline
    // with its real corners close together, and Douglas-Peucker collapsing
    // even a mild real bend near one of those corners changes which chord
    // spans it — measurably sharpening the angle that survives rather than
    // leaving it alone. Skipped for a pedestrian zone's own polygon for
    // that reason; `drop_slivers`/`snap_coords` still apply, since neither
    // one repositions a real vertex the way `simplify` does.
    let polygon = if is_pedestrian { polygon } else { polygon.simplify(SIMPLIFY_TOLERANCE_METERS) };
    Ok(snap_coords(drop_slivers(polygon)))
}

/// `zone`'s own `stop_line`: the average, in the network's own local
/// coordinates, of the point on each of `zone`'s own exit lanes at its own
/// exit gate — a single representative point even when the zone spans
/// several lanes (and so several individual stop lines). Independent of
/// `pad_meters`/[`zone_polygon`]: an exit's own position never moves under
/// padding (see [`padded_entry`]'s own docs — only an entry ever does), so
/// this only needs computing once per zone, not once per padding attempt
/// [`resolve_overlaps`] tries.
fn stop_line_point(zone: &E3Detector, lanes: &HashMap<&str, &Lane>) -> Result<Point> {
    let mut stop_points = Vec::with_capacity(zone.exits.len());
    for exit in &zone.exits {
        let lane = lanes.get(exit.lane.0.as_str()).copied().with_context(|| {
            format!("zone {:?} references lane {:?}, which isn't in the network", zone.id, exit.lane)
        })?;
        let exit_distance = distance_from_start(exit.position, lane.length);
        stop_points.push(point_and_tangent_at(&lane.shape, exit_distance).0);
    }
    Ok(centroid(&stop_points))
}

/// Converts `polygon` (already resolved, in the network's own local
/// coordinates — see [`zone_polygon`]/[`resolve_overlaps`]) into `zone`'s
/// finished GeoJSON `Feature`: a `MultiPolygon` geometry reprojected to
/// WGS84 lon/lat, plus the `waiting_zone_id`/`stop_line`/`modes` triple of
/// properties. The only place in the pipeline [`Reprojector::to_lon_lat`]
/// runs (see the module docs) — everything upstream of this stays in local
/// metres.
fn build_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    polygon: &MultiPolygon<f64>,
    reproject: &Reprojector,
) -> Result<Feature> {
    let stop_line = reproject.to_lon_lat(stop_line_point(zone, lanes)?)?;

    let ring = |line: &LineString<f64>| -> Result<Vec<Position>> {
        line.coords().map(|c| reproject.to_lon_lat(Point { x: c.x, y: c.y, z: 0.0 }).map(Position::from)).collect()
    };
    let polygons = polygon
        .0
        .iter()
        .map(|part| {
            let mut rings = vec![ring(part.exterior())?];
            for interior in part.interiors() {
                rings.push(ring(interior)?);
            }
            Ok(rings)
        })
        .collect::<Result<Vec<_>>>()?;

    let mut properties = JsonObject::new();
    properties.insert("waiting_zone_id".to_string(), zone.id.0.clone().into());
    properties.insert("stop_line".to_string(), serde_json::json!(stop_line));
    properties.insert("modes".to_string(), serde_json::json!(zone_modes(zone, lanes)));

    let mut feature = Feature::from(Geometry::new_multi_polygon(polygons));
    feature.properties = Some(properties);
    Ok(feature)
}

/// [`zone_polygon`] and [`build_feature`] combined into `zone`'s finished
/// `Feature` in one call — a convenience for building (or testing) a single
/// zone's own output in isolation; [`to_feature_collection`] itself calls
/// the two separately so [`resolve_overlaps`] can work with every zone's
/// own local-coordinate polygon directly, without reprojecting and
/// un-reprojecting on every attempt. Test-only in practice — every
/// production call site needs that split — kept as a real (if
/// `#[cfg(test)]`) function rather than inlined into each test, since
/// several of them build a `Feature` this same way.
#[cfg(test)]
fn zone_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    reproject: &Reprojector,
    pad_meters: f64,
) -> Result<Feature> {
    build_feature(zone, lanes, &zone_polygon(zone, lanes, successors, pad_meters)?, reproject)
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

    let mut polygons = zones
        .iter()
        .map(|zone| zone_polygon(zone, &lanes, &successors, MIN_DRAWN_LANE_LENGTH_METERS))
        .collect::<Result<Vec<_>>>()?;
    // `resolve_overlaps`'s own reference point per zone — see
    // `keep_part_near`'s own docs for what it's for.
    let stop_points = zones
        .iter()
        .map(|zone| stop_line_point(zone, &lanes).map(|p| Coord { x: p.x, y: p.y }))
        .collect::<Result<Vec<_>>>()?;

    resolve_overlaps(zones, &lanes, &successors, &stop_points, &mut polygons)?;

    let features = zones
        .iter()
        .zip(&polygons)
        .map(|(zone, polygon)| build_feature(zone, &lanes, polygon, &reproject))
        .collect::<Result<Vec<_>>>()?;

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
    /// needed for a multi-lane zone's own tests, which build a real
    /// same-edge group of lanes rather than just one lane repeated.
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
    /// shape [`merged_core_polygon`] exists for. Every lane's own entry/exit
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
        // e0, e1 and e2 are collinear and meet exactly end to end, so their
        // union is genuinely one seamless 60m ribbon, not three independent
        // rectangles -- nor even two separately-tracked "core" and "chain"
        // polygons that merely happen to touch: a real `union` merges
        // anything that touches into one connected polygon, which is a
        // strictly better result than the former design's own "list of
        // rings, however each one was built" ever guaranteed.
        assert_eq!(coordinates.len(), 1, "expected one seamless polygon for e0+e1+e2 combined");

        let ring = &coordinates[0][0];
        assert!(!ring_self_intersects(ring), "{ring:?}");
        let lats = ring.iter().map(|p| p[1]);
        let lat_span_m = (lats.clone().fold(f64::MIN, f64::max) - lats.fold(f64::MAX, f64::min)) * 111_320.0;
        assert!(
            lat_span_m > 55.0,
            "expected the merged e0+e1+e2 ribbon to span close to their combined 60m, \
             got {lat_span_m:.1}m"
        );
    }

    #[test]
    fn two_leaves_merging_into_a_shared_ancestor_produce_no_duplicated_ground() {
        // A real Y-merge: `e_leaf_a` and `e_leaf_b` are two independent
        // real streets that both feed into `e_mid`, which then feeds into
        // the zone's own core (`e0`) -- the exact shape a real Barcelona
        // corner hit (see `chain_shape`'s own module docs). `e_mid` has two
        // real predecessors (`in_degree` 2), so it has to start its *own*
        // segment rather than being drawn a second time by each branch: an
        // earlier, hand-rolled version of this pipeline drew every branch's
        // own segment as an independent ring, and Leaflet's own default
        // `evenodd` fill rule turns ground covered by two of a zone's own
        // rings into a rendered hole -- a real, visible defect on real
        // Barcelona data. `chain_shape` still stops each branch's own walk
        // at `e_mid` for that reason (see its own docs), but now that every
        // segment is unioned into one `MultiPolygon` rather than kept as
        // independent rings regardless of whether they touch, duplicated
        // ground isn't just avoided at the `chain_shape` level any more --
        // `union` couldn't double-count it even if it were still handed
        // twice, which is what this test actually checks: the merged
        // area is real coverage, not gap nor duplication.
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
        // Every segment (core, e_mid, both leaves) touches at least one
        // other at a real junction corner, so their union is one connected
        // polygon -- there's no "was e_mid drawn twice" question left to
        // ask ring-by-ring any more, since a real union structurally can't
        // double-count the ground two of its own inputs share.
        assert_eq!(coordinates.len(), 1, "expected one polygon covering the whole merged shape");
        let ring = &coordinates[0][0];
        assert!(!ring_self_intersects(ring), "{ring:?}");

        // Real, non-overlapping area: e0 (20m) + e_mid (20m) + leaf_a/b
        // (√(20²+20²) ≈ 28.28m each), all 3.2m wide ⇒ 64 + 64 + 90.5 + 90.5
        // ≈ 309m². If e_mid's own ~64m² were still counted twice (the bug
        // `chain_shape` itself guards against — see its own docs), the
        // total would jump to ~373m²; if the merge lost real ground at the
        // Y instead, it would fall well short of 309m².
        let points: Vec<(f64, f64)> = ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
        assert!(
            (270.0..340.0).contains(&area_m2),
            "expected the merged Y-shape's own real area close to 309m² (64 + 64 + 90.5 \
             + 90.5, minus a little for the sharp `via`-less corners' own approximation), \
             got {area_m2:.1}m2 -- too high looks like e_mid's own ground duplicated, too \
             low looks like real ground lost at the merge"
        );
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

    /// Twice the signed area of triangle `a`, `b`, `c` — positive when `c`
    /// is left of the ray `a -> b`, negative when right, zero when
    /// collinear. Test-only: production code answers every question this
    /// used to answer (a lane's own bend, a ring's own self-intersection,
    /// two rings' own overlap) through [`geo::BooleanOps`] instead — this
    /// still backs a couple of tests that check *that* replacement against
    /// a hand-computed geometric primitive directly, rather than trusting
    /// `geo` circularly.
    fn orientation(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    }

    /// The signed area enclosed by `ring` (the shoelace formula, no closing
    /// repeat) — positive for a counterclockwise winding, negative for
    /// clockwise. Test-only, for the same reason as [`orientation`].
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

    /// Whether any two non-adjacent edges of closed ring `ring` (GeoJSON
    /// style: first position repeated last) properly cross, via
    /// [`orientation`]. A self-intersecting ("bowtie") polygon is invalid
    /// GeoJSON a client's point-in-polygon test can't reason about at all —
    /// every coherence test in this module checks real output against this
    /// directly, rather than trusting that [`geo::BooleanOps`] alone is
    /// enough to guarantee it (it should be; this is the check that would
    /// catch it if it somehow weren't).
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
    /// exactly the shape [`buffer_shape`]'s per-segment quads, unioned
    /// rather than joined and cleaned up by hand, are meant to handle
    /// correctly regardless of how tight the turn is.
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
    fn never_bridges_across_a_lane_gap_the_zone_does_not_actually_claim() {
        // Lane 1 sits physically between lanes 0 and 2, but this zone only
        // claims 0 and 2 (as if lane 1 belonged to some other movement) --
        // exactly the shape a non-contiguous group `merged_core_polygon`'s
        // own docs describe. An earlier, hand-rolled version of this
        // pipeline picked the group's own leftmost and rightmost lane and
        // joined them directly regardless, which silently claimed lane 1's
        // own ground (not part of this zone) as if it belonged here too.
        // `merged_core_polygon` doesn't assume anything about lane order or
        // contiguity: lanes 0 and 2 don't actually touch, so their union is
        // honestly two separate polygons, with the real, unclaimed gap
        // between them left alone.
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
        assert_eq!(
            coordinates.len(),
            2,
            "lanes 0 and 2 don't touch (lane 1's own gap sits between them, unclaimed) -- \
             expected two separate polygons, not one bridged fraudulently across the gap"
        );
        for polygon in coordinates {
            assert!(!ring_self_intersects(&polygon[0]), "{:?}", polygon[0]);
        }
    }

    #[test]
    fn buffer_shape_never_self_intersects_on_a_short_wide_zigzag() {
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
            "buffer_shape produced a self-intersecting polygon for a short, wide, \
             sharply zigzagging lane: {ring:?}"
        );
    }

    #[test]
    fn buffer_shape_does_not_collapse_to_a_sliver_on_a_short_lane_with_tight_bends() {
        // A real Barcelona lane (`1395130587_2`, found while fixing an
        // earlier, hand-rolled version of this pipeline) shifted to the
        // origin: five segments, none longer than ~1.8m, with two real
        // direction changes -- short enough, and tight enough relative to a
        // 3.2m-wide lane, that a splice-based self-intersection cleanup
        // used to cut away real, non-crossing area along with the actual
        // self-crossing loop, collapsing a ~21.6m² ribbon down to 0.003m²
        // (a few points a few centimetres apart). `buffer_shape`'s union of
        // per-segment quads has no such collapse risk at all: `union`
        // never removes real, non-overlapping area, so there's nothing
        // here for a fallback to guard against any more.
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
             {naive_area_m2:.3}m2 for a {length:.2}m lane 3.2m wide: {ring:?}"
        );
    }

    /// The area two closed rings `a` and `b` (GeoJSON style: first position
    /// repeated last) actually have in common, via [`geo::BooleanOps::intersection`]
    /// — the test-only counterpart of [`overlap_area_m2`]/[`overlapping_zone_ids`]'s
    /// own production use of `intersection`, built directly from `Position`s
    /// rather than a [`MultiPolygon`] a caller already has, for tests that
    /// only have raw ring coordinates on hand.
    fn ring_overlap_area(a: &[Position], b: &[Position]) -> f64 {
        let polygon = |ring: &[Position]| {
            MultiPolygon::new(vec![GeoPolygon::new(
                LineString::new(ring.iter().map(|p| Coord { x: p[0], y: p[1] }).collect()),
                Vec::new(),
            )])
        };
        polygon(a).intersection(&polygon(b)).unsigned_area()
    }

    #[test]
    fn ring_overlap_area_handles_a_non_convex_ring_correctly() {
        // An L-shape (non-convex) and a 1x1 square entirely inside its own
        // vertical leg (x in [0,1], y in [1,2]) -- real, unambiguous
        // overlap of exactly 1.0. A hand-rolled clipper that assumes one
        // side of an overlap test is convex (an earlier version of this
        // module did) can silently give `0.0` for a real overlap like this
        // depending on which side lands where -- confirmed on real
        // Barcelona zones sharing several real square metres that measured
        // `0.0` one direction and multiple m² the other. `geo::BooleanOps`
        // makes no such assumption about either side.
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

        let square: Vec<Position> = [(0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0), (0.0, 1.0)]
            .into_iter()
            .map(|(x, y)| Position::from([x, y]))
            .collect();

        let forward = ring_overlap_area(&square, &l_shape);
        let backward = ring_overlap_area(&l_shape, &square);
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
    /// zone at ~0.22m², well above this).
    const MIN_PLAUSIBLE_RING_AREA_M2: f64 = 0.05;

    /// Below this, in degrees, a vertex is judged a convex spike — a
    /// needle-thin protrusion a client's own rendering (and a human looking
    /// at the map) reads as visibly wrong — rather than a real corner a
    /// waiting area's own shape can legitimately have. A waiting zone's
    /// boundary is either a straight lane edge (interior angle 180°, dead
    /// flat) or a real bend a round join (see `buffer_shape`'s own docs)
    /// turns smoothly; both stay well clear of 80° on any real street
    /// geometry, so a survivor below it is exactly the shape of defect this
    /// crate's own bug history is full of (a self-intersection a boolean op
    /// left uncleaned, a sliver from a `resolve_overlaps` cut, ...), not a
    /// legitimate acute corner this crate has any reason to draw.
    const MIN_INTERIOR_ANGLE_DEGREES: f64 = 80.0;

    /// Above this, in degrees, a vertex is judged a *reflex* spike — the
    /// concave mirror image of [`MIN_INTERIOR_ANGLE_DEGREES`]'s own convex
    /// one: a needle-thin notch cut *into* the polygon's own interior
    /// rather than protruding out of it, which an undirected angle-between-
    /// edges measure (the angle between two rays, always folded into
    /// `[0°, 180°]`) can't even represent as a value near 360° to catch —
    /// it reads a spike like this as the same small number a genuine convex
    /// spike would give, which is *usually* still caught by
    /// `MIN_INTERIOR_ANGLE_DEGREES` (a bad enough reflex spike folds well
    /// under 80° too) but not reliably close to the boundary, and reports a
    /// misleading angle either way. [`interior_angles_degrees`]'s own
    /// signed computation avoids folding in the first place, so this can be
    /// checked (and reported) directly, symmetric with the convex case
    /// around 180° (`360° - 280° = 80°`).
    const MAX_INTERIOR_ANGLE_DEGREES: f64 = 280.0;

    /// Below this, in degrees (roughly a centimetre at Barcelona's own
    /// latitude — see [`OVERLAP_AREA_THRESHOLD_DEG2`]'s own deg-to-metre
    /// conversion), an edge is too short for its direction to mean
    /// anything — a stray near-duplicate vertex from a boolean op can land
    /// this close to its own neighbour, and the angle either one of them
    /// forms with its *other*, real-length neighbour is noise, not a
    /// spike — skipped by [`interior_angles_degrees`] rather than reported
    /// as a false `0.0`.
    const MIN_EDGE_LENGTH_DEGREES: f64 = 1e-7;

    /// The interior angle, in degrees (`[0°, 360°)`), at every vertex of
    /// `points` (a ring's own distinct vertices — no repeated closing
    /// point) whose two neighbouring edges are both at least
    /// [`MIN_EDGE_LENGTH_DEGREES`] long: `180°` for a vertex a straight
    /// line already passes straight through, sliding down toward `0°` as
    /// the two segments fold back on themselves into a convex spike (the
    /// material pinching to a point), or up toward `360°` as they fold
    /// back the *other* way into a reflex spike (a thin notch cut into the
    /// material instead). Signed via the ring's own overall winding
    /// ([`signed_area`]'s own sign) rather than the plain undirected angle
    /// between the two edge vectors: that alternative can't distinguish
    /// "180° short of a full turn the convex way" from "180° short the
    /// reflex way" at all — both fold to the same value — which is exactly
    /// the distinction [`MAX_INTERIOR_ANGLE_DEGREES`]'s own check needs to
    /// mean anything.
    fn interior_angles_degrees(points: &[(f64, f64)]) -> Vec<f64> {
        let n = points.len();
        let counterclockwise = signed_area(points) >= 0.0;
        (0..n)
            .filter_map(|i| {
                let (prev, cur, next) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
                // Edge directions, not the "pointing away from the vertex"
                // rays an undirected measure would use: the sign of the
                // turn from the incoming edge to the outgoing one is what
                // tells convex from reflex apart.
                let (inx, iny) = (cur.0 - prev.0, cur.1 - prev.1);
                let (outx, outy) = (next.0 - cur.0, next.1 - cur.1);
                let (lin, lout) = (inx.hypot(iny), outx.hypot(outy));
                if lin < MIN_EDGE_LENGTH_DEGREES || lout < MIN_EDGE_LENGTH_DEGREES {
                    return None;
                }
                let cross = inx * outy - iny * outx;
                let dot = inx * outx + iny * outy;
                let turn_degrees = cross.atan2(dot).to_degrees(); // signed, (-180°, 180°]
                let interior = if counterclockwise { 180.0 - turn_degrees } else { 180.0 + turn_degrees };
                Some(((interior % 360.0) + 360.0) % 360.0)
            })
            .collect()
    }

    #[test]
    fn interior_angles_degrees_reports_a_square_corner_as_90_degrees_either_winding() {
        let ccw = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        assert_eq!(interior_angles_degrees(&ccw), vec![90.0, 90.0, 90.0, 90.0]);

        let cw: Vec<(f64, f64)> = ccw.into_iter().rev().collect();
        assert_eq!(interior_angles_degrees(&cw), vec![90.0, 90.0, 90.0, 90.0]);
    }

    #[test]
    fn interior_angles_degrees_reports_the_true_reflex_angle_not_its_convex_fold() {
        // The L-shape `ring_overlap_area_handles_a_non_convex_ring_correctly`
        // also uses: a 4x4 square with its own top-right 3x3 corner cut
        // out. The inner corner of that notch, at (1.0, 1.0), is a real
        // 270° reflex vertex -- an undirected angle-between-edges measure
        // would report this as 90° (360° - 270°), indistinguishable from a
        // real 90° convex corner and nowhere near `MAX_INTERIOR_ANGLE_DEGREES`.
        let l_shape = vec![(0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (1.0, 1.0), (1.0, 4.0), (0.0, 4.0)];
        let angles = interior_angles_degrees(&l_shape);
        assert_eq!(
            angles,
            vec![90.0, 90.0, 90.0, 270.0, 90.0, 90.0],
            "expected every outer corner at 90° and the notch's own inner corner at a true 270° reflex"
        );
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
                if ring_self_intersects(ring) {
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
                             (below {MIN_INTERIOR_ANGLE_DEGREES}°) — looks like a convex spike: {points:?}"
                        ));
                    } else if angle > MAX_INTERIOR_ANGLE_DEGREES {
                        failures.push(format!(
                            "{id} ring {ri} vertex {vi} has a {angle:.1}° interior angle \
                             (above {MAX_INTERIOR_ANGLE_DEGREES}°) — looks like a reflex spike: {points:?}"
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
    fn debug_diagnose_50926861_spike() {
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None);
        let lanes: HashMap<&str, &Lane> = network
            .edges
            .iter()
            .flat_map(|edge| &edge.lanes)
            .map(|lane| (lane.id.0.as_str(), lane))
            .collect();
        let successors = single_successors(&network);
        for id in ["50926861#0_straight", "50926859#0_straight"] {
            let zone = zones.iter().find(|z| z.id.0 == id).unwrap();
            let pre = zone_polygon(zone, &lanes, &successors, MIN_DRAWN_LANE_LENGTH_METERS).unwrap();
            println!("=== {id} PRE-cut: {} part(s) ===", pre.0.len());
            for part in &pre.0 {
                println!("  ring ({} pts): {:?}", part.exterior().0.len(), part.exterior().0);
            }
            let stop = stop_line_point(zone, &lanes).unwrap();
            println!("  stop_line_point: ({}, {})", stop.x, stop.y);
            for entry in &zone.entries {
                let lane = lanes.get(entry.lane.0.as_str()).unwrap();
                println!(
                    "  entry lane {:?} width={:?} first_pt={:?} last_pt={:?}",
                    entry.lane.0,
                    lane.width,
                    lane.shape.0.first(),
                    lane.shape.0.last(),
                );
            }
        }
        let collection = to_feature_collection(&network, &zones).unwrap();
        for id in ["50926861#0_straight", "50926859#0_straight"] {
            let feature = collection
                .features
                .iter()
                .find(|f| f.property("waiting_zone_id").unwrap().as_str().unwrap() == id)
                .unwrap();
            println!("=== {id} POST-cut ===");
            for ring in feature_rings(feature) {
                println!("  ring ({} pts): {:?}", ring.len(), ring);
            }
        }

        // Same as `to_feature_collection`'s internals, but printed in local
        // (un-reprojected) meters so it lines up 1:1 with the PRE-cut dump
        // above -- lets us see exactly which vertices the cut introduced.
        let mut polygons = zones
            .iter()
            .map(|zone| zone_polygon(zone, &lanes, &successors, MIN_DRAWN_LANE_LENGTH_METERS))
            .collect::<Result<Vec<_>>>()
            .unwrap();
        let stop_points = zones
            .iter()
            .map(|zone| stop_line_point(zone, &lanes).map(|p| Coord { x: p.x, y: p.y }))
            .collect::<Result<Vec<_>>>()
            .unwrap();
        resolve_overlaps(&zones, &lanes, &successors, &stop_points, &mut polygons).unwrap();
        for id in ["50926861#0_straight", "50926859#0_straight"] {
            let idx = zones.iter().position(|z| z.id.0 == id).unwrap();
            println!("=== {id} POST-cut (local meters) ===");
            for part in &polygons[idx].0 {
                println!("  ring ({} pts): {:?}", part.exterior().0.len(), part.exterior().0);
            }
        }
    }

    /// A closed rectangle polygon, corners in `(x, y)` — built directly as a
    /// [`MultiPolygon`] rather than a [`Position`] ring, since production
    /// code now carries geometry that way throughout (see the module docs)
    /// and only ever converts to `Position`s at the very end, in
    /// [`build_feature`].
    fn rect(min: (f64, f64), max: (f64, f64)) -> MultiPolygon<f64> {
        MultiPolygon::new(vec![GeoPolygon::new(
            LineString::from(vec![
                (min.0, min.1),
                (max.0, min.1),
                (max.0, max.1),
                (min.0, max.1),
                (min.0, min.1),
            ]),
            Vec::new(),
        )])
    }

    #[test]
    fn subtracting_the_real_overlap_removes_only_the_locally_contested_ground() {
        // A long, straight zone whose real conflict is entirely at its far
        // end (x in [95, 100]) -- the same shape of problem as a real
        // Barcelona bug this guards against (an 80.9m ribbon contested by a
        // neighbour clustered around one small bend near its own far end).
        // A half-plane cut aimed by the two polygons' own overall centroids
        // (an earlier version of `resolve_overlaps` worked this way) could
        // remove most of `a`, far past the real, local overlap; subtracting
        // `a.intersection(b)` directly can only ever remove exactly the
        // disputed ground, wherever it actually is.
        let a = rect((0.0, 0.0), (100.0, 10.0));
        let b = rect((95.0, 3.0), (105.0, 7.0));

        let remaining = a.difference(&a.intersection(&b));
        assert!(
            remaining.unsigned_area() > 900.0,
            "expected to keep almost all of a's own 1000 units\u{b2}, losing only the small \
             overlap with b near its far end -- got {:.1}",
            remaining.unsigned_area()
        );
        assert_eq!(
            remaining.intersection(&b).unsigned_area(),
            0.0,
            "a and b must not overlap any more after a gives back their real overlap"
        );
    }

    #[test]
    fn a_long_bent_ribbon_keeps_most_of_its_own_area_when_only_one_end_is_contested() {
        // The real Barcelona ribbon this bug was found on: `-27641458#20_1`
        // is an 80.9m lane with one gentle bend, contested near that bend
        // by three separate neighbouring zones at once. A hand-rolled
        // clipper that anchored on the two rings' own centroids instead of
        // the real overlap, and that couldn't safely apply more than one
        // cutting constraint at once against a non-convex base, used to
        // collapse this to a ~53m² disconnected notch nowhere near any of
        // the three real overlaps; `bisecting_cut` (anchored on the real
        // overlap) and `geo::BooleanOps::difference` (exact regardless of
        // convexity or how many cuts are applied) should instead keep the
        // great majority of the ribbon's own ~260m² and lose only a small
        // piece near the bend.
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

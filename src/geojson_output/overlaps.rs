use anyhow::Result;
use anstream::eprintln;
use anstyle::{AnsiColor, Style};
use geo::algorithm::area::Area;
use geo::{
    BooleanOps, BoundingRect, Contains, Coord, LineString, MapCoords, MultiPolygon, Point as GeoPoint,
    Polygon as GeoPolygon,
};
use geojson::{Feature, FeatureCollection, Position};
use std::collections::{HashMap, HashSet};
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::Lane;
use crate::geojson_output::geometry::{
    pedestrian_lane_polygon, rectangle_if_close, zone_polygon, LaneLinks, SPIKE_WELD_EPSILON_METERS,
};
use crate::geojson_output::reprojection::MIN_DRAWN_LANE_LENGTH_METERS;

pub const ERROR: Style = AnsiColor::Red.on_default().bold();

/// The exterior ring of `feature`'s own geometry — `geojson_output` only
/// ever emits a `Polygon` (one exterior, holes aside), so there is exactly
/// one to return; kept as a `Vec` (0 or 1 element, never more) rather than
/// `Option<&[Position]>` so callers built around "one ring per polygon
/// part" (`feature_multipolygon` among them) don't need their own shape
/// change now that there is only ever one part.
pub fn feature_rings(feature: &Feature) -> Vec<&[Position]> {
    match feature.geometry.as_ref().map(|g| &g.value) {
        // A vehicle zone (or a pedestrian zone with a single bank) is one
        // exterior ring.
        Some(geojson::GeometryValue::Polygon { coordinates }) => {
            coordinates.first().map(Vec::as_slice).into_iter().collect()
        }
        // A pedestrian zone spans both banks of its crossing, one exterior
        // ring per bank.
        Some(geojson::GeometryValue::MultiPolygon { coordinates }) => coordinates
            .iter()
            .filter_map(|polygon| polygon.first().map(Vec::as_slice))
            .collect(),
        _ => Vec::new(),
    }
}

pub fn feature_multipolygon(feature: &Feature) -> MultiPolygon<f64> {
    let rings = feature_rings(feature).into_iter().map(|ring| {
        GeoPolygon::new(LineString::new(ring.iter().map(|p| Coord { x: p[0], y: p[1] }).collect()), Vec::new())
    });
    MultiPolygon::new(rings.collect())
}


pub fn overlapping_zone_ids(collection: &FeatureCollection) -> Vec<(String, String)> {
    overlapping_zone_ids_larger_than(collection, OVERLAP_AREA_THRESHOLD_M2)
}

/// [`overlapping_zone_ids`], reporting only pairs that overlap by more
/// than `min_area_m2` — for a caller that cares about an overlap a client
/// could actually be positioned inside, rather than about the last
/// square millimetre [`resolve_overlaps`] is still aiming to remove.
pub fn overlapping_zone_ids_larger_than(
    collection: &FeatureCollection,
    min_area_m2: f64,
) -> Vec<(String, String)> {
    let threshold_deg2 = min_area_m2 / (84_000.0 * 111_000.0);
    let ids: Vec<Option<String>> = collection
        .features
        .iter()
        .map(|feature| feature.property("waiting_zone_id").and_then(|v| v.as_str()).map(str::to_string))
        .collect();
    let polygons: Vec<MultiPolygon<f64>> = collection.features.iter().map(feature_multipolygon).collect();

    let mut found = Vec::new();
    for i in 0..polygons.len() {
        for j in (i + 1)..polygons.len() {
            if polygons[i].intersection(&polygons[j]).unsigned_area() > threshold_deg2
                && let (Some(a), Some(b)) = (&ids[i], &ids[j])
            {
                found.push((a.clone(), b.clone()));
            }
        }
    }
    found
}

// Treat tiny positive-area crossings as real overlaps too. They are commonly
// produced where two buffered lane chains share a connector vertex; leaving
// them below the old threshold still makes their perimeter lines visibly
// cross in the client, even though the measured area is only a few mm².
pub const OVERLAP_AREA_THRESHOLD_M2: f64 = 0.000001;

pub fn overlap_area_m2(a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> f64 {
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

/// Where segments `a1`->`a2` and `b1`->`b2` properly cross, if they do —
/// `None` for parallel or non-crossing segments. Standard parametric
/// line intersection: solve `a1 + t*(a2-a1) == b1 + s*(b2-b1)` for
/// `t, s`, both required in `[0, 1]` for the crossing to be a real point
/// on both segments rather than on their infinite extensions.
fn segment_intersection(a1: Coord<f64>, a2: Coord<f64>, b1: Coord<f64>, b2: Coord<f64>) -> Option<Coord<f64>> {
    let (adx, ady) = (a2.x - a1.x, a2.y - a1.y);
    let (bdx, bdy) = (b2.x - b1.x, b2.y - b1.y);
    let denom = adx * bdy - ady * bdx;
    if denom == 0.0 {
        return None;
    }
    let (ex, ey) = (b1.x - a1.x, b1.y - a1.y);
    let t = (ex * bdy - ey * bdx) / denom;
    let s = (ex * ady - ey * adx) / denom;
    if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&s) {
        Some(Coord { x: a1.x + t * adx, y: a1.y + t * ady })
    } else {
        None
    }
}

/// How close a computed crossing point has to land to an existing vertex
/// already in `ring`, in metres, to snap onto that vertex's own exact
/// coordinate instead of keeping its own separately-computed one — see
/// [`split_self_intersection`]'s own docs for why: confirmed on real
/// Barcelona data (`207322888#0_straight`), a computed crossing point can
/// agree with an existing vertex to 9 significant digits (nanometres
/// apart) and *still* not be bit-identical to it, which is close enough
/// that the split is simple in this crate's own local, metre-scale
/// coordinates but not with any real margin — reprojecting to lon/lat
/// (an entirely different `f64` computation over the same near-coincident
/// points) can and does flip that residual nanometre-scale gap into a
/// real crossing again. Nanometres is noise by any physical measure a
/// waiting zone cares about; snapping onto the vertex it already agrees
/// with removes the make-believe precision (and the two points to
/// disagree about) rather than trying to preserve it through a
/// computation that was never going to end up meaning anything anyway.
pub const SPLIT_SNAP_MARGIN_METERS: f64 = 0.01;

/// Snaps `p` onto the closest vertex in `coords` if one lands within
/// [`SPLIT_SNAP_MARGIN_METERS`] — see that constant's own docs for why.
/// Returns `p` unchanged if nothing is that close.
fn snap_to_nearby_vertex(p: Coord<f64>, coords: &[Coord<f64>]) -> Coord<f64> {
    coords
        .iter()
        .copied()
        .min_by(|a, b| {
            let da = (a.x - p.x).hypot(a.y - p.y);
            let db = (b.x - p.x).hypot(b.y - p.y);
            da.total_cmp(&db)
        })
        .filter(|v| (v.x - p.x).hypot(v.y - p.y) < SPLIT_SNAP_MARGIN_METERS)
        .unwrap_or(p)
}

/// Perpendicular distance from `p` to segment `a`->`b`, and the closest
/// point on it — clamped to the segment's own `[0, 1]` extent, not its
/// infinite line: a point past the segment's own end has to report
/// distance to that end, not to a point the segment doesn't actually
/// reach.
fn point_segment_distance(p: Coord<f64>, a: Coord<f64>, b: Coord<f64>) -> (f64, Coord<f64>) {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len_sq = dx * dx + dy * dy;
    if len_sq == 0.0 {
        return ((p.x - a.x).hypot(p.y - a.y), a);
    }
    let t = (((p.x - a.x) * dx + (p.y - a.y) * dy) / len_sq).clamp(0.0, 1.0);
    let closest = Coord { x: a.x + t * dx, y: a.y + t * dy };
    ((p.x - closest.x).hypot(p.y - closest.y), closest)
}

/// If some vertex of `ring` sits within [`SPLIT_SNAP_MARGIN_METERS`] of a
/// *non-adjacent* edge's own segment without properly crossing it, returns
/// that vertex's own ring index, the near edge's own start index, and the
/// exact point both should be welded onto (that edge's own closest point
/// to the vertex — see [`point_segment_distance`]). This is a "near
/// T-touch": a vertex grazing a distant edge rather than two edges
/// crossing transversally, which [`split_self_intersection`]'s own strict
/// sign-flip test structurally cannot see (there's no sign to flip — the
/// vertex sits *beside* the edge, not through it). Confirmed on real
/// Barcelona data (`207322888#0_straight`): a vertex left within
/// nanometres of a real, original edge from *before* any cut ever ran,
/// close enough that the ring reads as simple in this crate's own local
/// coordinates (right at the edge of `f64` precision — the sign of the
/// underlying cross product there differs between this crate's own
/// release-mode arithmetic and the same formula evaluated in plain
/// Python on identical literals) but not with any real margin:
/// reprojecting to lon/lat (an entirely different set of `f64`
/// operations over the same near-coincident points) flips that residual
/// nanometre-scale gap into a real crossing again.
///
/// Excludes not just `i`/`i2` themselves but their own immediate
/// neighbours too (`i`'s predecessor, `i2`'s successor) — confirmed on
/// real Barcelona data (`6119951203_w0_straight_ped`): welding a vertex
/// that's already ring-adjacent to one of the edge's own endpoints (not
/// the endpoint itself, but sitting right next to it) overwrites that
/// vertex to the *same* coordinate as its own immediate neighbour,
/// leaving a genuine zero-length edge — a self-intersection this
/// function would be introducing, not fixing. A vertex that close to an
/// edge it's already adjacent to is exactly what `despike`'s own
/// weld-and-remove-spike pass (for a vehicle zone) or nothing at all (for
/// a pedestrian zone, which never runs `despike`) is the right tool for,
/// not this one.
pub fn find_near_touch(coords: &[Coord<f64>]) -> Option<(usize, usize, Coord<f64>)> {
    let n = coords.len();
    for k in 0..n {
        for i in 0..n {
            let i2 = (i + 1) % n;
            let before_i = (i + n - 1) % n;
            let after_i2 = (i2 + 1) % n;
            if k == i || k == i2 || k == before_i || k == after_i2 {
                continue; // shares, or is one ring-step from, an endpoint of this edge already
            }
            let (distance, closest) = point_segment_distance(coords[k], coords[i], coords[i2]);
            if distance < SPLIT_SNAP_MARGIN_METERS {
                return Some((k, i, closest));
            }
        }
    }
    None
}

/// How far `point` sits from `polygon` — `0.0` anywhere inside it or on
/// its boundary, and the distance to the nearest boundary point
/// otherwise. Used to pick which of a zone's candidate stop lines to
/// publish (see `feature::build_feature`) and, in the tests, to check
/// that the published one lands on the zone it belongs to.
pub fn distance_to_polygon(point: Coord<f64>, polygon: &MultiPolygon<f64>) -> f64 {
    if polygon.contains(&GeoPoint::from(point)) {
        return 0.0;
    }
    polygon
        .0
        .iter()
        .flat_map(|part| std::iter::once(part.exterior()).chain(part.interiors()))
        .flat_map(|ring| {
            let coords: Vec<Coord<f64>> = ring.coords().copied().collect();
            (0..coords.len().saturating_sub(1))
                .map(|i| point_segment_distance(point, coords[i], coords[i + 1]).0)
                .collect::<Vec<_>>()
        })
        .fold(f64::INFINITY, f64::min)
}

/// Removes every vertex that grazes an edge it is *ring-adjacent* to —
/// the one shape of near-degeneracy [`find_near_touch`] deliberately
/// refuses to look at (see its own docs on why welding one of these
/// creates a genuine zero-length edge rather than fixing anything), and,
/// measured across the whole real Barcelona network, the sole remaining
/// cause of a self-intersecting output ring: every one of the 20 rings
/// that still crossed itself had a vertex within 0.03mm of an edge two
/// ring-steps away, while their nearest *non*-adjacent approach was
/// never closer than 16mm — comfortably outside
/// [`SPLIT_SNAP_MARGIN_METERS`], so `find_near_touch` correctly reported
/// nothing and nothing else in the pipeline was looking.
///
/// Adjacency is what makes dropping the vertex the right move rather
/// than splitting: `k`, `i` and `i+1` are consecutive around the ring, so
/// `k` is a local spike tip or a near-collinear point on its own
/// neighbours' span, not a waist pinching two real lobes together. The
/// enclosed area it accounts for is bounded by its own distance to that
/// span times the span's length — sub-square-millimetre at these
/// distances — so removing it is a no-op on the zone's real ground and
/// removes the coincidence outright.
///
/// Removing the coincidence, rather than welding it into an exactly
/// shared point, is the part that actually matters. These rings are
/// *simple* in this crate's own local metre coordinates; they only cross
/// once [`crate::geojson_output::reprojection::Reprojector`] has
/// re-derived every vertex through an entirely different `f64`
/// computation, which is free to move two points that agree to 12
/// significant digits onto opposite sides of each other. A weld would
/// preserve exactly the coincidence that reprojection gets to break.
///
/// Runs to a fixed point (removing one vertex can expose another behind
/// it), bounded by the ring's own shrinking length. Returns `None` if the
/// ring can't survive as a triangle.
fn drop_grazing_vertices(ring: &LineString<f64>) -> Option<LineString<f64>> {
    let mut coords: Vec<Coord<f64>> = ring.coords().copied().collect();
    if coords.first() == coords.last() {
        coords.pop(); // work in distinct-point indices; re-closed on the way out
    }

    loop {
        let n = coords.len();
        if n < 3 {
            return None;
        }
        let grazing = (0..n).find(|&k| {
            // The two edges `k` is ring-adjacent to without being an
            // endpoint of: the one before its predecessor, and the one
            // after its successor.
            [(k + n - 2) % n, (k + 1) % n].into_iter().any(|i| {
                let (distance, _) = point_segment_distance(coords[k], coords[i], coords[(i + 1) % n]);
                distance < SPLIT_SNAP_MARGIN_METERS
            })
        });
        let Some(k) = grazing else { break };
        coords.remove(k);
    }

    if coords.len() < 3 {
        return None;
    }
    coords.push(coords[0]);
    Some(LineString::new(coords))
}

/// How extreme an interior angle has to be, in degrees away from a
/// straight boundary's own 180°, before the vertex carrying it is a
/// needle rather than a corner — see [`drop_needle_vertices`].
///
/// There is no gap in the real distribution to find: measured over
/// Barcelona's own 5861 output vertices, interior angles run smoothly
/// from 0.19° upward, and the band from roughly 60° to 90° is densely
/// populated by *legitimate* corners — a stop line meeting a lane edge
/// at anything but a right angle, which is most of them on a real street
/// grid. So this deliberately names only the unambiguous end of that
/// range. A round join's own segments (`ROUND_JOIN_SEGMENT_ANGLE_RADIANS`,
/// ~17° of turn each) sit near 163°, nowhere near it; a genuine skewed
/// stop-line corner sits near 75°, also nowhere near it; a needle sits
/// under 5°, where the two edges meeting at the vertex are so nearly
/// anti-parallel that the boundary runs out and straight back, enclosing
/// no meaningful ground on the way.
pub const MIN_INTERIOR_ANGLE_DEGREES: f64 = 5.0;

/// The signed interior angle at `b`, in degrees, for a ring wound
/// counterclockwise — `180°` on a dead-straight boundary, less than that
/// at a convex corner, more at a reflex one. Signed rather than the
/// angle *between* two rays (which folds into `[0°, 180°]` and so reports
/// a 359° reflex needle as an innocuous-looking 1°), so both ends of the
/// range are distinguishable and checkable on their own.
fn interior_angle_degrees(a: Coord<f64>, b: Coord<f64>, c: Coord<f64>, counterclockwise: bool) -> f64 {
    let (v1x, v1y) = (a.x - b.x, a.y - b.y);
    let (v2x, v2y) = (c.x - b.x, c.y - b.y);
    let angle = (v1x * v2y - v1y * v2x).atan2(v1x * v2x + v1y * v2y).to_degrees();
    let angle = if counterclockwise { -angle } else { angle };
    if angle < 0.0 { angle + 360.0 } else { angle }
}

/// Removes every needle: a vertex whose two edges are so nearly
/// anti-parallel ([`MIN_INTERIOR_ANGLE_DEGREES`]) that the boundary runs
/// out to it and straight back rather than turning at it.
///
/// This is [`crate::geojson_output::geometry::despike`]'s own concern
/// generalised past the one case it can see. That pass asks whether a
/// vertex's two *neighbours* land within 15cm of each other, which
/// catches a short spike and structurally cannot catch a long thin one —
/// a needle 2m deep has its neighbours 2m apart and sails straight
/// through. Asking about the angle instead catches both, and asks the
/// question the defect is actually about.
///
/// Removing the tip is a no-op on the zone's real ground by construction:
/// at under 5° the triangle the vertex forms with its own neighbours is
/// degenerate, so the area it accounts for is a rounding error against
/// any zone a client cares about. Runs to a fixed point, since removing
/// one tip can leave its neighbour a needle in turn.
///
/// Can, like any local vertex removal, leave the *rest* of the ring
/// crossing itself — the caller runs
/// [`split_self_intersections`] afterward for exactly that reason (see
/// `to_feature_collection`'s own final pass).
fn drop_needle_vertices(ring: &LineString<f64>) -> Option<LineString<f64>> {
    let mut coords: Vec<Coord<f64>> = ring.coords().copied().collect();
    if coords.first() == coords.last() {
        coords.pop();
    }

    loop {
        let n = coords.len();
        if n < 3 {
            return None;
        }
        let twice_area: f64 =
            (0..n).map(|i| coords[i].x * coords[(i + 1) % n].y - coords[(i + 1) % n].x * coords[i].y).sum();
        let counterclockwise = twice_area > 0.0;
        let needle = (0..n).find(|&i| {
            let angle =
                interior_angle_degrees(coords[(i + n - 1) % n], coords[i], coords[(i + 1) % n], counterclockwise);
            !(MIN_INTERIOR_ANGLE_DEGREES..=360.0 - MIN_INTERIOR_ANGLE_DEGREES).contains(&angle)
        });
        let Some(i) = needle else { break };
        coords.remove(i);
    }

    if coords.len() < 3 {
        return None;
    }
    coords.push(coords[0]);
    Some(LineString::new(coords))
}

/// Moves every convex needle vertex to the area-preserving position that
/// opens its interior angle as wide as possible, instead of deleting it —
/// the counterpart to [`drop_needle_vertices`] for a spike whose own
/// triangle encloses real ground.
///
/// A needle's own triangle is long and thin: its tip `b` sits far from the
/// chord `a`->`c` joining its neighbours, so `a` and `c` lie in nearly the
/// same direction from `b` and the interior angle there collapses toward
/// `0°`. Deleting `b` gives up whatever ground that triangle encloses, which
/// is exactly what makes [`drop_needle_vertices`] need its budget and why
/// that budget can leave a long thin spike in place. Moving `b` does not
/// have to give anything up: the set of points preserving the triangle's
/// signed area is the line parallel to `a`->`c` through `b`, and along it
/// the interior angle at `b` is widest when `b` sits symmetrically over the
/// chord's own midpoint. So the tip is relocated there — the zone's exact
/// area is kept, and the spike becomes a shallow, ordinary corner. A tip
/// already at (or within a floating-point hair of) that spot has nothing to
/// gain and is left alone.
///
/// Only convex needles (a small interior angle — a spike of material) are
/// touched. A reflex one (near `360°`, a slit cut *into* the material) is
/// left to [`drop_needle_vertices`]: symmising a slit would push ground
/// outward into whatever the slit was separating this zone from, which is a
/// worse defect than the one it would be cleaning up.
///
/// Area preservation makes this safe to run without
/// [`MAX_CLEANUP_AREA_DRIFT_M2`]'s own guard: the shoelace contribution of
/// `a`,`b`,`c` and of `a`,`b'`,`c` are identical by construction, so the
/// ring's area is unchanged however many tips move. A move *can* still
/// leave the ring crossing itself (the tip lands somewhere new), which is
/// why the caller runs [`split_self_intersections`] afterwards — the same
/// reason [`drop_needle_vertices`]'s own removals do.
fn relax_needle_vertices(ring: &LineString<f64>) -> Option<LineString<f64>> {
    let mut coords: Vec<Coord<f64>> = ring.coords().copied().collect();
    if coords.first() == coords.last() {
        coords.pop();
    }

    // Each move is judged against the ring's *current* neighbours, so a
    // fixed-point walk is what actually converges; bounded by the ring's own
    // length so a pathological pair of tips can't ping-pong forever.
    for _ in 0..coords.len() {
        let n = coords.len();
        if n < 3 {
            return None;
        }
        let twice_area: f64 =
            (0..n).map(|i| coords[i].x * coords[(i + 1) % n].y - coords[(i + 1) % n].x * coords[i].y).sum();
        let counterclockwise = twice_area > 0.0;

        let mut relaxed = None;
        for i in 0..n {
            let (a, b, c) = (coords[(i + n - 1) % n], coords[i], coords[(i + 1) % n]);
            // Convex needle only (`angle < MIN`) -- see this function's own docs.
            if interior_angle_degrees(a, b, c, counterclockwise) >= MIN_INTERIOR_ANGLE_DEGREES {
                continue;
            }
            let (dx, dy) = (c.x - a.x, c.y - a.y);
            let chord = dx.hypot(dy);
            if chord < SPIKE_WELD_EPSILON_METERS {
                continue; // neighbours already coincide; removal is the only real move
            }
            let signed_height = (dx * (b.y - a.y) - dy * (b.x - a.x)) / chord;
            let (nx, ny) = (-dy / chord, dx / chord);
            let moved = Coord {
                x: (a.x + c.x) / 2.0 + nx * signed_height,
                y: (a.y + c.y) / 2.0 + ny * signed_height,
            };
            if (moved.x - b.x).hypot(moved.y - b.y) > 1e-9 {
                relaxed = Some((i, moved));
                break;
            }
        }

        match relaxed {
            Some((i, moved)) => coords[i] = moved,
            None => break,
        }
    }

    if coords.len() < 3 {
        return None;
    }
    coords.push(coords[0]);
    Some(LineString::new(coords))
}

/// [`relax_needle_vertices`], applied to every ring of `part`.
fn relax_needles_in(part: GeoPolygon<f64>) -> GeoPolygon<f64> {
    let Some(exterior) = relax_needle_vertices(part.exterior()) else { return part };
    let interiors: Vec<LineString<f64>> =
        part.interiors().iter().filter_map(relax_needle_vertices).collect();
    GeoPolygon::new(exterior, interiors)
}

/// The unsigned area enclosed by an open (first point not repeated)
/// coordinate sequence — the shoelace formula. What
/// [`drop_negligible_vertices`] measures a ring's own *starting* area
/// with, once, before its own removal loop begins (see that function's
/// own docs on why the threshold is a fraction of a fixed reference
/// rather than of a shrinking one).
fn polygon_area(coords: &[Coord<f64>]) -> f64 {
    let n = coords.len();
    (0..n).map(|i| coords[i].x * coords[(i + 1) % n].y - coords[(i + 1) % n].x * coords[i].y).sum::<f64>().abs() / 2.0
}

/// The unsigned area of the triangle formed by three points — exactly
/// how much a ring's own area changes if `b` is removed and `a`/`c`
/// become directly adjacent, which is the quantity
/// [`drop_negligible_vertices`] actually needs, not the angle at `b`.
///
/// The distinction matters: an interior angle near 180° (what "removing
/// this vertex barely changes anything" naturally suggests checking) is
/// a *local* measure that individually looks negligible on both a
/// harmless duplicate point *and* on one link of a long, gently curving
/// chain — sin(180° - θ) → 0 either way, independent of how long the two
/// edges are. Triangle area is too, taken on its own
/// (`0.5 * |ab| * |bc| * sin(interior angle)`) — but
/// [`drop_negligible_vertices`] recomputes it fresh against each
/// vertex's *current* neighbours after every removal, which a raw angle
/// check invites skipping (angles don't visibly accumulate the way area
/// does). See that function's own docs for the real Barcelona case this
/// distinction was found on.
fn triangle_area(a: Coord<f64>, b: Coord<f64>, c: Coord<f64>) -> f64 {
    ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs() / 2.0
}

/// How much of a ring's own *starting* area a single vertex's own
/// removal may account for before it's kept rather than dropped — see
/// [`drop_negligible_vertices`].
///
/// A *fraction* of the ring's own area, not an absolute one, because the
/// defect this targets scales with the zone: a lane's own real-world
/// `.net.xml` shape is essentially never perfectly straight (netconvert
/// digitisation noise, or two separately-buffered pieces — an
/// ancestor-extension chain and its own core gate — landing their own
/// copy of what should be one shared corner a few centimetres to ~20cm
/// apart), and stroke-buffering a small kink like that over tens of
/// metres of lane produces a stray vertex whose own *absolute* area can
/// be a good fraction of a square metre on an ordinary street-width
/// zone — confirmed on real Barcelona data, a rectangle that should have
/// exactly 4 corners
/// (`-27489988#4_straight`, `-207322888#12_straight+right`,
/// `-27490015#2_straight`, `-27490015#2_turn+left`,
/// `-665948577#3_straight`, `-673862127#2_straight`,
/// `166024474#3_straight`, `-558212802#2_straight`) instead carrying one
/// extra vertex worth 0.006m²–0.42m². An absolute cap tight enough to
/// leave the two known-sensitive pedestrian fixtures below untouched
/// (their own smallest real corners are 0.044m²/0.221m² in *absolute*
/// terms) would have to sit low enough to leave every one of those
/// eight rectangles' own stray vertex (up to 0.42m²) in place too — the
/// two requirements are incompatible in absolute terms, because a
/// 240m²–800m² street-width zone and a 12m²–34m² walkingarea patch
/// simply operate at different absolute scales.
///
/// Relative to each ring's own area, they don't: every one of those
/// eight stray vertices accounts for between 0.0024% and 0.1017% of its
/// own zone, while the two sensitive fixtures' own smallest real corners
/// are 0.3496% and 0.6542% — a clean 3.4x gap between the worst bug and
/// the best real corner, confirmed by sweeping every ring in the real
/// Barcelona network at this exact cap and finding nothing else closer
/// to it than that gap allows.
const NEGLIGIBLE_VERTEX_AREA_FRACTION: f64 = 0.0015;

/// Removes every vertex whose own removal — against its *current*
/// immediate neighbours, recomputed fresh after every prior removal —
/// would change the ring's area by less than
/// [`NEGLIGIBLE_VERTEX_AREA_FRACTION`] of the ring's own *starting*
/// area (computed once, before the loop below, so the threshold itself
/// doesn't drift as vertices come out — see [`NEGLIGIBLE_VERTEX_AREA_FRACTION`]'s
/// own docs for why a fraction, not an absolute area, is what actually
/// separates a real corner from a stray one here).
///
/// This exists because the obvious-looking alternative is unsound: asking
/// "is this vertex's interior angle close to 180°" and removing every one
/// that qualifies, all at once, against their *original* neighbours.
/// Individually each such vertex looks harmless — confirmed on real
/// Barcelona data, `171839324#6_straight` (a 440m² vehicle zone) has 7
/// vertices within a fraction of a degree of 180°, each accounting for
/// under 0.07% of the zone's own area on its own. But that zone's own
/// shape isn't a straight line there: it's a real, gentle ~0.5m bow over
/// 138m (this crate draws a lane's own surveyed shape, not an idealised
/// straight approximation of the road), and stripping all 7 in one pass
/// — the natural reading of the angle test — shifted the finished
/// polygon by 36m², 8.2% of its own area: a real, visible defect, not a
/// cleanup. Each individual step looked free; the chain of them wasn't,
/// because removing one vertex moves its neighbours' own effective gap
/// wider for whichever removal is judged next.
///
/// The fix is the fixed-point loop below, not a smaller-looking
/// threshold: after each removal, every remaining vertex's own triangle
/// area is recomputed against its *new* current neighbours before the
/// next decision. A vertex partway along a real curve fails this test as
/// soon as its neighbours have widened enough to reveal the curve's own
/// sag — which is exactly why, on that same 138m bow, this correctly
/// keeps stopping after removing only the segment's own sub-millimetre
/// noise (confirmed: the fraction-of-original-area test below removes
/// nothing more from it, leaving the real bend untouched) rather than
/// continuing on to the real bend.
///
/// Recomputing isn't quite enough on its own, though: `spent` tracks how
/// much of the ring's own budget every removal *so far* has already
/// used, and a candidate is only taken if what's left can still afford
/// it. Without that shared budget, one real, legitimate removal can
/// silently *unlock* a second, illegitimate one purely by coincidence —
/// confirmed on real Barcelona data, `-27641458#2_straight`: three
/// vertices sit in a tight cluster right at the zone's own stop line,
/// one genuinely negligible (0.089% of the zone) and one a real corner
/// that keeps the polygon reaching the stop line at all (0.86% measured
/// against its own original neighbours — safely above the cap on its
/// own). Removing the first, negligible one moves the real corner's
/// *own* neighbour further away; measured fresh against that new,
/// farther neighbour, its own triangle happened to fall to 0.146% —
/// under the cap too, by coincidence of exactly where these three
/// points happen to sit, not because it stopped being a real corner.
/// Unbudgeted, the loop removed it anyway, and the finished zone no
/// longer reached its own stop line by 1.13m. A per-ring budget catches
/// this the same way it would a longer chain: the first removal alone
/// (0.089%) fits; adding the second (0.089% + 0.146% = 0.235%) doesn't,
/// so the loop keeps the first and correctly leaves the real corner in
/// place.
///
/// The *smallest* qualifying vertex is removed each time, not the first
/// one ring order happens to reach — confirmed necessary, not just
/// tidier, on `-27489988#4_straight` (the very rectangle-with-a-stray-
/// vertex bug this whole pass exists for): its own genuinely spurious
/// vertex (0.077% of the zone) sits at a *later* ring index than a real
/// corner (0.319%, comfortably under the cap on its own too, purely
/// because this zone is large enough that even a real corner's own
/// triangle reads small as a fraction of it). Scanning in ring order and
/// taking whichever qualifies first would remove that real corner
/// first, simply because it happens to come first in the array — not
/// because it's less real than the vertex after it. Taking the smallest
/// first instead removes the genuinely spurious one, and only then
/// re-measures the real corner against its own new, farther neighbour —
/// where it lands at 120m² and is correctly kept, exactly as
/// [`triangle_area`]'s own docs on recomputing-against-current-
/// neighbours already promise, just with the two candidates visited in
/// the order that actually matters.
///
/// Applies to every zone — vehicle zones already run Douglas-Peucker
/// (`geometry::zone_polygon`'s own `simplify`, vehicle-only), which
/// removes much of what this would from an ordinary noisy chain, but not
/// all of it: DP's own chord-based criterion judges a candidate
/// segment's *cumulative* deviation from the chord spanning it, which
/// for the eight-rectangle bug this fraction was calibrated against
/// (see [`NEGLIGIBLE_VERTEX_AREA_FRACTION`]'s own docs) missed a stray
/// vertex sitting only 2mm off the very chord DP itself would have
/// measured it against — confirmed by instrumenting `zone_polygon`
/// directly on `-27489988#4_straight`: the vertex was already there
/// immediately after `simplify()` ran, at 5cm tolerance, thirty times
/// looser than that 2mm deviation. This pass is the only cleanup of its
/// kind pedestrian zones get at all, since DP is skipped for them
/// entirely (`geometry::zone_polygon`'s own docs on the two fixtures a
/// blanket DP pass broke there once).
fn drop_negligible_vertices(ring: &LineString<f64>) -> Option<LineString<f64>> {
    let mut coords: Vec<Coord<f64>> = ring.coords().copied().collect();
    if coords.first() == coords.last() {
        coords.pop();
    }
    let ring_area = polygon_area(&coords);
    let budget = ring_area * NEGLIGIBLE_VERTEX_AREA_FRACTION;
    // How much of `budget` every removal so far has already spent — see
    // this function's own docs on why a single ring's *whole* cleanup
    // has to share one budget rather than judging each step only against
    // the untouched, full-size `budget` on its own.
    let mut spent = 0.0;

    loop {
        let n = coords.len();
        if n < 4 {
            break;
        }
        // The *smallest* candidate, not the first one found in ring
        // order — see this function's own docs for the real case
        // (`-27489988#4_straight`) where that distinction is the whole
        // fix: a genuinely spurious vertex and a real corner can *both*
        // read as individually removable, and ring order has no reason
        // to put the spurious one first. Taking the smallest first means
        // a real corner is only ever removed after every genuinely
        // smaller (and so, by construction, less consequential)
        // candidate has already been dealt with — which is exactly when
        // a real corner's own recomputed triangle, against neighbours
        // that have already absorbed every smaller nearby vertex, either
        // still exceeds the budget (kept, correctly) or is small enough
        // that removing it truly doesn't matter either.
        let smallest = (0..n)
            .map(|i| (triangle_area(coords[(i + n - 1) % n], coords[i], coords[(i + 1) % n]), i))
            .min_by(|(a, _), (b, _)| a.total_cmp(b));
        let Some((area, i)) = smallest else { break };
        if area > budget || spent + area > budget {
            break;
        }
        spent += area;
        coords.remove(i);
    }

    if coords.len() < 3 {
        return None;
    }
    coords.push(coords[0]);
    Some(LineString::new(coords))
}

/// How much a part's own area may move, in either direction, before
/// [`drop_grazing_vertices_everywhere`] judges its own three passes to be
/// undoing someone's work rather than removing noise.
///
/// Originally a *growth*-only cap, back when [`drop_grazing_vertices`]
/// and [`drop_needle_vertices`] were the only two passes here. Neither
/// can tell, from local shape alone, the difference between two boundary
/// segments that lie on top of each other because of floating-point
/// noise and two that lie on top of each other because
/// [`resolve_overlaps`] deliberately cut a thin notch out between them —
/// and "cleaning up" the second kind fills the notch back in, handing a
/// zone back exactly the ground it had just been made to give up.
/// Measured on real Barcelona data, that took the vehicle zone pairs
/// overlapping by more than 0.1m² from 2 up to 11 — hence a growth cap,
/// not the passes being restricted by shape: four orders of magnitude
/// above the noise they exist to remove, and an order below the smallest
/// overlap anyone cares about.
///
/// [`drop_negligible_vertices`] then added a third pass that, by design,
/// *shrinks* a ring's real area on purpose (see its own docs on why that
/// still can't reopen an overlap — each of its own steps is bounded far
/// below what any of this would ever notice). The cap is symmetric now
/// only as a second, redundant line of defence should that pass's own
/// per-vertex bookkeeping ever be wrong — not because ordinary,
/// intended shrinkage is expected to come anywhere near it.
const MAX_CLEANUP_AREA_DRIFT_M2: f64 = 0.01;

/// [`drop_grazing_vertices`] and [`drop_needle_vertices`], applied to
/// every ring of `part` — dropping the exterior outright (there's no
/// sensible fallback for it, unlike an interior ring) if it can't survive
/// as a triangle.
///
/// `part` unchanged if this would move its own area past
/// [`MAX_CLEANUP_AREA_DRIFT_M2`] in either direction — see that
/// constant's own docs for what that's guarding against. Split out from
/// [`drop_negligible_vertices`] specifically so the two never share one
/// fallback decision: bundling every pass into a single "is the combined
/// result too different, then throw all of it away" check means a
/// [`drop_negligible_vertices`] shrink that alone is nowhere near either
/// cap can still push a *combined* result over it on a large zone (its
/// own per-vertex cap is absolute, so summed across enough vertices on a
/// zone large enough, even a fully safe cumulative shrink can exceed
/// 0.01m² in the raw) — and a bundled check would then discard this
/// function's own, unrelated fix for a real self-intersection or needle
/// right along with it. Confirmed on real Barcelona data,
/// `395406703#2_straight` (1865.76m²): bundled, [`drop_negligible_vertices`]'s
/// own harmless trim of a few sub-cm vertices was enough to swing the
/// *combined* result past this cap, silently reverting this function's
/// own cleanup back to its raw, undespiked shape too.
fn drop_grazing_and_needle_vertices(part: GeoPolygon<f64>) -> GeoPolygon<f64> {
    let clean = |ring: &LineString<f64>| drop_needle_vertices(&drop_grazing_vertices(ring)?);
    let Some(exterior) = clean(part.exterior()) else { return part };
    let interiors: Vec<LineString<f64>> = part.interiors().iter().filter_map(clean).collect();
    let cleaned = GeoPolygon::new(exterior, interiors);
    if (cleaned.unsigned_area() - part.unsigned_area()).abs() > MAX_CLEANUP_AREA_DRIFT_M2 {
        return part;
    }
    cleaned
}

/// [`drop_negligible_vertices`], applied to every ring of `part` —
/// dropping the exterior outright if it can't survive as a triangle.
///
/// Unconditional, unlike [`drop_grazing_and_needle_vertices`]'s own
/// [`MAX_CLEANUP_AREA_DRIFT_M2`] check: [`drop_negligible_vertices`] is
/// already safe by construction (every removal it makes is individually
/// bounded well below anything a whole-ring drift check would exist to
/// catch — see its own docs), so re-litigating its result against a cap
/// calibrated for a completely different failure mode
/// ([`drop_grazing_and_needle_vertices`]'s own risk of undoing a real
/// overlap cut) only adds a spurious way to reject a perfectly safe
/// cleanup, exactly as it did before this was split out (see that
/// function's own docs on the real zone this broke).
fn drop_negligible_vertices_everywhere_in(part: GeoPolygon<f64>) -> GeoPolygon<f64> {
    let Some(exterior) = drop_negligible_vertices(part.exterior()) else { return part };
    let interiors: Vec<LineString<f64>> =
        part.interiors().iter().filter_map(drop_negligible_vertices).collect();
    GeoPolygon::new(exterior, interiors)
}

/// [`drop_grazing_vertices`], [`drop_needle_vertices`] and
/// [`drop_negligible_vertices`], applied to every ring of every part —
/// see [`drop_grazing_and_needle_vertices`] and
/// [`drop_negligible_vertices_everywhere_in`] for why each runs under its
/// own, separately-scoped safety check rather than one shared decision
/// over all three combined.
pub fn drop_grazing_vertices_everywhere(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(
        polygon
            .0
            .into_iter()
            .map(|part| drop_negligible_vertices_everywhere_in(drop_grazing_and_needle_vertices(part)))
            .collect(),
    )
}

/// [`relax_needles_in`], applied to every part of `polygon` — a standalone
/// pass (rather than folded into [`drop_grazing_vertices_everywhere`])
/// because it is *not* safe to run unconditionally: relocating a tip can
/// move the ground a zone's own published stop line sits on out from under
/// it. The caller keeps the original whenever that happens — see
/// `feature::to_feature_collection`'s own final pass.
pub fn relax_needle_vertices_everywhere(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(polygon.0.into_iter().map(relax_needles_in).collect())
}

/// Fallback for [`split_self_intersection`], for the shape of defect that
/// one can't see at all — see [`find_near_touch`]'s own docs. Welds the
/// touch into a genuine, bit-identical shared point (reusing the near
/// edge's own endpoint outright if the touch already lands within margin
/// of one, rather than inserting a new point a hair away from it — this
/// avoids ever creating a near-zero-length edge right next to a real
/// vertex) and splits `ring` there exactly like `split_self_intersection`
/// splits at a real crossing: cutting a simple loop at the one point it
/// revisits turns it into two loops, each simple on its own. Bit-identical
/// points matter because `Reprojector::to_lon_lat` is a pure function of
/// its own input coordinate — two ring positions holding the *exact same*
/// `Coord<f64>` are guaranteed to reproject to the exact same point,
/// which a "close enough" snap doesn't guarantee at all (that's the whole
/// problem this function exists to fix). Returns `None` if `ring` has no
/// near-touch to weld.
pub fn weld_near_touch_and_split(ring: &LineString<f64>) -> Option<(GeoPolygon<f64>, GeoPolygon<f64>)> {
    let mut coords: Vec<Coord<f64>> = ring.coords().copied().collect();
    if coords.first() == coords.last() {
        coords.pop(); // work in distinct-point indices only; re-close explicitly below
    }
    let n = coords.len();
    if n < 4 {
        return None;
    }
    let (k, i, proj) = find_near_touch(&coords)?;
    let i2 = (i + 1) % n;
    let close = |a: Coord<f64>, b: Coord<f64>| (a.x - b.x).hypot(a.y - b.y) < SPLIT_SNAP_MARGIN_METERS;

    // `touch_index` and `split_index` end up holding the exact same
    // `Coord<f64>` value by construction (either an existing vertex
    // reused outright, or a freshly inserted one that also overwrites
    // the touching vertex's own slot) — so the slice between them below
    // already starts and ends at that shared point on its own; no
    // separate closing push needed, and pushing one anyway would just
    // duplicate whichever end already holds it.
    let (touch_index, split_index) = if close(proj, coords[i]) {
        coords[k] = coords[i];
        (k, i)
    } else if close(proj, coords[i2]) {
        coords[k] = coords[i2];
        (k, i2)
    } else {
        coords.insert(i2, proj);
        let k = if k >= i2 { k + 1 } else { k };
        coords[k] = proj;
        (k, i2)
    };

    let n = coords.len();
    let (lo, hi) = (touch_index.min(split_index), touch_index.max(split_index));

    let inner = LineString::new(coords[lo..=hi].to_vec());

    let mut outer_coords = coords[hi..n].to_vec();
    outer_coords.extend(coords[0..=lo].iter().copied());
    let outer = LineString::new(outer_coords);

    Some((GeoPolygon::new(inner, Vec::new()), GeoPolygon::new(outer, Vec::new())))
}

/// If `ring` crosses itself, splits it at the first crossing found into
/// the two simple sub-rings that share that crossing point as a vertex —
/// the standard fix for a "figure-8": cutting a simple loop at the one
/// point it revisits turns it into two loops, each simple on its own.
/// `resolve_overlaps`' own `difference` can produce exactly this
/// (confirmed on real Barcelona data, `207322888#0_straight`): a cut
/// close enough to the far side of the polygon being cut can leave the
/// cut boundary revisit ground already behind it in the same trip around
/// the ring, rather than staying a simple loop — the two zones' own
/// share of that cut ends up as one self-crossing ring instead of two
/// disjoint ones, so `keep_part_near`'s own "more than one part" check
/// (`polygon.0.len() <= 1`) never even sees a second part to choose
/// between. Returns `None` if `ring` doesn't self-intersect at all —
/// this is meant to run unconditionally on every cut's own result, most
/// of which never need it.
fn split_self_intersection(ring: &LineString<f64>) -> Option<(GeoPolygon<f64>, GeoPolygon<f64>)> {
    let coords: Vec<Coord<f64>> = ring.coords().copied().collect();
    let n = coords.len().saturating_sub(1); // last position repeats the first
    if n < 4 {
        return None;
    }
    for i in 0..n {
        let (a1, a2) = (coords[i], coords[(i + 1) % n]);
        for j in (i + 2)..n {
            if i == 0 && j == n - 1 {
                continue; // adjacent via the closing wrap-around
            }
            let (b1, b2) = (coords[j], coords[(j + 1) % n]);
            let Some(p) = segment_intersection(a1, a2, b1, b2) else { continue };
            let p = snap_to_nearby_vertex(p, &coords[..n]);

            let mut inner = vec![p];
            inner.extend(coords[(i + 1)..=j].iter().copied());
            inner.push(p);

            let mut outer = vec![p];
            outer.extend(coords[(j + 1)..n].iter().copied());
            outer.extend(coords[0..=i].iter().copied());
            outer.push(p);

            return Some((
                GeoPolygon::new(LineString::new(inner), Vec::new()),
                GeoPolygon::new(LineString::new(outer), Vec::new()),
            ));
        }
    }
    None
}

/// [`split_self_intersection`], applied to every part of `polygon` that
/// needs it, and to each half again in turn — a single cut can leave a
/// ring crossing itself more than once, and each split's own two halves
/// are new rings in their own right, not guaranteed simple just because
/// they're smaller. Falls back to [`weld_near_touch_and_split`] only when
/// the strict crossing test finds nothing: every ring with a real
/// crossing keeps taking the exact same path it always has, and the
/// near-touch check only ever runs on rings that would otherwise pass
/// through untouched — precisely the residual defect's own scope, no
/// wider.
fn split_part_fully(part: GeoPolygon<f64>, depth: u32) -> Vec<GeoPolygon<f64>> {
    const MAX_SPLIT_DEPTH: u32 = 8;
    if depth >= MAX_SPLIT_DEPTH {
        return vec![part];
    }
    let split = split_self_intersection(part.exterior())
        .or_else(|| weld_near_touch_and_split(part.exterior()));
    match split {
        Some((inner, outer)) => {
            let mut parts = split_part_fully(inner, depth + 1);
            parts.extend(split_part_fully(outer, depth + 1));
            parts
        }
        None => vec![part],
    }
}

/// [`split_self_intersection`], applied to every part of `polygon` that
/// needs it — a part that's already simple passes through unchanged.
pub fn split_self_intersections(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(polygon.0.into_iter().flat_map(|part| split_part_fully(part, 0)).collect())
}

pub fn keep_part_near(polygon: MultiPolygon<f64>, reference: Coord<f64>) -> MultiPolygon<f64> {
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

/// Below this, in real m², a polygon part can't plausibly be real ground
/// this zone means to claim — not even standing room for a single
/// pedestrian — so it's a construction artefact (a cut's own thin
/// leftover, a chain union's own pinched-off scrap) rather than real
/// ground `zone_generator` meant to claim.
///
/// Deliberately the *same* number as
/// `geojson_output::tests::MIN_PLAUSIBLE_RING_AREA_M2`, not merely a
/// similar one: that test's own docs already establish 0.05m² as
/// comfortably below the smallest *legitimate* ring found on real data
/// (~0.22m² in Barcelona) — this constant is the production-side half of
/// the same claim, and the two drifting apart is exactly what let 10 real
/// Eixample slivers (0.012m²–0.048m², all triangles) ship for a while:
/// this constant used to be a separate, undocumented `0.01`, low enough
/// that `drop_slivers` judged them "large enough to keep" while the test
/// judged the same shapes "too small to be real" — two contradictory
/// answers to what should be one question.
pub const MIN_KEPT_PART_AREA_M2: f64 = 0.05;

pub fn drop_slivers(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(polygon.0.into_iter().filter(|part| part.unsigned_area() > MIN_KEPT_PART_AREA_M2).collect())
}

/// Removes every interior ring (hole) from every part — a waiting zone is
/// the union of one or more buffered lane strips, always simply connected
/// by construction, so a real hole in the finished output can never be a
/// legitimate feature; every one this pipeline has ever produced is a
/// numerical artefact of unioning several independently-buffered pieces
/// (this function's own docs on `zone_polygon`'s "unioning the finished
/// result with itself" step — meant to *close* a self-touching contour,
/// not to open a second, disjoint loop, but `i_overlay`'s own normaliser
/// can resolve a self-touch either way) or of two parallel lane chains
/// extending to slightly different real lengths, leaving a gap neither
/// individually covers that the combined union boundary nonetheless
/// wraps around.
///
/// This used to be a size filter (`MIN_INTERIOR_RING_AREA_M2 = 1.0`,
/// applied only in `feature::build_feature`, right before writing the
/// GeoJSON) rather than an unconditional drop, on the unstated assumption
/// that a real hole would always be small — confirmed wrong on real data:
/// sweeping every zone in both the Barcelona and Eixample networks finds
/// 106 interior rings, from 1.07m² up to **182.66m²**
/// (`1409641098#0_straight`, a 3-parallel-lane zone whose own chains
/// don't reach equally far back) — nowhere near "small", and every one of
/// them exactly matches one of the two mechanisms above, not a real gap
/// in the road. A size filter could only ever paper over the *small* end
/// of that range.
///
/// Applied first inside [`crate::geojson_output::geometry::zone_polygon`]
/// itself, rather than staying a last-step-only filter in `build_feature`:
/// `resolve_overlaps`'s own overlap/padding decisions read a polygon's
/// `unsigned_area()`, which already silently subtracts any interior ring's
/// own area — leaving a spurious hole in place until the very last step
/// would have those decisions reasoning about ground this zone doesn't
/// actually claim to give up.
///
/// Also applied to every part [`resolve_overlaps`] itself cuts, because
/// `difference` is a *third* way to produce one: when a neighbour's
/// contested ground sits wholly inside this zone rather than touching its
/// boundary, subtracting it leaves an annulus, and the hole is just as
/// illegitimate as the two union-shaped ones above. Waiting until the final
/// pass to drop it was not enough, and was itself the bug: the holed part
/// then fed back into the next round's `difference`/`intersection`, where
/// `i_overlay`'s own `debug_assert` on a hole edge aborts the whole run
/// (confirmed on real Eixample data, `478725549#1_straight`), and a zone
/// that survived to the end carried a hole `build_feature`'s
/// "exactly one ring" contract would reject anyway. `feature`'s final pass
/// applies it once more as a belt-and-braces guarantee that what reaches
/// `build_feature` is unconditionally simply connected.
pub fn drop_interior_rings(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(polygon.0.into_iter().map(|part| GeoPolygon::new(part.exterior().clone(), Vec::new())).collect())
}

pub const SNAP_GRID_METERS: f64 = 0.0001;

pub fn snap_coords(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    let snap = |v: f64| (v / SNAP_GRID_METERS).round() * SNAP_GRID_METERS;
    polygon.map_coords(|c| Coord { x: snap(c.x), y: snap(c.y) })
}

pub fn overlapping_pairs(polygons: &[MultiPolygon<f64>]) -> Vec<(usize, usize)> {
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

pub fn still_overlapping(polygons: &[MultiPolygon<f64>], pairs: &[(usize, usize)]) -> Vec<(usize, usize)> {
    pairs
        .iter()
        .copied()
        .filter(|&(i, j)| overlap_area_m2(&polygons[i], &polygons[j]) > OVERLAP_AREA_THRESHOLD_M2)
        .collect()
}

pub fn max_safe_pad(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    links: &LaneLinks<'_>,
    upper: f64,
    opposing: &MultiPolygon<f64>,
) -> Result<f64> {
    let overlaps_at = |pad: f64| -> Result<bool> {
        Ok(overlap_area_m2(&zone_polygon(zone, lanes, links, pad)?, opposing) > OVERLAP_AREA_THRESHOLD_M2)
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

pub const PAD_SEARCH_STEPS: u32 = 10;

pub const MAX_RESOLUTION_ROUNDS: u32 = 8;

pub fn resolve_overlaps(
    zones: &[E3Detector],
    lanes: &HashMap<&str, &Lane>,
    links: &LaneLinks<'_>,
    stop_points: &[Coord<f64>],
    polygons: &mut [MultiPolygon<f64>],
) -> Result<()> {
    // Every zone pair that could possibly still need fixing, for the rest
    // of this function's own life — one real `O(n²)` scan across the whole
    // network, never repeated (see [`still_overlapping`]'s own docs for why
    // that's sound: both fixes below only ever shrink a polygon, so a pair
    // that isn't here yet can never become one later).
    // Only vehicle-vehicle pairs are resolved. A vehicle zone against a
    // pedestrian one is not resolved because they ship to different clients
    // in different files (`feature::write` splits them) and a pedestrian's
    // own waiting area deliberately sits where a vehicle lane's stop-line
    // buffer reaches. A pedestrian against a pedestrian one is not resolved
    // *here* either: two crossings meeting at one corner each span both
    // banks, so they *share* that corner, and subtracting the shared
    // ground from both (what this function does) would leave neither
    // covering it — taking each zone's own stop line with it.
    // `resolve_pedestrian_overlaps`, run later in `to_feature_collection`,
    // handles the common shape of that case instead (partitioning the
    // shared corner between the two zones rather than cutting it from
    // both); whatever it can't partition is left exactly as this function
    // leaves it — a pedestrian at the shared corner legitimately belongs
    // to either crossing (there is no heading filter), so an unresolved
    // overlap there is a real, if narrow, ambiguity, not a defect this
    // function failed to cut away.
    let both_vehicles = |i: usize, j: usize| {
        zones[i].detect_persons.is_empty() && zones[j].detect_persons.is_empty()
    };
    let candidates: Vec<(usize, usize)> =
        overlapping_pairs(polygons).into_iter().filter(|&(i, j)| both_vehicles(i, j)).collect();
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
                let best = max_safe_pad(&zones[shrink], lanes, links, current, &polygons[opposing])?;
                if best < current {
                    polygons[shrink] = zone_polygon(&zones[shrink], lanes, links, best)?;
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
                // A cut this close to the far side of `part_polygon` can
                // leave `cut` self-crossing rather than genuinely split
                // into separate pieces (see `split_self_intersection`'s
                // own docs) — split it first, so `keep_part_near` below
                // actually has the near/far halves to choose between
                // instead of one pinched ring it has no reason to touch
                // (`polygon.0.len() <= 1` there never fires on a single
                // self-crossing ring).
                let cleaned = drop_interior_rings(keep_part_near(
                    drop_slivers(split_self_intersections(cut)),
                    stop_points[zone],
                ));
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
             {MAX_RESOLUTION_ROUNDS} resolution rounds — left as-is for now; \
             `resolve_vehicle_overlaps` gets another attempt once every other \
             pass has run",
            zones[i].id, zones[j].id,
        );
    }
    Ok(())
}

/// Resolves a pedestrian-pedestrian overlap whose shared ground is a clean
/// quadrilateral — the common "two crossings meet at one corner" shape
/// [`resolve_overlaps`]'s own docs describe as legitimate, not a defect —
/// by splitting that quad along whichever of its two diagonals separates
/// it into one triangle nearer each zone's own stop point, and handing
/// each zone only its own triangle.
///
/// This is deliberately *not* [`resolve_overlaps`]'s own fix: that one
/// subtracts the whole contested patch from *both* zones, opening a gap
/// neither claims — correct for two vehicle zones, where a driver has
/// committed to one lane or the other, but wrong for two pedestrians: a
/// person standing exactly in that gap would then count as waiting for
/// neither crossing, worse than the double-count the overlap warning
/// exists to flag. Partitioning instead keeps the two zones' own combined
/// ground exactly as before — only which one of them claims the shared
/// corner changes — so coverage never drops and a pedestrian anywhere in
/// it still counts as waiting for the nearer crossing.
///
/// A pair is left untouched (and still reported overlapping) whenever the
/// shared ground isn't a simple quadrilateral, or neither diagonal
/// separates it cleanly (see [`split_quad_by_nearest_zone`]) — a shape
/// that doesn't fit this pattern is left exactly as it was rather than
/// forced into a partition that wouldn't make geometric sense. Real
/// Barcelona data has needed this for cases messier than a clean
/// rectangle (three or more crossings meeting at one corner, or a quad
/// `drop_grazing_vertices_everywhere`'s own cleanup left with a fifth,
/// near-collinear vertex); those still show up as an overlap warning for
/// an operator to look at by hand, the same as an unresolved
/// vehicle-vehicle pair.
pub fn resolve_pedestrian_overlaps(
    zones: &[E3Detector],
    stop_points: &[Coord<f64>],
    polygons: &mut [MultiPolygon<f64>],
) {
    let pedestrian = |i: usize| !zones[i].detect_persons.is_empty();
    let candidates: Vec<(usize, usize)> =
        overlapping_pairs(polygons).into_iter().filter(|&(i, j)| pedestrian(i) && pedestrian(j)).collect();

    for (i, j) in candidates {
        let overlap = polygons[i].intersection(&polygons[j]);
        if overlap.unsigned_area() <= OVERLAP_AREA_THRESHOLD_M2 {
            continue; // already below the noise floor `overlapping_pairs` itself judges by
        }
        let quad_split = split_quad_by_nearest_zone(&overlap, stop_points[i], stop_points[j])
            .map(|(a, b)| (MultiPolygon::new(vec![a]), MultiPolygon::new(vec![b])));
        // [`split_quad_by_nearest_zone`]'s own refusal on a same-junction
        // pair is left exactly as this function's own module docs already
        // describe — three or more crossings meeting at one corner is a
        // real, deliberately-ambiguous shared corner, not a defect this
        // function should force a partition onto. [`split_overlap_by_bisector`]
        // — built for [`resolve_vehicle_overlaps`] — only ever runs as a
        // fallback here for two zones belonging to *different* junctions
        // (`pedestrian_junction`, below): that overlap was never the
        // "shared corner" case in the first place — the two crossings
        // aren't even the same physical intersection — so the module
        // docs' own "either crossing is a legitimate answer" reasoning
        // doesn't apply, and it's a genuine geometry defect the same way a
        // vehicle-vehicle overlap between unrelated movements is.
        // Confirmed needed on real Barcelona data
        // (`590815505_w0_straight_ped`/`6556429525_w0_straight_ped`, two
        // closely-spaced junctions along a slip road whose own crossing
        // zones substantially overlap in a shape well past a simple quad).
        let used_quad_split = quad_split.is_some();
        let different_junctions = pedestrian_junction(&zones[i].id) != pedestrian_junction(&zones[j].id);
        let Some((near_i, near_j)) = quad_split.or_else(|| {
            different_junctions.then(|| split_overlap_by_bisector(&overlap, stop_points[i], stop_points[j])).flatten()
        }) else {
            continue;
        };

        // Deliberately no `drop_grazing_vertices_everywhere` on the
        // bisector-fallback path — see `resolve_vehicle_overlaps`'s own
        // docs on why running it independently on each side of a shared
        // cut line can nudge the two copies of that line apart again once
        // reprojected. The quad path keeps running it: it's the one this
        // function has always used, and changing it isn't this fallback's
        // job.
        // `keep_part_near`, but only on the bisector path: a straight cut
        // through a pedestrian zone's own more complex outline (it isn't
        // always the clean rectangle-ish shape the quad path expects) can
        // pinch off a small disconnected sliver where the crossing lane's
        // own stripe bridges the two banks, alongside the one real,
        // dominant remaining piece — confirmed on real Barcelona data
        // (`590815505_w0_straight_ped` cut against `6556429525_w0_straight_ped`:
        // a 154.9m² main piece plus two artefacts of 2.2m² and 0.2m², not
        // three genuine separate banks). The quad path never needed this:
        // it starts from a shape simple enough that a straight cut through
        // it stays one piece, which is exactly the case this pass's own
        // `given_i.0.len() > polygons[i].0.len()` guard, below, still
        // catches for it.
        let keep_near = |zone: usize, polygon: MultiPolygon<f64>| {
            if used_quad_split { polygon } else { keep_part_near(polygon, stop_points[zone]) }
        };
        let finish = |zone: usize, polygon: MultiPolygon<f64>| {
            let cleaned = if used_quad_split { drop_grazing_vertices_everywhere(polygon) } else { polygon };
            keep_near(zone, drop_interior_rings(drop_slivers(split_self_intersections(cleaned))))
        };
        // Each zone keeps everything it already had *except* the other
        // zone's own share — not "cut the whole overlap, then union
        // `near_i` back in". The two are mathematically the same ground
        // (`near_i` and `near_j` exactly partition `overlap`, and
        // `overlap` ⊆ both zones already, so
        // `polygon \ overlap ∪ near_i == polygon \ near_j`) but not the
        // same *operation*: a `difference` against the other zone's own
        // small share is a much more local cut than removing the whole
        // shared patch and rebuilding half of it back with a `union`,
        // whose own seam isn't guaranteed to reconnect a zone's own
        // crossing-lane bridge between its two banks (see
        // `pedestrian_zones`'s own docs) the same way simply never having
        // cut it in the first place does. Confirmed necessary on real
        // Barcelona data: `10265139316_w1_straight_ped`/`..._w3_...`
        // squares off into a clean, cleanly-separable quad but the
        // cut-then-union form left one side split into two pieces; the
        // plain `difference` form here resolves it in one connected piece.
        let give = |zone: usize, polygon: &MultiPolygon<f64>, other_share: &MultiPolygon<f64>| -> MultiPolygon<f64> {
            finish(zone, polygon.difference(other_share))
        };
        let (given_i, given_j) = (give(i, &polygons[i], &near_j), give(j, &polygons[j], &near_i));
        // `near_i`/`near_j` are bit-identical complements of the same
        // `overlap` polygon (see `split_overlap_by_bisector`'s own docs),
        // so `given_i` and `given_j` are supposed to come out disjoint by
        // construction — but each is the result of an independent
        // `difference` against a large, complex zone polygon of its own,
        // and `i_overlay`'s own re-triangulation of that *whole* operation
        // (not just locally at the shared cut line) can still leave a
        // real, measurable residual neither `near_i` nor `near_j` ever
        // captured in the first place. Confirmed on real Barcelona data
        // (`590815505_w0_straight_ped`/`6556429525_w0_straight_ped`): each
        // side's own raw `difference` result barely touched its own
        // `other_share` (a few mm²) individually, yet `given_i` and
        // `given_j` themselves still shared a real 2.2m² — arithmetic
        // noise from two separate large operations, not a second, real
        // dispute over that ground.
        //
        // Subtracting that residual back out (from whichever side is
        // smaller) looks like the obvious fix and isn't one: confirmed on
        // real Barcelona data, handing an already-marginal residual shape
        // into a *third* boolean operation crashes `i_overlay` outright
        // (`assertion failed: overlay_rule.is_fill_top(link.fill)`,
        // `a_long_bent_ribbon_keeps_most_of_its_own_area_when_only_one_
        // end_is_contested`) — the residual is exactly the kind of
        // degenerate, near-zero-width sliver a boolean library's own
        // invariants don't hold up against. Rejecting the whole attempt
        // when a meaningful residual survives is the safe move instead:
        // the pair is left exactly as [`split_quad_by_nearest_zone`]'s own
        // refusals leave it, still reported overlapping for a human, never
        // shipped as geometry this pass can't actually verify is clean.
        if !used_quad_split && given_i.intersection(&given_j).unsigned_area() > MIN_MEANINGFUL_VEHICLE_OVERLAP_M2 {
            continue;
        }
        // Still guarded, not trusted unconditionally: a local cut can in
        // principle still sever a genuinely narrow bridge. Left as a
        // reported overlap, same as `split_quad_by_nearest_zone`'s own
        // refusals, rather than risk a zone with a hole torn in it.
        if given_i.0.is_empty()
            || given_j.0.is_empty()
            || given_i.0.len() > polygons[i].0.len()
            || given_j.0.len() > polygons[j].0.len()
        {
            continue;
        }
        polygons[i] = given_i;
        polygons[j] = given_j;
    }
}

/// If `overlap` is (or squares off cleanly into) a simple quadrilateral,
/// splits it along whichever of its two diagonals leaves each resulting
/// triangle's own centroid closer to a *different* one of `near_a`/`near_b`
/// — the diagonal that actually separates "nearer zone A" ground from
/// "nearer zone B" ground, not an arbitrary one of the two. Returns
/// `(triangle_near_a, triangle_near_b)`.
///
/// Squared off via [`rectangle_if_close`] first, the same tolerance a
/// zone's own finished shape is squared off with — real Barcelona overlaps
/// are rarely a literal four-vertex ring even when they're visibly
/// rectangular; a reprojection or a buffering seam almost always leaves a
/// near-collinear extra vertex or two on the shared patch. Confirmed
/// necessary on real data: `5588597262_w0_straight_ped`/
/// `5588597263_w0_straight_ped` (an operator hand-edited one side of this
/// exact pair expecting the warning to clear) is a five-vertex overlap that
/// only resolves once it's squared off first. Most of a real Barcelona
/// network's own ped-ped overlaps still don't square off this cleanly at
/// all, though — three or more crossings meeting at one corner is at least
/// as common as two, and that shape is genuinely not a quadrilateral no
/// matter how it's measured; those are correctly left as a reported
/// overlap rather than forced through this pass.
///
/// `None` when `overlap` doesn't square off into a single, hole-free
/// four-vertex rectangle at all (a three-or-more-crossing corner, or a
/// shape too far from rectangular for [`rectangle_if_close`]'s own
/// tolerance), or when *neither* diagonal separates it — both triangles
/// read nearer the same zone's own stop point, which happens when one
/// zone's approach sits almost entirely to one side of the shared patch
/// rather than the two genuinely interleaving. Forcing a split in either
/// case would hand one zone a shape that isn't actually the ground nearer
/// it, which is worse than leaving the pair as a reported overlap for a
/// human to look at.
///
/// A real, signalized pedestrian crossing — a lane
/// `zone_generator::pedestrian_zones` lists among some zone's own
/// *entries* but never its *exits* (that function's own module docs: the
/// near bank is both, a crossing lane is entry-only) — always ends up
/// wholly inside exactly one pedestrian zone: its own structural owner,
/// the zone whose own entries name it. No other zone keeps any part of
/// that same lane's own buffered polygon, however it got there.
///
/// Run last, after every cross-zone pass above: a crossing can end up
/// split for more than one reason, not just [`resolve_pedestrian_overlaps`]'s
/// own bisector cut slicing through it. Confirmed on real Barcelona data,
/// junction `590815505`'s own crossing `:590815505_c1_0`: even with that
/// crossing's own reach toward its excluded far bank capped to zero
/// (`zone_polygon`'s own docs on `crossing_banks`), it still landed
/// partly inside the *neighbouring* junction `6556429525`'s own zone —
/// 8.5m away, under one shared `joinTLS` program — because that
/// neighbour's own near-bank walkingarea is independently large enough to
/// reach into `590815505`'s own territory with no crossing lane involved
/// at all. Chasing every way two independently-drawn `netconvert`
/// walkingareas in a dense real cluster can end up physically overlapping
/// is an open-ended search with no guarantee of ever being complete; this
/// instead fixes the one property that actually matters regardless of
/// *why* two zones both reached the same crossing — a client (and the
/// panel) must never see one physical crosswalk drawn as two
/// differently-coloured halves.
pub fn unsplit_crossings(
    zones: &[E3Detector],
    lanes: &HashMap<&str, &Lane>,
    stop_points: &[Coord<f64>],
    polygons: &mut [MultiPolygon<f64>],
    osm_footprints: &HashMap<&str, MultiPolygon<f64>>,
) {
    // Same cleanup every other cross-zone pass in this file already runs
    // on its own `union`/`difference` output before trusting it — see
    // `resolve_overlaps`'s own main loop and `resolve_pedestrian_overlaps`'s
    // own `finish` for the established pattern. Skipping it left a real
    // interior ring behind on real Barcelona data
    // (`2112389514_w1_straight_ped`): unioning a crossing's own polygon
    // back into an owner whose neighbour's own cut had left a
    // near-but-not-quite-matching seam closed off a sliver between them
    // rather than merging cleanly.
    let finish = |zone: usize, polygon: MultiPolygon<f64>| {
        let cleaned = drop_interior_rings(drop_slivers(split_self_intersections(polygon)));
        if cleaned.0.len() > 1 { keep_part_near(cleaned, stop_points[zone]) } else { cleaned }
    };

    // Every real crossing's own buffered polygon, keyed by whichever
    // zone structurally owns it — computed once, up front, entirely
    // independent of `polygons`' own current (still-maybe-split) state.
    // Two neighbouring junctions in the same dense cluster can each own
    // their own, *different* crossing whose buffered polygons happen to
    // touch or overlap each other (two real, physically close crosswalks,
    // not one split in two) — folding "subtract every foreign crossing"
    // and "add back my own" into one pass, zone by zone, in array order,
    // let a *later* zone's own give-back subtract a slice back out of an
    // *earlier* zone's own crossing right where the two touch (confirmed
    // on real Barcelona data, `2119385421_w1_straight_ped`'s own
    // `:2119385421_c0_0` and `2119385421_w2_straight_ped`'s own sibling
    // `:2119385421_c1_0`). Splitting into the two passes below — first
    // strip every foreign crossing from every zone, using every crossing's
    // own original polygon; only once that's done, hand each zone its own
    // back — makes the result independent of iteration order: a zone's
    // own crossing is always unioned in *after* every foreign one has
    // already been subtracted, never the other way around.
    let mut crossings_by_owner: HashMap<usize, Vec<MultiPolygon<f64>>> = HashMap::new();
    for (owner, zone) in zones.iter().enumerate() {
        if zone.detect_persons.is_empty() {
            continue;
        }
        let exit_ids: HashSet<&str> = zone.exits.iter().map(|gate| gate.lane.0.as_str()).collect();
        for entry in &zone.entries {
            let lane_id = entry.lane.0.as_str();
            if exit_ids.contains(lane_id) {
                continue; // the near bank itself (both an entry and an exit), not a crossing lane
            }
            let Some(&lane) = lanes.get(lane_id) else { continue };
            let default_reach = lane.width / 2.0;
            let mut crossing = pedestrian_lane_polygon(lane, default_reach, default_reach);
            // The real, surveyed crosswalk this lane was matched to (see
            // `crosswalks::match_crosswalks`) can reach past SUMO's own
            // synthetic buffer in places — unioned in here, before the
            // subtract/give-back passes below, so it's treated exactly like
            // the rest of this crossing's own footprint rather than a
            // separate thing a neighbouring zone could keep a bite of.
            // Grow-only: this never *shrinks* the SUMO-derived shape, even
            // where the real crosswalk happens to be narrower.
            if let Some(osm_footprint) = osm_footprints.get(lane_id) {
                crossing = crossing.union(osm_footprint);
            }
            if !crossing.0.is_empty() {
                crossings_by_owner.entry(owner).or_default().push(crossing);
            }
        }
    }
    if crossings_by_owner.is_empty() {
        return;
    }

    for zone in 0..zones.len() {
        if zones[zone].detect_persons.is_empty() {
            continue;
        }
        for (&owner, owned) in &crossings_by_owner {
            if owner == zone {
                continue;
            }
            for crossing in owned {
                let claimed = polygons[zone].intersection(crossing);
                if claimed.unsigned_area() > OVERLAP_AREA_THRESHOLD_M2 {
                    polygons[zone] = finish(zone, polygons[zone].difference(crossing));
                }
            }
        }
    }
    for (&owner, owned) in &crossings_by_owner {
        // Restored, not just left alone: the owner's own polygon may
        // already have lost part of a crossing to whatever split it in
        // the first place (a bisector cut, most often) — unioning it
        // back in is what actually gives the owner the *whole* crossing
        // rather than only the fraction the cut happened to leave it.
        for crossing in owned {
            polygons[owner] = finish(owner, polygons[owner].union(crossing));
        }
    }
}

fn split_quad_by_nearest_zone(
    overlap: &MultiPolygon<f64>,
    near_a: Coord<f64>,
    near_b: Coord<f64>,
) -> Option<(GeoPolygon<f64>, GeoPolygon<f64>)> {
    let squared = rectangle_if_close(overlap.clone());
    if squared.0.len() != 1 {
        return None;
    }
    let part = &squared.0[0];
    if !part.interiors().is_empty() {
        return None;
    }
    let mut coords: Vec<Coord<f64>> = part.exterior().coords().copied().collect();
    if coords.first() == coords.last() {
        coords.pop();
    }
    if coords.len() != 4 {
        return None;
    }

    let triangle = |a: Coord<f64>, b: Coord<f64>, c: Coord<f64>| {
        GeoPolygon::new(LineString::new(vec![a, b, c, a]), Vec::new())
    };
    let centroid = |a: Coord<f64>, b: Coord<f64>, c: Coord<f64>| Coord {
        x: (a.x + b.x + c.x) / 3.0,
        y: (a.y + b.y + c.y) / 3.0,
    };
    let distance_sq = |p: Coord<f64>, q: Coord<f64>| {
        let (dx, dy) = (p.x - q.x, p.y - q.y);
        dx * dx + dy * dy
    };

    // Tries splitting along one diagonal (the two triangles it produces),
    // accepting it only if the two triangles' own centroids land nearer
    // *different* zones — see this function's own docs.
    let try_diagonal = |t1: (Coord<f64>, Coord<f64>, Coord<f64>),
                         t2: (Coord<f64>, Coord<f64>, Coord<f64>)|
     -> Option<(GeoPolygon<f64>, GeoPolygon<f64>)> {
        let c1 = centroid(t1.0, t1.1, t1.2);
        let c2 = centroid(t2.0, t2.1, t2.2);
        let t1_nearer_a = distance_sq(c1, near_a) < distance_sq(c1, near_b);
        let t2_nearer_a = distance_sq(c2, near_a) < distance_sq(c2, near_b);
        if t1_nearer_a == t2_nearer_a {
            return None;
        }
        Some(if t1_nearer_a {
            (triangle(t1.0, t1.1, t1.2), triangle(t2.0, t2.1, t2.2))
        } else {
            (triangle(t2.0, t2.1, t2.2), triangle(t1.0, t1.1, t1.2))
        })
    };

    // Diagonal 0-2 (triangles 0,1,2 and 0,2,3), then diagonal 1-3
    // (triangles 1,2,3 and 1,3,0) — whichever one actually separates.
    try_diagonal((coords[0], coords[1], coords[2]), (coords[0], coords[2], coords[3]))
        .or_else(|| try_diagonal((coords[1], coords[2], coords[3]), (coords[1], coords[3], coords[0])))
}

/// Splits `overlap` into the part nearer `near_a` and the part nearer
/// `near_b`, by clipping it against the half-plane on each side of the
/// perpendicular bisector of the segment `near_a`->`near_b`. Unlike
/// [`split_quad_by_nearest_zone`], this makes no assumption about
/// `overlap`'s own shape (a clean four-vertex rectangle, or anything
/// else) — a half-plane clip via [`BooleanOps::intersection`] partitions
/// whatever is handed to it, which is what a vehicle-vehicle overlap
/// needs: it is the leftover of a `difference` cut, not a hand-drawn
/// crossing corner, and rarely squares off as cleanly as a pedestrian
/// one does.
///
/// `None` only when `near_a` and `near_b` coincide — there is then no
/// direction to bisect along at all, left as an unresolved, still-reported
/// overlap by the caller, same as [`split_quad_by_nearest_zone`]'s own
/// refusals. Unlike that function, landing entirely on one side is *not*
/// treated as a refusal: it means every bit of the disputed ground is
/// genuinely nearer one zone's own stop line than the other's, so that
/// zone keeps all of it and the far zone's own share is simply empty —
/// still a valid partition, just a lopsided one.
fn pedestrian_junction(id: &sumo_types::additional::domain::DetectorId) -> &str {
    let id: &str = id.as_ref();
    id.split("_w").next().unwrap_or(id)
}

fn split_overlap_by_bisector(
    overlap: &MultiPolygon<f64>,
    near_a: Coord<f64>,
    near_b: Coord<f64>,
) -> Option<(MultiPolygon<f64>, MultiPolygon<f64>)> {
    let (dx, dy) = (near_b.x - near_a.x, near_b.y - near_a.y);
    let len = dx.hypot(dy);
    if len < 1e-6 {
        return None;
    }
    let (ux, uy) = (dx / len, dy / len); // unit vector, near_a -> near_b
    let (px, py) = (-uy, ux); // perpendicular to it
    let mid = Coord { x: (near_a.x + near_b.x) / 2.0, y: (near_a.y + near_b.y) / 2.0 };

    // Far past any real zone's own extent — this crate's own local
    // coordinates are metre-scale, so a million metres is comfortably
    // outside anything `overlap` could ever reach, the same margin
    // [`weld_near_touch_and_split`]'s own callers rely on elsewhere in
    // this file for "large enough to not matter, small enough to stay
    // finite".
    const EXTENT: f64 = 1_000_000.0;
    // `sign > 0.0` builds the half-plane on `near_a`'s own side of the
    // bisector (extending away from `near_b`); `sign < 0.0` builds
    // `near_b`'s.
    let half_plane = |sign: f64| -> MultiPolygon<f64> {
        let p1 = Coord { x: mid.x + px * EXTENT, y: mid.y + py * EXTENT };
        let p2 = Coord { x: mid.x - px * EXTENT, y: mid.y - py * EXTENT };
        let p3 = Coord { x: p2.x - ux * EXTENT * sign, y: p2.y - uy * EXTENT * sign };
        let p4 = Coord { x: p1.x - ux * EXTENT * sign, y: p1.y - uy * EXTENT * sign };
        MultiPolygon::new(vec![GeoPolygon::new(LineString::new(vec![p1, p2, p3, p4, p1]), Vec::new())])
    };

    let near_a_half = overlap.intersection(&half_plane(1.0));
    // `overlap`'s own complement of `near_a_half`, not a second independent
    // `intersection` against `half_plane(-1.0)` — this is the whole reason
    // the two shares end up sharing bit-identical vertices along their cut
    // line rather than two close-but-not-quite-matching ones. Confirmed
    // necessary on real Barcelona data: computed as two separate
    // intersections, `resolve_vehicle_overlaps`'s own two `difference`
    // calls each carved their zone's own new boundary from a *different*
    // polygon (this same `overlap`, sliced by the same line but through
    // two unrelated boolean-op calls), and the boolean library's own
    // floating-point rounding didn't always land the two results on
    // exactly the same points — reopening, once reprojected to lon/lat, a
    // real if small overlap right where this pass had just cut one away.
    // `difference` against `near_a_half` itself instead guarantees the
    // shared edge is the exact same one `near_a_half` already has.
    let near_b_half = overlap.difference(&near_a_half);
    // Self-unioned before handing either share back as a future
    // `difference` subtrahend — same trick `zone_polygon`'s own
    // `polygon.union(&polygon)` uses to close a self-touching contour, and
    // `resolve_overlaps`'s own main loop uses on a multi-neighbour cut for
    // the same reason (see its own docs): clipping a complex, many-vertex,
    // non-convex pedestrian overlap against a half-plane can leave `near_a_half`/
    // `near_b_half` as several disjoint parts rather than one clean piece,
    // and a `MultiPolygon` handed to `difference` still self-overlapping or
    // barely-touching along its own part boundaries is exactly the
    // "subtrahend a difference never otherwise gets to clean up itself"
    // case that function's own docs warn about — cheap insurance against
    // the same failure mode on a genuinely complex, non-convex pedestrian
    // overlap (`resolve_pedestrian_overlaps`'s own caller is the only one
    // that ever hands this function a shape that isn't close to a simple
    // rectangle). [`resolve_pedestrian_overlaps`]'s own residual check,
    // after both `difference` calls, is what actually guarantees the two
    // results end up disjoint — this is a first line of defence, not a
    // substitute for that guarantee.
    let clean = |half: MultiPolygon<f64>| half.union(&half);
    Some((clean(near_a_half), clean(near_b_half)))
}

/// [`resolve_vehicle_overlaps`]'s own trigger — the same "a real client's
/// GPS fix could actually land in both zones" bound
/// `tests::geojson_zones_do_not_overlap`'s own `MIN_MEANINGFUL_OVERLAP_M2`
/// judges the finished output by, not `OVERLAP_AREA_THRESHOLD_M2`'s own
/// square-millimetre goal. The overwhelming majority of what
/// `overlapping_pairs` finds this late — after `resolve_overlaps`'s own
/// rounds, every per-zone cleanup pass and `rectangle_if_close`'s own
/// squaring have all already run — is exactly that kind of residual
/// arithmetic noise, not a real dispute: cutting a real zone over a sliver
/// neither a client nor a human would ever notice only adds risk (a fresh
/// `difference` can, in principle, leave a self-intersection behind — see
/// [`resolve_vehicle_overlaps`]'s own guard) for no product benefit.
pub const MIN_MEANINGFUL_VEHICLE_OVERLAP_M2: f64 = 0.01;

/// Resolves a vehicle-vehicle overlap left over once every other pass has
/// run — [`resolve_overlaps`]'s own iterative shrink-and-cut, then every
/// per-zone cleanup pass (`despike`, `drop_grazing_vertices_everywhere`,
/// `relax_needle_vertices_everywhere`, …) `feature::to_feature_collection`
/// runs afterward, any of which can nudge a boundary [`resolve_overlaps`]
/// had already made disjoint back into a sliver of real, measurable
/// overlap (see `resolve_overlaps`'s own docs on why cleanup runs once,
/// outside its loop, rather than being re-litigated round to round).
///
/// Run on the zones' own finished, per-zone-cleaned polygons — the same
/// point in the pipeline [`resolve_pedestrian_overlaps`] runs at, and for
/// the same reason: the overlap this looks for has to be the real,
/// settled one, not a still-noisy intermediate shape neither pass would
/// ever see again.
///
/// Two zones sharing one approach that hasn't yet diverged into separate
/// lanes (`-207322888#12_straight` and `-207322888#12_straight+right`,
/// siblings off the same source edge — see
/// `tests::geojson_zones_do_not_overlap`'s own `ZONES_SHARING_ONE_APPROACH`)
/// both genuinely reach back over the same asphalt, and
/// [`resolve_overlaps`]'s own cut-from-both fix (correct where two zones
/// simply shouldn't share any ground at all) would open a gap neither
/// zone claims right where a real, undecided driver is standing. This
/// partitions the disputed ground between the two zones instead — nearer
/// each zone's own stop line goes to that zone — so the pair's combined
/// coverage never shrinks and every point in dispute still counts for
/// exactly one movement, the nearer one, rather than for both or for
/// neither.
///
/// Left as a reported overlap, same as [`resolve_pedestrian_overlaps`]'s
/// own refusals, whenever [`split_overlap_by_bisector`] can't partition
/// the shared ground, or the resulting cut is degenerate (empties a
/// zone, or leaves it with more disjoint parts than it started with) —
/// a local cut can in principle still sever a genuine, if narrow, bridge
/// of ground a zone needs to stay connected to its own stop line.
pub fn resolve_vehicle_overlaps(
    zones: &[E3Detector],
    stop_points: &[Coord<f64>],
    polygons: &mut [MultiPolygon<f64>],
) {
    let vehicle = |i: usize| zones[i].detect_persons.is_empty();
    let candidates: Vec<(usize, usize)> =
        overlapping_pairs(polygons).into_iter().filter(|&(i, j)| vehicle(i) && vehicle(j)).collect();

    for (i, j) in candidates {
        let overlap = polygons[i].intersection(&polygons[j]);
        if overlap.unsigned_area() <= MIN_MEANINGFUL_VEHICLE_OVERLAP_M2 {
            continue; // see that constant's own docs — arithmetic noise, not a real dispute
        }
        let Some((near_i, near_j)) = split_overlap_by_bisector(&overlap, stop_points[i], stop_points[j]) else {
            continue;
        };

        // Deliberately no `drop_grazing_vertices_everywhere` here, unlike
        // [`resolve_pedestrian_overlaps`]'s own `finish` — confirmed on
        // real Barcelona data (`-207322888#12_straight`/`+right`,
        // `550448817#3_straight+left`/`+right`): running it independently
        // on `given_i` and `given_j` nudged their shared new boundary a
        // hair differently for each zone, reopening a real, several-
        // square-centimetre overlap once reprojected to lon/lat — smaller
        // than the one this pass had just cut away, but still well past
        // `MIN_MEANINGFUL_OVERLAP_M2`. `near_i`/`near_j` already share
        // bit-identical vertices along that line (see
        // [`split_overlap_by_bisector`]'s own docs on why `near_j` is
        // `near_i`'s own complement rather than a second, independent cut)
        // — a pass that can reposition either side's own copy of it
        // separately is the one thing left that could still break that
        // agreement.
        let finish = |polygon: MultiPolygon<f64>| drop_interior_rings(drop_slivers(split_self_intersections(polygon)));
        // Each zone keeps everything it already had except the *other*
        // zone's own share of the disputed ground — see
        // [`resolve_pedestrian_overlaps`]'s own `give` for why this, not
        // "cut the whole overlap out and union the zone's own share back
        // in", is the right operation.
        let give = |polygon: &MultiPolygon<f64>, other_share: &MultiPolygon<f64>| -> MultiPolygon<f64> {
            finish(polygon.difference(other_share))
        };
        let (given_i, given_j) = (give(&polygons[i], &near_j), give(&polygons[j], &near_i));
        if given_i.0.is_empty()
            || given_j.0.is_empty()
            || given_i.0.len() > polygons[i].0.len()
            || given_j.0.len() > polygons[j].0.len()
        {
            continue;
        }
        polygons[i] = given_i;
        polygons[j] = given_j;
    }
}

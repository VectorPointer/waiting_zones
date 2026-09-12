use anyhow::{Context, Result};
use geo::{
    Area, BooleanOps, ConvexHull, Coord, LineString, MultiPolygon, Polygon as GeoPolygon, Simplify,
};
use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, LineJoin, StrokeStyle};
use std::collections::{BTreeSet, HashMap, HashSet};
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{EdgeFunction, EdgeId, Lane, LaneIndex, Network, Point, Shape, VClass};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;
use crate::geojson_output::overlaps::{drop_interior_rings, drop_slivers, snap_coords};
use crate::geojson_output::reprojection::{
    distance_from_start, padded_entry, shape_length, trimmed_path_points,
};

pub const ROUND_JOIN_SEGMENT_ANGLE_RADIANS: f64 = 0.3;

pub fn shapes_to_multipolygon(shapes: i_overlay::i_shape::base::data::Shapes<[f64; 2]>) -> MultiPolygon<f64> {
    let close = |points: Vec<[f64; 2]>| -> LineString<f64> {
        let mut coords: Vec<Coord<f64>> = points.into_iter().map(|[x, y]| Coord { x, y }).collect();
        if !coords.is_empty() && coords.first() != coords.last() {
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

/// How close a vertex's two neighbours (skipping the vertex itself) have
/// to land, in metres, to treat that vertex as a spike tip worth removing
/// — see [`weld_and_remove_spikes`]'s own docs for the shape of defect
/// this catches. Two independently-buffered pieces meant to share a
/// boundary exactly land their own copy of that shared point a few
/// millimetres apart (confirmed on real Barcelona data,
/// `1395134884#0_straight`: ~3mm), not exactly on top of each other —
/// `i_overlay`'s own `DeSpikeContour` needs the two neighbours to be
/// exactly, bit-for-bit equal to fire at all (it checks the two edge
/// vectors' cross product against exactly zero) and silently no-ops on a
/// few millimetres of real floating-point noise, so it can't be reused
/// as-is here. Comfortably above that few-millimetre noise floor and
/// comfortably below anything a real lane's own geometry legitimately
/// bends within.
pub const SPIKE_WELD_EPSILON_METERS: f64 = 0.15;

/// Removes two shapes of noise from a ring, both stemming from the exact
/// same root cause: two independently-buffered pieces meant to share a
/// boundary exactly (two parallel same-edge lanes' own core gates, or a
/// chain meeting the neighbouring lane's own core gate) land their own
/// copy of that shared point a few millimetres apart rather than exactly
/// on top of each other, confirmed on real Barcelona data
/// (`1395134884#0_straight`) with no `resolve_overlaps` cut involved at
/// all, so the fix belongs here, on the union's own output, not on
/// `resolve_overlaps`:
///
/// - A vertex `p1` whose immediate neighbours `p0`/`p2` land within
///   [`SPIKE_WELD_EPSILON_METERS`] of each other — the boundary running
///   out to `p1` and straight back rather than continuing past it,
///   enclosing no real area of its own. Removing `p1` alone would leave
///   `p0` and `p2` as an (almost) exact duplicate pair right next to each
///   other; removing both collapses that pair down to the one real point
///   they both approximate.
/// - Two *adjacent* vertices that are themselves within
///   [`SPIKE_WELD_EPSILON_METERS`] of each other, with no spike tip
///   between them at all. Sub-centimetre apart, the edge joining them is
///   too short for its own direction to mean anything, which then reads
///   as a sharp turn at *its* neighbours purely from that noise — dropping
///   one of the pair removes the meaningless edge rather than leaving it
///   in to keep confusing its neighbours' own angles.
///
/// Runs to a fixed point (removing one instance can expose another
/// behind it), bounded by the ring's own shrinking length rather than
/// risking an infinite loop.
fn weld_and_remove_spikes(mut points: Vec<Coord<f64>>) -> Vec<Coord<f64>> {
    loop {
        let n = points.len();
        if n < 4 {
            return points;
        }
        let close = |a: Coord<f64>, b: Coord<f64>| (b.x - a.x).hypot(b.y - a.y) < SPIKE_WELD_EPSILON_METERS;
        let duplicate = (0..n).find(|&i| close(points[i], points[(i + 1) % n]));
        if let Some(i) = duplicate {
            points.remove((i + 1) % n);
            continue;
        }
        let spike = (0..n).find(|&i| close(points[(i + n - 1) % n], points[(i + 1) % n]));
        let Some(i) = spike else {
            return points;
        };
        let i2 = (i + 1) % n;
        points = points
            .into_iter()
            .enumerate()
            .filter_map(|(j, p)| (j != i && j != i2).then_some(p))
            .collect();
    }
}

/// [`weld_and_remove_spikes`], applied to every ring of `polygon`.
pub fn despike(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    let despike_ring = |ring: &LineString<f64>| -> Option<LineString<f64>> {
        let mut coords: Vec<Coord<f64>> = ring.coords().copied().collect();
        if !coords.is_empty() && coords.first() == coords.last() {
            coords.pop();
        }
        let cleaned = weld_and_remove_spikes(coords);
        if cleaned.len() < 3 {
            return None;
        }
        let mut closed = cleaned;
        closed.push(closed[0]);
        Some(LineString::new(closed))
    };

    MultiPolygon::new(
        polygon
            .0
            .into_iter()
            .filter_map(|part| {
                let exterior = despike_ring(part.exterior())?;
                let interiors = part.interiors().iter().filter_map(despike_ring).collect();
                Some(GeoPolygon::new(exterior, interiors))
            })
            .collect(),
    )
}

pub fn buffer_shape(shape: &Shape, entry: Length, exit: Length, half_width: Length, end_cap: LineCap<[f64; 2], f64>) -> MultiPolygon<f64> {
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

pub fn merged_core_polygon(lane_gates: &[(&Lane, Length, Length)]) -> MultiPolygon<f64> {
    lane_gates
        .iter()
        .map(|&(lane, entry, exit)| buffer_shape(&lane.shape, entry, exit, lane.width / 2.0, LineCap::Butt))
        .reduce(|acc, polygon| acc.union(&polygon))
        .unwrap_or_else(|| MultiPolygon::new(Vec::new()))
}

/// The smallest rectangle (any orientation) that contains every one of
/// `coords`, widened — never narrowed — so neither side falls below
/// `min_side` metres. `None` for fewer than three distinct points.
///
/// This is the shape a pedestrian waiting area is rendered as. A
/// walkingarea's own `shape` traces the path `netconvert` laid around a
/// street corner: it is frequently L-shaped, curved, or even doubles back
/// on itself, so treating it directly as a polygon (or taking its convex
/// hull) yields a wedge or a thin sliver that no client can geofence
/// against a real GPS fix. The corner a pedestrian actually waits on is
/// bounded by that same path, so the *rectangle it sits inside* is the
/// smallest honest stand-in: it never reaches past ground the walkingarea
/// itself occupies, and it squares the corner off the way the map reads.
///
/// Minimum-area, not axis-aligned: a crossing is rarely parallel to the
/// projected axes, and an axis-aligned box would grow to cover the whole
/// diagonal and spill across the street. Rotating calipers over the convex
/// hull (`min_side` is applied last, only to keep a near-collinear shape
/// from collapsing to zero width) finds the orientation that hugs the
/// corner instead.
fn min_area_rectangle(coords: &[Coord<f64>], min_side: f64) -> Option<MultiPolygon<f64>> {
    if coords.len() < 3 {
        return None;
    }
    let hull = GeoPolygon::new(LineString::new(coords.to_vec()), Vec::new()).convex_hull();
    let hull_coords: Vec<Coord<f64>> = hull.exterior().coords().copied().collect();
    // `convex_hull` closes its ring (first == last); the edge loop below
    // wants the distinct vertices only.
    let points = &hull_coords[..hull_coords.len().saturating_sub(1)];
    // A degenerate, collinear shape's hull is a single segment (two
    // points); the edge loop below still finds its direction, and the
    // widening pass gives it the lane's own width.
    if points.len() < 2 {
        return None;
    }

    let mut best: Option<(f64, Coord<f64>, f64, f64)> = None;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let len = dx.hypot(dy);
        if len <= f64::EPSILON {
            continue;
        }
        let (ux, uy) = (dx / len, dy / len);
        let (mut min_u, mut max_u, mut min_v, mut max_v) =
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for p in points {
            let u = p.x * ux + p.y * uy;
            let v = p.x * -uy + p.y * ux;
            min_u = min_u.min(u);
            max_u = max_u.max(u);
            min_v = min_v.min(v);
            max_v = max_v.max(v);
        }
        let area = (max_u - min_u) * (max_v - min_v);
        if best.is_none_or(|(best_area, _, _, _)| area < best_area) {
            best = Some((area, a, ux, uy));
        }
    }

    let (_, a, ux, uy) = best?;
    // `a` is a hull vertex on the winning edge, so every other point's own
    // projection is relative to it — recompute the bounds in that frame so
    // the rectangle can be placed back in world coordinates.
    let (mut min_u, mut max_u, mut min_v, mut max_v) =
        (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
    for p in points {
        let (rx, ry) = (p.x - a.x, p.y - a.y);
        let u = rx * ux + ry * uy;
        let v = rx * -uy + ry * ux;
        min_u = min_u.min(u);
        max_u = max_u.max(u);
        min_v = min_v.min(v);
        max_v = max_v.max(v);
    }
    // Square off the two sides of the corner to at least `min_side`,
    // centred on the shape so a degenerate, near-straight walkingarea
    // still yields a real rectangle rather than a zero-width line.
    let mut half_u = (max_u - min_u) / 2.0;
    let mut half_v = (max_v - min_v) / 2.0;
    let centre_u = (min_u + max_u) / 2.0;
    let centre_v = (min_v + max_v) / 2.0;
    if half_u <= half_v {
        half_u = half_u.max(min_side / 2.0);
    } else {
        half_v = half_v.max(min_side / 2.0);
    }
    let corner = |du: f64, dv: f64| {
        let u = centre_u + du;
        let v = centre_v + dv;
        Coord {
            x: a.x + u * ux + v * -uy,
            y: a.y + u * uy + v * ux,
        }
    };
    let mut ring = vec![
        corner(-half_u, -half_v),
        corner(half_u, -half_v),
        corner(half_u, half_v),
        corner(-half_u, half_v),
    ];
    ring.push(ring[0]);
    Some(MultiPolygon::new(vec![GeoPolygon::new(LineString::new(ring), Vec::new())]))
}

/// The ground a walkingarea covers, squared off into the smallest
/// rectangle that contains every point `netconvert` drew for it — see
/// [`min_area_rectangle`]'s own docs for why a walkingarea's `shape` is
/// not itself a usable footprint.
pub fn pedestrian_lane_polygon(lane: &Lane) -> MultiPolygon<f64> {
    let coords: Vec<Coord<f64>> = lane.shape.0.iter().map(|p| Coord { x: p.x, y: p.y }).collect();
    min_area_rectangle(&coords, lane.width.get::<meter>()).unwrap_or_else(|| MultiPolygon::new(Vec::new()))
}

pub fn merged_pedestrian_polygon(lanes: &[&Lane]) -> MultiPolygon<f64> {
    lanes
        .iter()
        .map(|lane| pedestrian_lane_polygon(lane))
        .reduce(|acc, polygon| acc.union(&polygon))
        .unwrap_or_else(|| MultiPolygon::new(Vec::new()))
}

pub fn centroid(points: &[Point]) -> Point {
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

pub fn zone_modes(zone: &E3Detector, lanes: &HashMap<&str, &Lane>) -> Vec<&'static str> {
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

/// The lane-to-lane links `zone_polygon` and `chain_shape` need: each
/// lane's single successor and the via bridging that hop, so a chain is
/// drawn forward, from an ancestor into the control lane. Bundled into a
/// named type so neither function's own signature drifts a parameter with
/// every future link it needs.
pub struct LaneLinks<'a> {
    pub successors: HashMap<&'a str, (&'a str, Option<&'a str>)>,
}

pub fn lane_links(network: &Network) -> LaneLinks<'_> {
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

    let successors = successors_by_from
        .into_iter()
        .filter_map(|(from, successors)| {
            let distinct: HashSet<&str> = successors.iter().map(|&(lane, _)| lane).collect();
            (distinct.len() == 1).then(|| (from, successors[0]))
        })
        .collect();

    LaneLinks { successors }
}

pub fn chain_shape<'a>(
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
            // A straight line between the via's own endpoints, not its
            // real (possibly sharply curved) shape: SUMO draws a via lane
            // as the *actual* swept turning path through a junction, which
            // at a busy, tightly-packed intersection can be a long, bent
            // curve — several such curves from different approaches
            // converging within a few metres of each other is exactly what
            // produced `203480266#0_straight`'s own self-intersecting ring
            // (confirmed: its own merge point combines a ~1.5m direct via
            // with a ~13m, 5-point sweeping one). The straight line still
            // meets both real lanes on either side exactly — a via's own
            // endpoints are shared with theirs by construction — so the
            // chain stays one continuous, gap-free path; it just stops
            // carrying the via's own interior bends into the union.
            if let (Some(&first), Some(&last)) = (via_lane.shape.0.first(), via_lane.shape.0.last()) {
                points.push(first);
                points.push(last);
            }
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

pub const SIMPLIFY_TOLERANCE_METERS: f64 = 0.05;

const MAX_FILLED_CONCAVITY_AREA_RATIO: f64 = 1.25;

/// Replaces a part with its own convex hull when that barely changes its
/// area — smoothing away the sub-metre notch a union of two buffers
/// leaves at their seam, without moving a boundary that encloses real
/// ground.
///
/// Deliberately per part, never over the whole `MultiPolygon` at once. An
/// earlier version took the hull of everything first and returned it
/// whenever *its* area passed the same ratio test, which on a zone whose
/// parts are genuinely disconnected (a core gate plus an extended-ancestor
/// chain that doesn't reach it — see [`zone_polygon`]) bridged straight
/// across the gap between them and claimed it: measured on real Barcelona
/// data, 154 vehicle zones grew by 3816m² in total that way, one of them
/// (`550667908#0_straight`) tripling from 55m² to 166m² by swallowing
/// ground no lane of its own covers. A zone ends where the gap is (see
/// `to_feature_collection`'s own `keep_part_near` reduction); filling one
/// in here quietly undid that.
fn fill_small_concavities(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(
        polygon
            .0
            .into_iter()
            .map(|part| {
                let hull = part.convex_hull();
                if hull.unsigned_area() <= part.unsigned_area() * MAX_FILLED_CONCAVITY_AREA_RATIO {
                    hull
                } else {
                    part
                }
            })
            .collect(),
    )
}

pub fn zone_polygon(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    links: &LaneLinks<'_>,
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

        if is_pedestrian {
            // Every entry of a pedestrian zone is a walkingarea — the near
            // bank (which is also its exit) and the far bank (an entry
            // only). Both are ground the zone covers, so both are core
            // lanes; matching on exits would draw only the near bank.
            pedestrian_core_lanes.push(lane);
        } else if let Some(&exit_distance) = exit_position_by_lane.get(entry.lane.0.as_str()) {
            let entry_distance = distance_from_start(entry.position, lane.length);
            core_gates.push((lane, padded_entry(entry_distance, exit_distance, target_length), exit_distance));
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
        if let Some(&(successor, _)) = links.successors.get(lane)
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
        if let Some(&(successor, _)) = links.successors.get(lane)
            && ancestor_lanes.contains(successor)
        {
            *in_degree.entry(successor).or_insert(0) += 1;
        }
    }
    let segment_starts =
        ancestor_lanes.iter().filter(|lane| in_degree.get(*lane).copied().unwrap_or(0) != 1);
    for &start in segment_starts {
        let Some((chain_lanes, shape, terminal_successor)) =
            chain_shape(start, &ancestor_lanes, &in_degree, &links.successors, lanes)
        else {
            continue;
        };
        // An ancestor with no resolvable successor is a short connector stub
        // (often an internal lane's 0.2 m remnant), not a path leading into
        // this zone's controlled lanes. Drawing its capped buffer creates a
        // narrow incision in the main polygon, so leave that stub out.
        let Some(terminal_successor) = terminal_successor.as_deref() else {
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
        if let Some(&gate_idx) = core_gate_index_by_lane.get(terminal_successor)
            && absorbed_core_gates.contains(&gate_idx)
        {
            let (core_lane, _core_entry, _core_exit_distance) = core_gates[gate_idx];
            let mut combined_points = shape.0.clone();
            combined_points.extend(core_lane.shape.0.iter().copied());
            let combined_shape = Shape(combined_points);
            // The exit is the *combined path's* own end, measured on it
            // directly rather than summed from `chain_length` + the core
            // gate's lane-distance: a hop between two real lanes can thread
            // more than one internal lane, and `chain_shape` only bridges
            // the connection's own `via` (the first one). Summing would
            // then fall short by exactly the remaining internal length —
            // confirmed on real Barcelona data, `734604335#0_straight` was
            // cut 5.41m short of its own stop line that way.
            let total_exit = shape_length(&combined_shape);
            let entry_distance = padded_entry(Length::new::<meter>(0.0), total_exit, target_length);
            let half_width = core_lane.width / 2.0;
            polygon =
                polygon.union(&buffer_shape(&combined_shape, entry_distance, total_exit, half_width, LineCap::Butt));
            continue;
        }

        let total_length = shape_length(&shape);
        let entry_distance = padded_entry(Length::new::<meter>(0.0), total_length, target_length);
        let half_width = chain_lanes[0].width / 2.0;
        // Keep the connecting end flush with the chain's real endpoint.
        // Extending it with a square cap creates a narrow wedge when this
        // chain meets a neighbouring lane buffer, which is rendered as an
        // incision into the finished waiting zone.
        let end_cap = LineCap::Butt;
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
    // Boolean unions can leave a self-touching contour where several capped
    // lane buffers meet. Unioning the finished result with itself makes the
    // overlay engine normalize that contour before it is serialized, rather
    // than exposing the touching boundary as an incision in the zone.
    let polygon = fill_small_concavities(despike(polygon.union(&polygon)));
    let polygon = if is_pedestrian { polygon } else { polygon.simplify(SIMPLIFY_TOLERANCE_METERS) };
    // `simplify`'s own Douglas-Peucker pass can drop a vertex that used to
    // sit between two of `despike`'s own near-duplicates above, newly
    // making them adjacent to each other where they weren't when `despike`
    // ran the first time -- run it again on whatever `simplify` leaves
    // behind, rather than leaving that new pair for the next thing down
    // the pipe to trip over.
    let polygon = despike(polygon);
    Ok(drop_interior_rings(snap_coords(drop_slivers(polygon))))
}

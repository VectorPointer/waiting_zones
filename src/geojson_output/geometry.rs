use anyhow::{Context, Result};
use geo::{
    BooleanOps, Coord, LineString, MultiPolygon, Polygon as GeoPolygon, Simplify,
};
use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, LineJoin, StrokeStyle};
use std::collections::{BTreeSet, HashMap, HashSet};
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{EdgeFunction, EdgeId, Lane, LaneIndex, Network, Point, Shape, VClass};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;
use crate::geojson_output::overlaps::{drop_slivers, snap_coords};
use crate::geojson_output::reprojection::{
    distance_from_start, padded_entry, shape_length, trimmed_path_points,
};

pub const ROUND_JOIN_SEGMENT_ANGLE_RADIANS: f64 = 0.3;

pub fn shapes_to_multipolygon(shapes: i_overlay::i_shape::base::data::Shapes<[f64; 2]>) -> MultiPolygon<f64> {
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

pub fn pedestrian_lane_polygon(lane: &Lane) -> MultiPolygon<f64> {
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

pub fn single_successors(network: &Network) -> HashMap<&str, (&str, Option<&str>)> {
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

pub const SIMPLIFY_TOLERANCE_METERS: f64 = 0.05;

pub fn zone_polygon(
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
        if let Some(&gate_idx) = terminal_successor
            .as_deref()
            .and_then(|id| core_gate_index_by_lane.get(id))
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

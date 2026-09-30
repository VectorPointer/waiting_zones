//! Matches a real, surveyed OSM crosswalk (`footway=crossing`, the zebra
//! stripe a pedestrian's GPS actually crosses and the panel actually
//! draws) to *the specific* SUMO crossing lane it belongs to, so
//! [`crate::geojson_output::overlaps::unsplit_crossings`] can widen that
//! lane's own buffered footprint to fully contain it.
//!
//! Calibrated against real Barcelona data: comparing every pedestrian
//! zone's own crossing lane to every OSM `footway=crossing` way within the
//! network, the median offset among plausible matches is 0.8m and the
//! median bearing difference is 3.5°, while an unrelated (unsignalized)
//! crossing nearby is typically 10m+ away. [`MATCH_DISTANCE_METERS`] and
//! [`MATCH_ANGLE_DEGREES`] are set from that distribution, not guessed.

use std::collections::HashMap;

use i_overlay::mesh::style::LineCap;
use osm_crosswalks::OsmCrosswalk;
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{EdgeFunction, Lane, Network, Point, Shape};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

use crate::geojson_output::geometry::buffer_shape;
use crate::geojson_output::reprojection::Reprojector;
use geo::{BooleanOps, MultiPolygon};

/// How far apart (metres, in local network coordinates) a SUMO crossing
/// lane and an OSM crosswalk may sit and still be considered the same
/// physical crossing. See this module's own docs for where this number
/// comes from.
pub const MATCH_DISTANCE_METERS: f64 = 4.0;

/// How different the two lines' own bearings may be (degrees, undirected —
/// a crossing walked in either direction is the same crossing) and still
/// count as "roughly parallel".
pub const MATCH_ANGLE_DEGREES: f64 = 20.0;

/// The angle tolerance for a crosswalk that runs through the crossing's
/// own junction node: that shared node already says it's this junction's
/// crossing, so only a gross misalignment rules it out. Calibrated on real
/// Barcelona data: SUMO's crossing lane at `302182247` is a 3m stub 25° off
/// the surveyed zebra through the same node, and `5588789873`'s 50° off;
/// while at `5588790790` a bicycle crossing of *another* arm, one SUMO put
/// no crossing on, runs through the node 58°+ off every crossing lane there.
pub const TOPOLOGICAL_MATCH_ANGLE_DEGREES: f64 = 55.0;

/// The distance tolerance for such a crosswalk, for the same reason: a
/// long surveyed crossing through the junction node can sit several metres
/// from SUMO's own short stub for it (confirmed on real Barcelona data,
/// `6556429525`: 6m).
pub const TOPOLOGICAL_MATCH_DISTANCE_METERS: f64 = 8.0;

/// The OSM node a SUMO crossing lane's junction was built from: `netconvert`
/// names a crossing lane `:{junction}_c{n}_{lane}`, and keeps an OSM node's
/// own id as the junction id (clusters get other names, which don't parse).
fn lane_junction(lane_id: &str) -> Option<i64> {
    lane_id.strip_prefix(':')?.rsplit_once("_c")?.0.parse().ok()
}

/// When a lane's best OSM match and its runner-up land within this many
/// metres of each other, the match is ambiguous — left unmatched (SUMO-only
/// geometry for that lane) rather than guessed, the same "leave it for a
/// human" stance the rest of this crate's overlap resolution already takes.
pub const AMBIGUOUS_MARGIN_METERS: f64 = 1.0;

/// An OSM way several lanes claim is shared out between them by proximity
/// (`partition_to_nearest_lane`) — a share shorter than this (metres)
/// contributes nothing worth adding.
pub const MIN_MATCHED_SPAN_METERS: f64 = 1.0;

/// The outcome of matching every pedestrian zone's own crossing lane
/// against `network`'s sibling `.osm` crosswalks.
pub struct CrosswalkMatches {
    /// Each matched lane's real-world footprint, in this crate's own local
    /// network metres — ready to fold straight into `unsplit_crossings`.
    pub footprints: HashMap<String, MultiPolygon<f64>>,
    /// Which OSM crosswalks (their first way's id) each matched lane got.
    pub matched_ways: HashMap<String, Vec<i64>>,
    /// The matched crosswalks' own polylines per lane, local metres — what
    /// each footprint was buffered from.
    pub lines: HashMap<String, Vec<Vec<Point>>>,
    /// Crosswalks (first way id) running through a pedestrian zone's
    /// junction node with none of that junction's crossing lanes even
    /// nearby — left out, and reported.
    pub without_lane: Vec<i64>,
    /// Lane ids with no OSM crosswalk within tolerance at all.
    pub unmatched: Vec<String>,
    /// Lane ids whose best and second-best OSM candidate were too close to
    /// call.
    pub ambiguous: Vec<String>,
}

/// (first point, last point) of a SUMO lane's or an OSM way's own polyline
/// — bearing and rough midpoint both only ever need the two ends, since
/// every crossing this crate has seen (SUMO's own two-point crossing lanes,
/// and every real OSM crosswalk checked in calibration) is close enough to
/// straight that its ends already characterise it.
fn ends(points: &[Point]) -> Option<(Point, Point)> {
    match (points.first(), points.last()) {
        (Some(&a), Some(&b)) => Some((a, b)),
        _ => None,
    }
}

fn bearing_degrees(a: Point, b: Point) -> f64 {
    (b.y - a.y).atan2(b.x - a.x).to_degrees()
}

/// The undirected difference between two bearings — 0° for parallel or
/// anti-parallel lines, since a crossing walked from either side is the
/// same physical crossing.
fn angle_difference(a: f64, b: f64) -> f64 {
    let raw = (a - b).rem_euclid(180.0);
    raw.min(180.0 - raw)
}

fn distance(a: (f64, f64), b: (f64, f64)) -> f64 {
    (a.0 - b.0).hypot(a.1 - b.1)
}

fn point_to_segment_distance(p: (f64, f64), a: (f64, f64), b: (f64, f64)) -> f64 {
    let (dx, dy) = (b.0 - a.0, b.1 - a.1);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 {
        (((p.0 - a.0) * dx + (p.1 - a.1) * dy) / len2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    distance(p, (a.0 + t * dx, a.1 + t * dy))
}

fn point_to_polyline_distance(p: (f64, f64), points: &[Point]) -> f64 {
    points
        .windows(2)
        .map(|pair| point_to_segment_distance(p, (pair[0].x, pair[0].y), (pair[1].x, pair[1].y)))
        .fold(f64::INFINITY, f64::min)
}

fn midpoint(a: Point, b: Point) -> (f64, f64) {
    ((a.x + b.x) / 2.0, (a.y + b.y) / 2.0)
}

/// The bearing of the segment of `points` nearest `target` — a long or
/// bent OSM way (a two-stage crossing drawn as one way, confirmed on real
/// Barcelona data at 17-22m) has an end-to-end bearing that says little
/// about the stretch actually beside a given SUMO lane.
fn local_bearing(points: &[Point], target: (f64, f64)) -> Option<f64> {
    points
        .windows(2)
        .min_by(|a, b| {
            let d = |pair: &[Point]| {
                point_to_segment_distance(target, (pair[0].x, pair[0].y), (pair[1].x, pair[1].y))
            };
            d(a).total_cmp(&d(b))
        })
        .map(|pair| bearing_degrees(pair[0], pair[1]))
}

/// How well `osm` (already reprojected to local metres) matches `lane` —
/// `None` when either line degenerates to a single point, or they are too
/// far apart or not roughly parallel where they meet.
fn score(lane_points: &[Point], osm_points: &[Point], max_angle: f64, max_distance: f64) -> Option<f64> {
    let (lane_a, lane_b) = ends(lane_points)?;
    let (osm_a, osm_b) = ends(osm_points)?;
    let lane_mid = midpoint(lane_a, lane_b);
    let osm_bearing = local_bearing(osm_points, lane_mid)?;
    if angle_difference(bearing_degrees(lane_a, lane_b), osm_bearing) > max_angle {
        return None;
    }
    let osm_mid = midpoint(osm_a, osm_b);
    let d = point_to_polyline_distance(lane_mid, osm_points)
        .min(point_to_polyline_distance(osm_mid, lane_points));
    (d <= max_distance).then_some(d)
}

/// Metres between the samples `partition_to_nearest_lane` walks a
/// crosswalk at.
const PARTITION_STEP_METERS: f64 = 0.5;

/// The part of `osm_points` nearer to `own` than to any lane in `others`:
/// a crosswalk several lanes claim (one OSM way over a crossing SUMO split
/// in two) is shared out by proximity, so the parts together cover all of
/// it. Clipping each to its own lane's span instead left the ground between
/// two lanes to neither (confirmed on real Barcelona data, OSM way
/// 1504493414 across `2119385411` and `2119385421`: 3m uncovered). The
/// longest contiguous run is kept.
fn partition_to_nearest_lane(own: &[Point], others: &[&[Point]], osm_points: &[Point]) -> Vec<Point> {
    let mut samples: Vec<Point> = Vec::new();
    for pair in osm_points.windows(2) {
        let length = (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y);
        let steps = (length / PARTITION_STEP_METERS).ceil().max(1.0) as usize;
        for i in 0..steps {
            let t = i as f64 / steps as f64;
            samples.push(Point {
                x: pair[0].x + (pair[1].x - pair[0].x) * t,
                y: pair[0].y + (pair[1].y - pair[0].y) * t,
                z: 0.0,
            });
        }
    }
    if let Some(&last) = osm_points.last() {
        samples.push(last);
    }
    let mine = |p: &Point| {
        let d = point_to_polyline_distance((p.x, p.y), own);
        others.iter().all(|other| d <= point_to_polyline_distance((p.x, p.y), other))
    };
    let (mut best, mut run): (Vec<Point>, Vec<Point>) = (Vec::new(), Vec::new());
    for p in samples {
        if mine(&p) {
            run.push(p);
        } else if !run.is_empty() {
            if polyline_length(&run) > polyline_length(&best) {
                best = std::mem::take(&mut run);
            }
            run.clear();
        }
    }
    if polyline_length(&run) > polyline_length(&best) {
        best = run;
    }
    best
}

fn polyline_length(points: &[Point]) -> f64 {
    points
        .windows(2)
        .map(|pair| (pair[1].x - pair[0].x).hypot(pair[1].y - pair[0].y))
        .sum()
}

/// Every pedestrian zone's own crossing lane, matched against `crosswalks`
/// — the real OSM `footway=crossing` ways beside `network`. `reproject`
/// turns each crosswalk's lon/lat into the same local metres every lane
/// shape is already in.
pub fn match_crosswalks(
    network: &Network,
    zones: &[E3Detector],
    reproject: &Reprojector,
    crosswalks: &[OsmCrosswalk],
) -> CrosswalkMatches {
    let lanes: HashMap<&str, &Lane> = network
        .edges
        .iter()
        .flat_map(|edge| {
            edge.lanes
                .iter()
                .map(move |lane| (lane.id.0.as_str(), (edge, lane)))
        })
        .filter(|(_, (edge, _))| edge.function == EdgeFunction::Crossing)
        .map(|(id, (_, lane))| (id, lane))
        .collect();

    let mut crossing_lane_ids: Vec<&str> = zones
        .iter()
        .flat_map(|zone| &zone.entries)
        .map(|entry| entry.lane.0.as_str())
        .filter(|id| lanes.contains_key(id))
        .collect();
    crossing_lane_ids.sort_unstable();
    crossing_lane_ids.dedup();

    let local_crosswalks: Vec<(i64, &[i64], Vec<Point>)> = crosswalks
        .iter()
        .filter_map(|crosswalk| {
            let points: Vec<Point> = crosswalk
                .points
                .iter()
                .filter_map(|&lon_lat| reproject.to_local(lon_lat).ok())
                .collect();
            (points.len() >= 2).then_some((crosswalk.way_id, crosswalk.nodes.as_slice(), points))
        })
        .collect();

    let mut footprints = HashMap::new();
    let mut matched_ways = HashMap::new();
    let mut lines: HashMap<String, Vec<Vec<Point>>> = HashMap::new();
    let mut unmatched = Vec::new();
    let mut ambiguous = Vec::new();

    // Which lane each OSM way is the *best* match for, so a way claimed by
    // more than one lane is split between them instead of handed whole to
    // each — see `partition_to_nearest_lane`'s own docs.
    let mut claims: HashMap<i64, Vec<&str>> = HashMap::new();
    let mut best_by_lane: HashMap<&str, (usize, f64)> = HashMap::new();

    // A crosswalk through a junction's node is that junction's crossing —
    // but of *which* of its crossing lanes? Only the best-fitting ones (and
    // any within `AMBIGUOUS_MARGIN_METERS` of it): every lane of the
    // junction being within tolerance of it doesn't make it theirs too.
    // Confirmed on real Barcelona data (`5588597501`): four crossing lanes,
    // one on another arm, all claimed OSM way 1504493411.
    let topological = |lane_id: &str, c: usize| {
        let (_, nodes, points) = &local_crosswalks[c];
        let junction = lane_junction(lane_id)?;
        if !nodes.contains(&junction) {
            return None;
        }
        score(
            &lanes[lane_id].shape.0,
            points,
            TOPOLOGICAL_MATCH_ANGLE_DEGREES,
            TOPOLOGICAL_MATCH_DISTANCE_METERS,
        )
    };
    let best_topological: HashMap<usize, f64> = (0..local_crosswalks.len())
        .filter_map(|c| {
            crossing_lane_ids
                .iter()
                .filter_map(|lane_id| topological(lane_id, c))
                .reduce(f64::min)
                .map(|best| (c, best))
        })
        .collect();

    for &lane_id in &crossing_lane_ids {
        let lane_points = &lanes[lane_id].shape.0;
        // A crosswalk through the junction's own node is this junction's
        // crossing: when there is one this lane fits (about) best, only
        // those compete.
        let through_junction: Vec<(usize, f64)> = (0..local_crosswalks.len())
            .filter_map(|c| topological(lane_id, c).map(|d| (c, d)))
            .filter(|&(c, d)| d <= best_topological[&c] + AMBIGUOUS_MARGIN_METERS)
            .collect();
        let mut scored: Vec<(usize, f64)> = if through_junction.is_empty() {
            local_crosswalks
                .iter()
                .enumerate()
                .filter_map(|(i, (_, _, points))| score(lane_points, points, MATCH_ANGLE_DEGREES, MATCH_DISTANCE_METERS).map(|d| (i, d)))
                .collect()
        } else {
            through_junction
        };
        scored.sort_by(|a, b| a.1.total_cmp(&b.1));

        match scored.first() {
            None => unmatched.push(lane_id.to_string()),
            Some(&(best_index, best_distance)) => {
                if let Some(&(_, runner_up)) = scored.get(1)
                    && runner_up - best_distance <= AMBIGUOUS_MARGIN_METERS
                {
                    ambiguous.push(lane_id.to_string());
                    continue;
                }
                best_by_lane.insert(lane_id, (best_index, best_distance));
                claims
                    .entry(local_crosswalks[best_index].0)
                    .or_default()
                    .push(lane_id);
            }
        }
    }

    for (&lane_id, &(way_index, _)) in &best_by_lane {
        let lane = lanes[lane_id];
        let (way_id, _, osm_points) = &local_crosswalks[way_index];
        let claimants = &claims[way_id];
        // Split between claimants only when they are different zones' lanes:
        // two crossing lanes of one zone (a two-stage crossing SUMO drew as
        // `c0` + `c1`, both the same zone's entries) both get the whole
        // crosswalk, since it all lands in that one zone anyway.
        let zones_of = |lane: &str| -> Vec<&str> {
            zones
                .iter()
                .filter(|z| z.entries.iter().any(|gate| gate.lane.0 == lane))
                .map(|z| z.id.0.as_str())
                .collect()
        };
        let one_zone = claimants.iter().all(|other| zones_of(other) == zones_of(lane_id));
        let matched_points = if claimants.len() > 1 && !one_zone {
            let others: Vec<&[Point]> = claimants
                .iter()
                .filter(|&&other| other != lane_id)
                .map(|&other| lanes[other].shape.0.as_slice())
                .collect();
            partition_to_nearest_lane(&lane.shape.0, &others, osm_points)
        } else {
            osm_points.clone()
        };
        if matched_points.len() < 2 || polyline_length(&matched_points) < MIN_MATCHED_SPAN_METERS {
            // Lost to the other lanes claiming the same crosswalk: reported,
            // not dropped silently.
            ambiguous.push(lane_id.to_string());
            continue;
        }
        let footprint = buffer_shape(
            &Shape(matched_points.clone()),
            Length::new::<meter>(0.0),
            Length::new::<meter>(polyline_length(osm_points)),
            lane.width / 2.0,
            LineCap::Butt,
        );
        if !footprint.0.is_empty() {
            footprints.insert(lane_id.to_string(), footprint);
            matched_ways.entry(lane_id.to_string()).or_insert_with(Vec::new).push(*way_id);
            lines.entry(lane_id.to_string()).or_default().push(matched_points);
        }
    }

    // A crosswalk through a junction's node that fits none of its crossing
    // lanes by angle is still that junction's: it goes, as an extra
    // footprint, to the junction's nearest crossing lane. Confirmed on real
    // Barcelona data (`5588790790`): OSM way 683963543 runs diagonally
    // through the junction's centre, 58°+ off both crossing lanes, and
    // showed half outside the zone owning them both.
    let matched: std::collections::HashSet<i64> = matched_ways.values().flatten().copied().collect();
    let mut without_lane = Vec::new();
    for (c, (way_id, nodes, points)) in local_crosswalks.iter().enumerate() {
        if best_topological.contains_key(&c) || matched.contains(way_id) {
            continue;
        }
        let nearest = crossing_lane_ids
            .iter()
            .filter(|lane_id| lane_junction(lane_id).is_some_and(|j| nodes.contains(&j)))
            .filter_map(|&lane_id| {
                let (a, b) = ends(&lanes[lane_id].shape.0)?;
                let d = point_to_polyline_distance(midpoint(a, b), points);
                (d <= TOPOLOGICAL_MATCH_DISTANCE_METERS).then_some((lane_id, d))
            })
            .min_by(|x, y| x.1.total_cmp(&y.1));
        let through_a_zone_junction = crossing_lane_ids
            .iter()
            .any(|lane_id| lane_junction(lane_id).is_some_and(|j| nodes.contains(&j)));
        match nearest {
            Some((lane_id, _)) => {
                let lane = lanes[lane_id];
                let extra = buffer_shape(
                    &Shape(points.clone()),
                    Length::new::<meter>(0.0),
                    Length::new::<meter>(polyline_length(points)),
                    lane.width / 2.0,
                    LineCap::Butt,
                );
                let footprint = footprints.entry(lane_id.to_string()).or_insert_with(|| MultiPolygon::new(Vec::new()));
                *footprint = footprint.union(&extra);
                matched_ways.entry(lane_id.to_string()).or_insert_with(Vec::new).push(*way_id);
                lines.entry(lane_id.to_string()).or_default().push(points.clone());
            }
            None if through_a_zone_junction => without_lane.push(*way_id),
            None => {}
        }
    }
    without_lane.sort_unstable();

    CrosswalkMatches {
        footprints,
        matched_ways,
        lines,
        without_lane,
        unmatched,
        ambiguous,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sumo_types::domain::{Boundary, Location, Projection};

    fn point(x: f64, y: f64) -> Point {
        Point { x, y, z: 0.0 }
    }

    /// Bearing/distance scoring in bare local coordinates, no reprojection
    /// involved — every test below builds its own lane/crosswalk points
    /// directly rather than round-tripping through lon/lat.
    #[test]
    fn a_crossing_lanes_junction_is_read_from_its_id() {
        assert_eq!(lane_junction(":302182247_c0_0"), Some(302182247));
        assert_eq!(lane_junction(":cluster_1_2_c0_0"), None);
        assert_eq!(lane_junction("302182247_0"), None);
    }

    #[test]
    fn a_parallel_close_crosswalk_matches() {
        let lane = vec![point(0.0, 0.0), point(0.0, 5.0)];
        let osm = vec![point(0.5, -1.0), point(0.5, 6.0)];
        assert!(score(&lane, &osm, MATCH_ANGLE_DEGREES, MATCH_DISTANCE_METERS).is_some());
    }

    #[test]
    fn a_perpendicular_crosswalk_is_rejected() {
        let lane = vec![point(0.0, 0.0), point(0.0, 5.0)];
        let osm = vec![point(-3.0, 2.5), point(3.0, 2.5)];
        assert_eq!(score(&lane, &osm, MATCH_ANGLE_DEGREES, MATCH_DISTANCE_METERS), None);
    }

    #[test]
    fn an_8m_away_crosswalk_is_rejected() {
        let lane = vec![point(0.0, 0.0), point(0.0, 5.0)];
        let osm = vec![point(8.0, -1.0), point(8.0, 6.0)];
        assert_eq!(score(&lane, &osm, MATCH_ANGLE_DEGREES, MATCH_DISTANCE_METERS), None);
    }

    #[test]
    fn a_way_spanning_two_lanes_is_shared_out_without_a_gap() {
        // Two crossing lanes on the same axis (a refuge island between
        // them), both close to one long OSM way that spans both — the "OSM
        // simplified a two-stage crossing into one way" case.
        let lane_a = vec![point(0.0, 0.0), point(0.0, 5.0)];
        let lane_b = vec![point(0.0, 20.0), point(0.0, 25.0)];
        let osm = vec![point(0.3, -1.0), point(0.3, 26.0)];

        let part_a = partition_to_nearest_lane(&lane_a, &[&lane_b], &osm);
        let part_b = partition_to_nearest_lane(&lane_b, &[&lane_a], &osm);
        // Each part sits by its own lane...
        assert!(part_a.iter().all(|p| p.y <= 12.6));
        assert!(part_b.iter().all(|p| p.y >= 12.4));
        // ...and together they cover the whole crosswalk.
        let covered = polyline_length(&part_a) + polyline_length(&part_b);
        assert!((covered - polyline_length(&osm)).abs() < 1.0, "{covered}");
    }

    fn barcelona_reprojector() -> Reprojector {
        let location = Location {
            net_offset: point(-435_316.02, -4_587_129.41),
            converted_boundary: Boundary {
                min: Point::default(),
                max: Point::default(),
            },
            original_boundary: Boundary {
                min: Point::default(),
                max: Point::default(),
            },
            projection: Projection::Proj4(
                "+proj=utm +zone=31 +ellps=WGS84 +datum=WGS84 +units=m +no_defs".to_string(),
            ),
        };
        Reprojector::new(&location).expect("a valid PROJ4 string")
    }

    #[test]
    fn ambiguous_when_two_candidates_are_equally_close() {
        let reproject = barcelona_reprojector();
        let lane_local = vec![point(1000.0, 1000.0), point(1000.0, 1005.0)];
        let osm_a = OsmCrosswalk {
            way_id: 1,
            nodes: Vec::new(),
            signalized: false,
            points: [point(1000.4, 999.0), point(1000.4, 1006.0)]
                .into_iter()
                .map(|p| reproject.to_lon_lat(p).unwrap())
                .collect(),
        };
        let osm_b = OsmCrosswalk {
            way_id: 2,
            nodes: Vec::new(),
            signalized: false,
            points: [point(1000.5, 999.0), point(1000.5, 1006.0)]
                .into_iter()
                .map(|p| reproject.to_lon_lat(p).unwrap())
                .collect(),
        };
        let local_crosswalks = [
            (
                osm_a.way_id,
                osm_a
                    .points
                    .iter()
                    .map(|&ll| reproject.to_local(ll).unwrap())
                    .collect::<Vec<_>>(),
            ),
            (
                osm_b.way_id,
                osm_b
                    .points
                    .iter()
                    .map(|&ll| reproject.to_local(ll).unwrap())
                    .collect::<Vec<_>>(),
            ),
        ];
        let mut scored: Vec<f64> = local_crosswalks
            .iter()
            .filter_map(|(_, pts)| score(&lane_local, pts, MATCH_ANGLE_DEGREES, MATCH_DISTANCE_METERS))
            .collect();
        scored.sort_by(f64::total_cmp);
        assert_eq!(scored.len(), 2);
        assert!(scored[1] - scored[0] <= AMBIGUOUS_MARGIN_METERS);
    }
}

//! Pedestrian waiting zones: the crosswalk itself — SUMO's own crossing
//! lane, plus every real surveyed OSM crosswalk that belongs to the zone —
//! each carried a short way onto the sidewalk at both ends.
//!
//! Two zones meeting at a corner split the sidewalk ground they'd share
//! along the line equidistant from their two crosswalks. A crosswalk itself
//! is never cut: a zone always keeps all of its own crosswalks, so the only
//! ground two zones can still share is where two crosswalks physically
//! overlap.

use std::collections::{HashMap, HashSet};

use anyhow::Result;
use geo::{Area, BooleanOps, Coord, Distance, LineString, MultiPolygon, Polygon};
use geojson::FeatureCollection;
use i_overlay::mesh::style::LineCap;
use osm_crosswalks::OsmCrosswalk;
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{EdgeFunction, Lane, Network, Point, Shape};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

use crate::geojson_output::crosswalks::match_crosswalks;
use crate::geojson_output::feature::{build_feature_at, feature_from_parts, lane_to_junction};
use crate::geojson_output::geometry::{buffer_shape, centroid, pedestrian_lane_polygon};
use crate::geojson_output::overlaps::{drop_interior_rings, drop_slivers, split_self_intersections};
use crate::geojson_output::reprojection::Reprojector;

/// How far past each end of the crosswalk a zone reaches onto the
/// sidewalk: where people actually stand waiting for green.
pub const SIDEWALK_OFFSET_METERS: f64 = 2.0;
/// A real crosswalk the matcher didn't pair with any crossing lane still
/// belongs to the nearest zone whose ground is this close to it: a zone is
/// meant to contain every real crosswalk this near it.
pub const NEAREST_CROSSWALK_METERS: f64 = 10.0;
/// Absorbing a nearby crosswalk grows a zone, which can bring another
/// within reach; bounded so a chain of crosswalks can't grow one forever.
const MAX_ABSORB_ROUNDS: usize = 10;
/// How far inside the crosswalk, from its near end, the published stop line
/// sits — on the zone's own crosswalk, which no neighbour can take.
const STOP_LINE_INSET_METERS: f64 = 0.5;
/// Width given to a crossing with no SUMO lane to take it from: a zebra
/// and the margin people stand in.
const DEFAULT_CROSSWALK_WIDTH_METERS: f64 = 4.0;
/// Shared ground below this is numerical noise, not a corner collision.
const COLLISION_AREA_M2: f64 = 0.01;

struct PedestrianShape {
    polygon: MultiPolygon<f64>,
    /// The zone's own crosswalk stripes, without the sidewalk offset.
    core: MultiPolygon<f64>,
    /// Every crosswalk line (SUMO lanes and the zone's OSM crosswalks).
    lines: Vec<Vec<Point>>,
    stop_line: Point,
    /// The bank the zone waits on, to join crosswalks on different arms.
    bank: MultiPolygon<f64>,
}

pub fn feature_collection(
    network: &Network,
    zones: &[E3Detector],
    crosswalks: &[OsmCrosswalk],
) -> Result<FeatureCollection> {
    let reproject = Reprojector::new(&network.location)?;
    let lanes: HashMap<&str, &Lane> = network
        .edges
        .iter()
        .flat_map(|edge| &edge.lanes)
        .map(|lane| (lane.id.0.as_str(), lane))
        .collect();
    let crossing_lanes: HashSet<&str> = network
        .edges
        .iter()
        .filter(|edge| edge.function == EdgeFunction::Crossing)
        .flat_map(|edge| edge.lanes.iter().map(|lane| lane.id.0.as_str()))
        .collect();
    let junction_ids: HashSet<&str> = network.junctions.iter().map(|j| j.id.0.as_str()).collect();
    let lane_to_junction = lane_to_junction(network);

    let crossings_of: Vec<Vec<&Lane>> = zones
        .iter()
        .map(|zone| {
            let mut crossings: Vec<&Lane> = zone
                .entries
                .iter()
                .filter(|gate| crossing_lanes.contains(gate.lane.0.as_str()))
                .filter_map(|gate| lanes.get(gate.lane.0.as_str()).copied())
                .collect();
            crossings.dedup_by(|a, b| a.id == b.id);
            crossings
        })
        .collect();
    let assigned = assign_crosswalks(network, zones, &reproject, crosswalks, &crossings_of);

    // Every zone this collection emits: SUMO's own (one per `zones` entry,
    // in order), then one per signalized OSM crossing SUMO gave no crossing
    // lane — a real crossing with its own light is its own waiting zone,
    // never folded into a neighbour's.
    let orphans: Vec<OsmZone> = assigned
        .orphans
        .into_iter()
        .map(|(crosswalk, line)| OsmZone {
            id: format!("osm{}_ped", crosswalk.way_id),
            intersection: crosswalk
                .nodes
                .iter()
                .map(|n| n.to_string())
                .find(|n| junction_ids.contains(n.as_str())),
            line,
        })
        .collect();
    let mut owned = assigned.owned;
    owned.resize(zones.len() + orphans.len(), Vec::new());

    let build = |owned: &[Vec<Vec<Point>>]| -> Vec<PedestrianShape> {
        zones
            .iter()
            .zip(&crossings_of)
            .zip(owned)
            .map(|((zone, crossings), osm)| shape_of(zone, &lanes, crossings, osm.clone()))
            .chain(orphans.iter().zip(&owned[zones.len()..]).map(|(orphan, osm)| orphan_shape(orphan, osm.clone())))
            .collect()
    };
    // An unsignalized crosswalk no zone claimed that still lies within
    // `NEAREST_CROSSWALK_METERS` of a zone's ground (its bank, most often)
    // is that zone's too; taking it in grows the zone, so repeat until
    // nothing more is that near.
    let mut unowned = assigned.unowned;
    let mut shapes = build(&owned);
    let mut rounds = 0;
    for _ in 0..MAX_ABSORB_ROUNDS {
        let mut absorbed = false;
        unowned.retain(|line| {
            let as_line = LineString::new(line.iter().map(|p| Coord { x: p.x, y: p.y }).collect());
            let nearest = shapes
                .iter()
                .enumerate()
                .filter(|(_, shape)| !shape.polygon.0.is_empty())
                .map(|(z, shape)| (z, geo::Euclidean.distance(&as_line, &shape.polygon)))
                .filter(|&(_, d)| d <= NEAREST_CROSSWALK_METERS)
                .min_by(|a, b| a.1.total_cmp(&b.1));
            match nearest {
                Some((z, _)) => {
                    owned[z].push(line.clone());
                    absorbed = true;
                    false
                }
                None => true,
            }
        });
        if !absorbed {
            break;
        }
        rounds += 1;
        shapes = build(&owned);
    }
    if !unowned.is_empty() && rounds == MAX_ABSORB_ROUNDS {
        anstream::eprintln!("warning: still absorbing nearby crosswalks after {MAX_ABSORB_ROUNDS} rounds");
    }
    resolve_corner_collisions(&mut shapes);

    let mut features = Vec::with_capacity(shapes.len());
    for (index, shape) in shapes.iter().enumerate() {
        if shape.polygon.0.is_empty() {
            let id = zones.get(index).map_or_else(|| orphans[index - zones.len()].id.clone(), |z| z.id.0.clone());
            anstream::eprintln!("warning: pedestrian zone {id:?} has no ground to draw; omitted from the GeoJSON output");
            continue;
        }
        features.push(match zones.get(index) {
            Some(zone) => build_feature_at(zone, &lanes, &lane_to_junction, &shape.polygon, &reproject, shape.stop_line)?,
            None => {
                let orphan = &orphans[index - zones.len()];
                feature_from_parts(
                    &orphan.id,
                    orphan.intersection.as_deref(),
                    serde_json::json!(["ON_FOOT"]),
                    &shape.polygon,
                    &reproject,
                    shape.stop_line,
                )?
            }
        });
    }
    Ok(FeatureCollection { bbox: None, features, foreign_members: None })
}

/// A pedestrian zone for a signalized OSM crossing SUMO has no crossing
/// lane for: built from the surveyed crossing alone.
struct OsmZone {
    id: String,
    /// The SUMO junction the crossing runs through, when it runs through one.
    intersection: Option<String>,
    line: Vec<Point>,
}

/// How OSM's crosswalks were shared out between the zones.
struct Assigned<'a> {
    /// Per SUMO zone, the whole crosswalks it contains.
    owned: Vec<Vec<Vec<Point>>>,
    /// Signalized crosswalks no SUMO zone owns: each becomes a zone of its own.
    orphans: Vec<(&'a OsmCrosswalk, Vec<Point>)>,
    /// Unsignalized crosswalks no zone owns (yet).
    unowned: Vec<Vec<Point>>,
}

/// Which real OSM crosswalks each zone contains, whole: every crosswalk the
/// matcher paired with one of a zone's crossing lanes goes to that zone.
/// Of the rest, a signalized one is a zone of its own; an unsignalized one
/// goes to the zone whose own crossing is nearest, within
/// [`NEAREST_CROSSWALK_METERS`].
fn assign_crosswalks<'a>(
    network: &Network,
    zones: &[E3Detector],
    reproject: &Reprojector,
    crosswalks: &'a [OsmCrosswalk],
    crossings_of: &[Vec<&Lane>],
) -> Assigned<'a> {
    let matches = match_crosswalks(network, zones, reproject, crosswalks);
    let mut assigned = Assigned { owned: vec![Vec::new(); zones.len()], orphans: Vec::new(), unowned: Vec::new() };
    for crosswalk in crosswalks {
        let line: Vec<Point> = crosswalk.points.iter().filter_map(|&ll| reproject.to_local(ll).ok()).collect();
        if line.len() < 2 {
            continue;
        }
        let paired: Vec<usize> = (0..zones.len())
            .filter(|&z| {
                crossings_of[z].iter().any(|lane| {
                    matches.matched_ways.get(&lane.id.0).is_some_and(|ways| ways.contains(&crosswalk.way_id))
                })
            })
            .collect();
        if !paired.is_empty() {
            for z in paired {
                assigned.owned[z].push(line.clone());
            }
            continue;
        }
        if crosswalk.signalized {
            assigned.orphans.push((crosswalk, line));
            continue;
        }
        let nearest = (0..zones.len())
            .filter_map(|z| {
                let d = crossings_of[z]
                    .iter()
                    .map(|lane| lines_distance(&line, &lane.shape.0))
                    .fold(f64::INFINITY, f64::min);
                (d <= NEAREST_CROSSWALK_METERS).then_some((z, d))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1));
        match nearest {
            Some((z, _)) => assigned.owned[z].push(line),
            None => assigned.unowned.push(line),
        }
    }
    assigned
}

fn segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let (dx, dy) = (b.x - a.x, b.y - a.y);
    let len2 = dx * dx + dy * dy;
    let t = if len2 > 0.0 { (((p.x - a.x) * dx + (p.y - a.y) * dy) / len2).clamp(0.0, 1.0) } else { 0.0 };
    (p.x - a.x - t * dx).hypot(p.y - a.y - t * dy)
}

/// The shortest distance between two polylines (0 when they cross).
fn lines_distance(a: &[Point], b: &[Point]) -> f64 {
    let to = |p: Point, line: &[Point]| {
        line.windows(2).map(|w| segment_distance(p, w[0], w[1])).fold(f64::INFINITY, f64::min)
    };
    let as_line = |points: &[Point]| LineString::new(points.iter().map(|p| Coord { x: p.x, y: p.y }).collect());
    if geo::Intersects::intersects(&as_line(a), &as_line(b)) {
        return 0.0;
    }
    a.iter().map(|&p| to(p, b)).chain(b.iter().map(|&p| to(p, a))).fold(f64::INFINITY, f64::min)
}

/// `line` carried `offset` metres further along its own direction at both
/// ends.
fn extended(line: &[Point], offset: f64) -> Vec<Point> {
    let mut out = line.to_vec();
    let push_out = |from: Point, to: Point| {
        let (dx, dy) = (to.x - from.x, to.y - from.y);
        let n = dx.hypot(dy);
        if n <= 0.0 {
            return to;
        }
        Point { x: to.x + dx / n * offset, y: to.y + dy / n * offset, z: to.z }
    };
    if let [first, second, ..] = line {
        out[0] = push_out(*second, *first);
    }
    if let [.., before, last] = line {
        let end = out.len() - 1;
        out[end] = push_out(*before, *last);
    }
    out
}

fn stripe(line: &[Point], width: Length) -> MultiPolygon<f64> {
    let length: f64 = line.windows(2).map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y)).sum();
    buffer_shape(
        &Shape(line.to_vec()),
        Length::new::<meter>(0.0),
        Length::new::<meter>(length),
        width / 2.0,
        LineCap::Butt,
    )
}

fn shape_of(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    crossings: &[&Lane],
    osm_lines: Vec<Vec<Point>>,
) -> PedestrianShape {
    let zero = Length::new::<meter>(0.0);
    let bank_lane = zone.exits.first().and_then(|gate| lanes.get(gate.lane.0.as_str())).copied();
    let bank = bank_lane.map_or_else(|| MultiPolygon::new(Vec::new()), |lane| pedestrian_lane_polygon(lane, zero, zero));
    let bank_centre = bank_lane.map(|lane| centroid(&lane.shape.0));

    let mut lines: Vec<Vec<Point>> =
        crossings.iter().map(|lane| lane.shape.0.clone()).filter(|l| l.len() >= 2).collect();
    lines.extend(osm_lines.into_iter().filter(|l| l.len() >= 2));
    // A zone naming no crossing lane and owning no crosswalk: its bank is
    // all there is.
    if lines.is_empty() {
        return PedestrianShape {
            polygon: bank.clone(),
            core: MultiPolygon::new(Vec::new()),
            lines,
            stop_line: bank_centre.unwrap_or_default(),
            bank,
        };
    }
    let width = crossings.first().map_or_else(|| Length::new::<meter>(DEFAULT_CROSSWALK_WIDTH_METERS), |lane| lane.width);
    let (mut polygon, core) = stripes(&lines, width);
    // Crossings on different arms of the junction: joined by the bank
    // they're all reached from.
    if polygon.0.len() > 1 {
        polygon = polygon.union(&bank);
    }

    // The stop line: the crossing end nearest the bank this zone waits on,
    // just inside the crossing.
    let own = if crossings.is_empty() { &lines[..] } else { &lines[..crossings.len().min(lines.len())] };
    let stop_line = stop_line_near(own, bank_centre);
    PedestrianShape { polygon, core, lines, stop_line, bank }
}

fn orphan_shape(orphan: &OsmZone, absorbed: Vec<Vec<Point>>) -> PedestrianShape {
    let mut lines = vec![orphan.line.clone()];
    lines.extend(absorbed.into_iter().filter(|l| l.len() >= 2));
    let (polygon, core) = stripes(&lines, Length::new::<meter>(DEFAULT_CROSSWALK_WIDTH_METERS));
    let stop_line = stop_line_near(&lines[..1], None);
    PedestrianShape { polygon, core, lines, stop_line, bank: MultiPolygon::new(Vec::new()) }
}

/// Every line's stripe `width` wide, carried [`SIDEWALK_OFFSET_METERS`]
/// onto the sidewalk (the zone), and without that offset (its core).
fn stripes(lines: &[Vec<Point>], width: Length) -> (MultiPolygon<f64>, MultiPolygon<f64>) {
    let mut polygon = MultiPolygon::new(Vec::new());
    let mut core = MultiPolygon::new(Vec::new());
    for line in lines {
        polygon = polygon.union(&stripe(&extended(line, SIDEWALK_OFFSET_METERS), width));
        core = core.union(&stripe(line, width));
    }
    (polygon, core)
}

/// Just inside the end of `lines` nearest `bank` (or the first line's first
/// end, with no bank to go by).
fn stop_line_near(lines: &[Vec<Point>], bank: Option<Point>) -> Point {
    let ends = lines.iter().filter(|l| l.len() >= 2).flat_map(|s| [(s[0], s[1]), (s[s.len() - 1], s[s.len() - 2])]);
    let pick = match bank {
        Some(bank) => ends.min_by(|(a, _), (b, _)| {
            let d = |p: &Point| (p.x - bank.x).hypot(p.y - bank.y);
            d(a).total_cmp(&d(b))
        }),
        None => ends.into_iter().next(),
    };
    let Some((end, inward)) = pick else { return Point::default() };
    let (dx, dy) = (inward.x - end.x, inward.y - end.y);
    let n = dx.hypot(dy).max(1e-9);
    let inset = STOP_LINE_INSET_METERS.min(n / 2.0);
    Point { x: end.x + dx / n * inset, y: end.y + dy / n * inset, z: 0.0 }
}

type Pt = Coord<f64>;

fn ends(line: &[Point]) -> (Pt, Pt) {
    let (a, b) = (line[0], line[line.len() - 1]);
    (Coord { x: a.x, y: a.y }, Coord { x: b.x, y: b.y })
}

/// The half-plane on `a`'s side of the line equidistant from crosswalks `a`
/// and `b`: the bisector of the angle between them where they meet at a
/// corner, their midline when they run parallel.
fn towards(a: (Pt, Pt), b: (Pt, Pt)) -> MultiPolygon<f64> {
    let unit = |p: Pt, q: Pt| {
        let (dx, dy) = (q.x - p.x, q.y - p.y);
        let n = dx.hypot(dy).max(1e-12);
        Coord { x: dx / n, y: dy / n }
    };
    let (ua, ub) = (unit(a.0, a.1), unit(b.0, b.1));
    let mid = |s: (Pt, Pt)| Coord { x: (s.0.x + s.1.x) / 2.0, y: (s.0.y + s.1.y) / 2.0 };
    let (ma, mb) = (mid(a), mid(b));
    let side = |origin: Pt, normal: Pt, p: Pt| (p.x - origin.x) * normal.x + (p.y - origin.y) * normal.y;

    let cross = ua.x * ub.y - ua.y * ub.x;
    let mut split: Option<(Pt, Pt)> = None; // (origin, normal pointing at `a`)
    if cross.abs() > 0.05 {
        // Where the two crosswalk lines meet, and the one of the two angle
        // bisectors there that separates the crosswalks' own midpoints.
        let (wx, wy) = (b.0.x - a.0.x, b.0.y - a.0.y);
        let t = (wx * ub.y - wy * ub.x) / cross;
        let origin = Coord { x: a.0.x + ua.x * t, y: a.0.y + ua.y * t };
        for dir in [Coord { x: ua.x + ub.x, y: ua.y + ub.y }, Coord { x: ua.x - ub.x, y: ua.y - ub.y }] {
            let normal = Coord { x: -dir.y, y: dir.x };
            let (sa, sb) = (side(origin, normal, ma), side(origin, normal, mb));
            if sa * sb < 0.0 {
                let sign = sa.signum();
                split = Some((origin, Coord { x: normal.x * sign, y: normal.y * sign }));
                break;
            }
        }
    }
    let (origin, normal) = split.unwrap_or_else(|| {
        // Parallel (or no separating bisector): the perpendicular bisector
        // of the two midpoints.
        let origin = Coord { x: (ma.x + mb.x) / 2.0, y: (ma.y + mb.y) / 2.0 };
        (origin, unit(mb, ma))
    });
    let along = Coord { x: -normal.y, y: normal.x };
    let big = 10_000.0;
    let corner = |n: f64, s: f64| Coord {
        x: origin.x + normal.x * n + along.x * s,
        y: origin.y + normal.y * n + along.y * s,
    };
    MultiPolygon::new(vec![Polygon::new(
        LineString::new(vec![corner(0.0, -big), corner(big, -big), corner(big, big), corner(0.0, big), corner(0.0, -big)]),
        Vec::new(),
    )])
}

/// The pair of crosswalk lines, one from each zone, closest to each other —
/// the two that actually meet at the corner the zones collide on.
fn closest_lines(a: &PedestrianShape, b: &PedestrianShape) -> Option<((Pt, Pt), (Pt, Pt))> {
    a.lines
        .iter()
        .flat_map(|la| b.lines.iter().map(move |lb| (la, lb)))
        .min_by(|x, y| lines_distance(x.0, x.1).total_cmp(&lines_distance(y.0, y.1)))
        .map(|(la, lb)| (ends(la), ends(lb)))
}

fn clean(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    drop_interior_rings(drop_slivers(split_self_intersections(polygon)))
}

/// Corner collisions: every pair of zones sharing ground splits it along the
/// line equidistant from their two crosswalks; then every zone gets all of
/// its own crosswalks back, and loses whatever of another zone's crosswalks
/// isn't also its own. All cuts are computed against the original shapes,
/// so the result doesn't depend on the order zones come in.
fn resolve_corner_collisions(shapes: &mut [PedestrianShape]) {
    let n = shapes.len();
    let mut losses: Vec<Vec<MultiPolygon<f64>>> = vec![Vec::new(); n];
    for i in 0..n {
        for j in i + 1..n {
            let shared = shapes[i].polygon.intersection(&shapes[j].polygon);
            if shared.unsigned_area() <= COLLISION_AREA_M2 {
                continue;
            }
            let Some((la, lb)) = closest_lines(&shapes[i], &shapes[j]) else {
                // A zone with no crosswalk (its bank alone): split by the
                // two banks' centres.
                let (ci, cj) = (centroid_of(&shapes[i].polygon), centroid_of(&shapes[j].polygon));
                let side_i = towards((ci, ci), (cj, cj));
                losses[j].push(shared.intersection(&side_i));
                losses[i].push(shared.difference(&side_i));
                continue;
            };
            let side_i = towards(la, lb);
            losses[j].push(shared.intersection(&side_i));
            losses[i].push(shared.difference(&side_i));
        }
    }
    let originals: Vec<MultiPolygon<f64>> = shapes.iter().map(|s| s.polygon.clone()).collect();
    for i in 0..n {
        let mut polygon = originals[i].clone();
        for piece in &losses[i] {
            polygon = polygon.difference(piece);
        }
        for (j, other) in shapes.iter().enumerate() {
            if j != i && !other.core.0.is_empty() {
                polygon = polygon.difference(&other.core.difference(&shapes[i].core));
            }
        }
        polygon = polygon.union(&shapes[i].core);
        // Still in pieces after the cuts (a crosswalk on another arm cut
        // off from the rest): rejoin through the bank.
        if polygon.0.len() > 1 {
            polygon = polygon.union(&shapes[i].bank);
        }
        shapes[i].polygon = clean(polygon);
    }
}

fn centroid_of(polygon: &MultiPolygon<f64>) -> Pt {
    use geo::Centroid;
    polygon.centroid().map_or(Coord { x: 0.0, y: 0.0 }, |p| p.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64) -> Point {
        Point { x, y, z: 0.0 }
    }

    fn shape(line: Vec<Point>) -> PedestrianShape {
        let width = Length::new::<meter>(4.0);
        PedestrianShape {
            polygon: stripe(&extended(&line, SIDEWALK_OFFSET_METERS), width),
            core: stripe(&line, width),
            stop_line: line[0],
            lines: vec![line],
            bank: MultiPolygon::new(Vec::new()),
        }
    }

    #[test]
    fn a_zone_is_its_crosswalk_plus_the_sidewalk_offset() {
        let s = shape(vec![p(0.0, 0.0), p(0.0, 10.0)]);
        // 4m wide, 10m crosswalk + 2m at each end.
        assert!((s.polygon.unsigned_area() - 4.0 * 14.0).abs() < 1e-6);
    }

    #[test]
    fn two_crosswalks_meeting_at_a_corner_split_the_sidewalk_but_keep_their_crosswalks() {
        // Two crossings of an L-shaped corner, their ends 3m apart: the
        // sidewalk offsets overlap, the crosswalks themselves don't.
        let mut shapes = vec![shape(vec![p(0.0, 3.0), p(0.0, 13.0)]), shape(vec![p(3.0, 0.0), p(13.0, 0.0)])];
        let cores: Vec<_> = shapes.iter().map(|s| s.core.clone()).collect();
        resolve_corner_collisions(&mut shapes);
        let shared = shapes[0].polygon.intersection(&shapes[1].polygon).unsigned_area();
        assert!(shared < COLLISION_AREA_M2, "still sharing {shared}m²");
        for (s, core) in shapes.iter().zip(&cores) {
            let lost = core.difference(&s.polygon).unsigned_area();
            assert!(lost < 1e-6, "lost {lost}m² of its own crosswalk");
        }
    }

    #[test]
    fn a_line_crossing_another_is_at_distance_zero() {
        assert_eq!(lines_distance(&[p(0.0, -1.0), p(0.0, 1.0)], &[p(-1.0, 0.0), p(1.0, 0.0)]), 0.0);
        assert!((lines_distance(&[p(0.0, 0.0), p(0.0, 1.0)], &[p(3.0, 0.0), p(3.0, 1.0)]) - 3.0).abs() < 1e-9);
    }
}

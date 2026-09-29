//! Turns generated zones into the two GeoJSON `FeatureCollection`s the rest
//! of Leave consumes (`<path>.vehicles.geojson`, `<path>.pedestrians.geojson`),
//! with the same per-feature properties the SUMO-based generator emits:
//! `waiting_zone_id`, `intersection_id`, `stop_line`, `modes`.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use geo::{Area, BooleanOps, Contains, Coord, LineString, MultiPolygon, Point, Polygon};
use geojson::{Feature, FeatureCollection, Geometry, JsonObject};
use serde::Serialize;

use crate::geometry::{self, Pt};
use crate::network::Mode;
use crate::pedestrians::PedestrianZone;
use crate::projection::Projection;
use crate::zones::{BandEnd, VehicleZone};

/// Width of the strip along a pedestrian zone's own crosswalk the zone must
/// keep whatever its neighbours.
const CROSSWALK_CORE_WIDTH_METERS: f64 = 2.5;
/// A vehicle zone's band end carries on to a pedestrian zone that starts
/// within this far of it.
const CROSSWALK_REACH_METERS: f64 = 3.0;
/// How far past a band end to look for that pedestrian zone's far side.
const CROSSWALK_SEARCH_AHEAD_METERS: f64 = 15.0;
/// One car's footprint, about 2m × 5m: the least a vehicle zone must hold.
const MIN_VEHICLE_ZONE_M2: f64 = 10.0;
/// Overlaps smaller than this are numerical noise, not shared ground.
const OVERLAP_AREA_THRESHOLD_M2: f64 = 0.01;

#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub enum Class {
    Vehicle(Mode),
    Pedestrian,
}

pub struct Zone {
    pub id: String,
    pub class: Class,
    pub polygon: MultiPolygon<f64>,
    pub stop_line: Pt,
    /// The point this zone keeps when an overlap with a same-class
    /// neighbour is split between them.
    pub site: Pt,
    /// Ground along a pedestrian zone's own crosswalk the zone must keep.
    /// Empty for vehicles.
    pub core: MultiPolygon<f64>,
    /// A vehicle zone's lane band ends. Empty for pedestrians.
    pub ends: Vec<BandEnd>,
    /// Ground a vehicle zone must stay out of: the junctions it ends at.
    pub keep_out: MultiPolygon<f64>,
    /// A pedestrian zone's crosswalk lines. Empty for vehicles.
    pub lines: Vec<Vec<Pt>>,
    pub intersection: String,
}

impl Zone {
    pub fn vehicle(zone: VehicleZone, intersection: String) -> Self {
        Zone {
            id: zone.id,
            class: Class::Vehicle(zone.mode),
            polygon: zone.polygon,
            stop_line: zone.stop_line,
            site: zone.stop_line,
            core: MultiPolygon::new(Vec::new()),
            ends: zone.ends,
            keep_out: zone.keep_out,
            lines: Vec::new(),
            intersection,
        }
    }

    pub fn pedestrian(zone: PedestrianZone, intersection: String) -> Self {
        Zone {
            core: zone
                .lines
                .iter()
                .map(|line| geometry::stroke(line, CROSSWALK_CORE_WIDTH_METERS))
                .fold(MultiPolygon::new(Vec::new()), |acc, stripe| {
                    acc.union(&stripe)
                }),
            ends: Vec::new(),
            keep_out: MultiPolygon::new(Vec::new()),
            lines: zone.lines,
            id: zone.id,
            class: Class::Pedestrian,
            polygon: zone.polygon,
            stop_line: zone.stop_line,
            site: zone.site,
            intersection,
        }
    }

    pub fn modes(&self) -> Vec<&'static str> {
        match self.class {
            Class::Vehicle(Mode::Car) => vec!["CAR", "MOTORCYCLE"],
            Class::Pedestrian => vec!["ON_FOOT"],
        }
    }
}

/// Brings vehicle zones up to the pedestrian zones they face, splits
/// same-class overlaps, and drops any zone left with no ground.
pub fn assemble(mut zones: Vec<Zone>) -> Vec<Zone> {
    meet_crosswalks(&mut zones);
    split_overlaps(&mut zones);
    for zone in &mut zones {
        let polygon = std::mem::replace(&mut zone.polygon, MultiPolygon::new(Vec::new()));
        // Needles first: a hole walled in only by a needle is a neighbour's
        // ground, and filling it before the needle goes would take that.
        // Cleaned again after, as filling redraws the outline.
        let cleaned = geometry::clean_rings(polygon);
        zone.polygon = geometry::clean_rings(geometry::without_holes(cleaned));
        // A twist a few centimetres across, just past what cleaning welds,
        // can leave the ring crossing itself: redrawn through a boolean
        // union, which resolves the crossing, and cleaned again.
        if !geo::Validation::is_valid(&zone.polygon) {
            let redrawn = zone.polygon.union(&zone.polygon);
            zone.polygon = geometry::clean_rings(geometry::without_holes(redrawn));
        }
    }
    settle_residual_overlaps(&mut zones);
    // A vehicle zone smaller than one car (the stub of a road between two
    // junctions a few metres apart) holds no one waiting.
    zones.retain(|z| {
        let least = if z.class == Class::Pedestrian {
            1.0
        } else {
            MIN_VEHICLE_ZONE_M2
        };
        z.polygon.unsigned_area() > least
    });
    // Cleaning can take away the needle a stop line was moved next to:
    // whatever is left, the stop line is on the zone's own ground.
    for zone in &mut zones {
        if !contains(&zone.polygon, zone.stop_line)
            && let Some(nearest) = nearest_inside(&zone.polygon, zone.stop_line)
        {
            zone.stop_line = nearest;
        }
    }
    zones
}

/// One junction's controller, shaped like `engine_unit::EngineJunction`
/// (what the engine consumes), plus `links` and `conflicts` for tooling.
#[derive(Serialize)]
pub struct JunctionProgram {
    pub tls_id: String,
    pub zones: Vec<ProgramZone>,
    pub transitions: Vec<Transition>,
    pub program: Program,
    pub links: Vec<ProgramLink>,
    pub conflicts: Vec<(String, String)>,
}

#[derive(Serialize)]
pub struct ProgramZone {
    pub detector_id: String,
    pub phases: Vec<i32>,
    pub lanes: u32,
    pub length_meters: f64,
    pub edge: String,
    pub is_pedestrian: bool,
    pub situational: bool,
}

#[derive(Serialize)]
pub struct Transition {
    pub green_phase: i32,
    pub amber_phase: i32,
    pub amber_duration_secs: f64,
}

#[derive(Serialize)]
pub struct Program {
    pub program_id: String,
    pub phases: Vec<(f64, String)>,
    pub is_static: bool,
    pub min_green_secs: Vec<Option<f64>>,
    pub max_green_secs: Vec<Option<f64>>,
}

#[derive(Serialize)]
pub struct ProgramLink {
    pub index: usize,
    pub zone: String,
    pub from_edge: Option<String>,
    pub from_lane: Option<u32>,
    pub to_edge: Option<String>,
    pub direction: String,
}

pub fn write_programs(path: &Path, programs: &[JunctionProgram]) -> Result<()> {
    let out = suffixed(path, "programs");
    let out = out.with_extension("json");
    std::fs::write(&out, serde_json::to_string_pretty(programs)?)
        .with_context(|| format!("writing {}", out.display()))
}

/// Every same-class pair sharing ground splits it: two vehicle zones along
/// the perpendicular bisector of their stop lines; two pedestrian zones by
/// which one's crosswalks are nearer, each shared point going to the zone
/// whose crosswalk is closest — so a zone always keeps its own crosswalk,
/// and two crosswalks meeting at a corner split it along the line
/// equidistant from both. All cuts are computed against the original
/// polygons, so the result doesn't depend on the order zones are visited;
/// ground shared by three zones ends with the nearest.
fn split_overlaps(zones: &mut [Zone]) {
    let mut losses: Vec<Vec<MultiPolygon<f64>>> = vec![Vec::new(); zones.len()];
    for i in 0..zones.len() {
        for j in i + 1..zones.len() {
            if zones[i].class != zones[j].class {
                continue;
            }
            let shared = zones[i].polygon.intersection(&zones[j].polygon);
            if shared.unsigned_area() <= OVERLAP_AREA_THRESHOLD_M2 {
                continue;
            }
            if zones[i].lines.is_empty() || zones[j].lines.is_empty() {
                let Some(side) = half_plane(zones[j].site, zones[i].site) else {
                    continue;
                };
                losses[i].push(shared.difference(&side));
                losses[j].push(shared.intersection(&side));
            } else if let Some((towards_i, towards_j)) =
                perimeter_cut(&zones[i].polygon, &zones[j].polygon, &shared)
            {
                // Each loses the neighbour's whole side of the neighbour's
                // own polygon, not just of the shared ground: cutting with
                // a piece whose edge runs exactly along the zone's own
                // border leaves a zero-width wall behind, and the notch
                // comes out as a hole.
                losses[i].push(zones[j].polygon.intersection(&towards_j));
                losses[j].push(zones[i].polygon.intersection(&towards_i));
            } else {
                losses[i].push(geometry::nearer_to(
                    &zones[j].lines,
                    &zones[i].lines,
                    &zones[j].polygon,
                ));
                losses[j].push(geometry::nearer_to(
                    &zones[i].lines,
                    &zones[j].lines,
                    &zones[i].polygon,
                ));
            }
        }
    }
    for (zone, lost) in zones.iter_mut().zip(losses) {
        for piece in lost {
            zone.polygon = zone.polygon.difference(&piece);
        }
    }
    // Ground a zone ends up surrounding (past two crosswalks' shared corner,
    // equally near both, the tie can fall to the one wrapped around by the
    // other) is the surrounding zone's: a zone has no holes.
    for i in 0..zones.len() {
        let filled = geometry::without_holes(zones[i].polygon.clone());
        let enclosed = filled.difference(&zones[i].polygon);
        if enclosed.unsigned_area() <= OVERLAP_AREA_THRESHOLD_M2 {
            continue;
        }
        for j in 0..zones.len() {
            if j != i && zones[j].class == zones[i].class {
                zones[j].polygon = zones[j].polygon.difference(&enclosed);
            }
        }
        zones[i].polygon = filled;
    }
    for zone in zones.iter_mut() {
        zone.polygon = keep_part_with(
            &zone.polygon,
            zone.stop_line,
            zone.class == Class::Pedestrian,
        );
        if !contains(&zone.polygon, zone.stop_line) && contains(&zone.polygon, zone.site) {
            zone.stop_line = zone.site;
        }
    }
}

/// Cleaning straightens each zone on its own, and straightening one side's
/// copy of a long border two zones share by a centimetre adds up to a
/// visible overlap: what two zones still share goes to one of them — the
/// pedestrian zone against a vehicle zone, the earlier one between two of
/// a kind — and the one giving way is only straightened inwards after, so
/// no border moves back onto its neighbour.
fn settle_residual_overlaps(zones: &mut [Zone]) {
    let pedestrian = |z: &Zone| z.class == Class::Pedestrian;
    for i in 0..zones.len() {
        for j in 0..zones.len() {
            let gives_way = if pedestrian(&zones[i]) == pedestrian(&zones[j]) {
                i > j && zones[i].class == zones[j].class
            } else {
                !pedestrian(&zones[i])
            };
            if gives_way
                && zones[i]
                    .polygon
                    .intersection(&zones[j].polygon)
                    .unsigned_area()
                    > OVERLAP_AREA_THRESHOLD_M2
            {
                let cut = zones[i].polygon.difference(&zones[j].polygon);
                zones[i].polygon = geometry::straighten_inwards(cut);
            }
        }
    }
}

/// Where two overlapping zones' outlines cross at exactly two points (two
/// stripes overlapping at a corner or side by side), the straight line
/// through those points: each zone keeps the half-plane its own
/// unshared ground lies in. `None` when the outlines cross some other way
/// (one stripe right across the other), or both keep the same side.
fn perimeter_cut(
    a: &MultiPolygon<f64>,
    b: &MultiPolygon<f64>,
    shared: &MultiPolygon<f64>,
) -> Option<(MultiPolygon<f64>, MultiPolygon<f64>)> {
    use geo::Centroid;
    let segments = |m: &MultiPolygon<f64>| -> Vec<(Pt, Pt)> {
        m.0.iter()
            .flat_map(|p| p.exterior().lines())
            .map(|l| ([l.start.x, l.start.y], [l.end.x, l.end.y]))
            .collect()
    };
    let mut crossings: Vec<Pt> = Vec::new();
    for (p0, p1) in segments(a) {
        for &(q0, q1) in &segments(b) {
            let (d, e) = (
                [p1[0] - p0[0], p1[1] - p0[1]],
                [q1[0] - q0[0], q1[1] - q0[1]],
            );
            let Some((t, u)) = geometry::line_intersection(p0, d, q0, e) else {
                continue;
            };
            if (0.0..=1.0).contains(&t) && (0.0..=1.0).contains(&u) {
                let x = [p0[0] + d[0] * t, p0[1] + d[1] * t];
                if crossings
                    .iter()
                    .all(|&c| geometry::dist(c, x) > geometry::WELD_METERS)
                {
                    crossings.push(x);
                }
            }
        }
    }
    let [p, q] = crossings[..] else {
        return None;
    };
    let along = geometry::unit([q[0] - p[0], q[1] - p[1]])?;
    if geometry::dist(p, q) < 0.5 {
        return None;
    }
    let side = |m: &MultiPolygon<f64>| {
        let c = m.difference(shared).centroid()?;
        Some((c.x() - p[0]) * along[1] - (c.y() - p[1]) * along[0])
    };
    let (sa, sb) = (side(a)?, side(b)?);
    if sa * sb >= 0.0 {
        return None;
    }
    let half = |sign: f64| {
        let normal = [along[1] * sign, -along[0] * sign];
        let big = 100_000.0;
        let corner = |n: f64, s: f64| Coord {
            x: p[0] + normal[0] * n + along[0] * s,
            y: p[1] + normal[1] * n + along[1] * s,
        };
        MultiPolygon::new(vec![Polygon::new(
            LineString::new(vec![
                corner(0.0, -big),
                corner(big, -big),
                corner(big, big),
                corner(0.0, big),
                corner(0.0, -big),
            ]),
            Vec::new(),
        )])
    };
    Some((half(sa.signum()), half(sb.signum())))
}

/// Vehicle zones reach the pedestrian zones in front of (and behind) them
/// and stop there: no car waits on a crosswalk, and no strip of road is left
/// between where cars wait and where people cross. Each flat end of a lane
/// band with a pedestrian zone starting within [`CROSSWALK_REACH_METERS`]
/// carries on across that zone; then every pedestrian zone is taken out of
/// every vehicle zone, leaving the two sharing one border. A diagonal
/// crosswalk thus fills the wedge a square band end would leave short of it.
fn meet_crosswalks(zones: &mut [Zone]) {
    let crosswalks: Vec<MultiPolygon<f64>> = zones
        .iter()
        .filter(|z| z.class == Class::Pedestrian)
        .map(|z| z.polygon.clone())
        .collect();
    let all = crosswalks
        .iter()
        .fold(MultiPolygon::new(Vec::new()), |acc, c| acc.union(c));
    for zone in zones.iter_mut().filter(|z| z.class != Class::Pedestrian) {
        for end in &zone.ends {
            let across = |from: f64, to: f64| {
                let middle = (from + to) / 2.0;
                geometry::rectangle(
                    [
                        end.point[0] + end.outward[0] * middle,
                        end.point[1] + end.outward[1] * middle,
                    ],
                    end.outward,
                    to - from,
                    end.width,
                )
            };
            let window = across(0.0, CROSSWALK_SEARCH_AHEAD_METERS);
            let along = |c: &Coord<f64>| {
                (c.x - end.point[0]) * end.outward[0] + (c.y - end.point[1]) * end.outward[1]
            };
            let mut reach: f64 = 0.0;
            for crosswalk in &crosswalks {
                let hit = window.intersection(crosswalk);
                if hit.unsigned_area() <= OVERLAP_AREA_THRESHOLD_M2 {
                    continue;
                }
                let coords = || hit.0.iter().flat_map(|p| p.exterior().coords());
                let near = coords().map(along).fold(f64::INFINITY, f64::min);
                if near <= CROSSWALK_REACH_METERS {
                    reach = reach.max(coords().map(along).fold(0.0, f64::max));
                }
            }
            if reach > 0.0 {
                // Overlapping the band's own end a little, so the two meet
                // with no hairline between them.
                zone.polygon = zone.polygon.union(&across(-0.1, reach));
            }
        }
        if !zone.ends.is_empty() {
            let outside = zone.polygon.difference(&all).difference(&zone.keep_out);
            let kept = keep_part_with(&outside, zone.stop_line, false);
            zone.polygon = kept;
        }
        if !contains(&zone.polygon, zone.stop_line)
            && let Some(nearest) = nearest_inside(&zone.polygon, zone.stop_line)
        {
            zone.stop_line = nearest;
        }
    }
}

/// The point of `polygon` nearest to `point`, nudged a few centimetres
/// inside it.
fn nearest_inside(polygon: &MultiPolygon<f64>, point: Pt) -> Option<Pt> {
    use geo::ClosestPoint;
    let on_edge = match polygon.closest_point(&Point::new(point[0], point[1])) {
        geo::Closest::Intersection(p) | geo::Closest::SinglePoint(p) => [p.x(), p.y()],
        geo::Closest::Indeterminate => return None,
    };
    [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0]
        .into_iter()
        .find_map(|step| {
            (0..32).find_map(|k| {
                let angle = k as f64 * std::f64::consts::PI / 16.0;
                let candidate = [
                    on_edge[0] + angle.cos() * step,
                    on_edge[1] + angle.sin() * step,
                ];
                contains(polygon, candidate).then_some(candidate)
            })
        })
}

/// The half-plane of points closer to `near` than to `far`.
fn half_plane(far: Pt, near: Pt) -> Option<MultiPolygon<f64>> {
    let normal = geometry::unit([near[0] - far[0], near[1] - far[1]])?;
    let mid = [(near[0] + far[0]) / 2.0, (near[1] + far[1]) / 2.0];
    let big = 100_000.0;
    let along = geometry::right_of(normal);
    let corner = |n: f64, a: f64| Coord {
        x: mid[0] + normal[0] * n + along[0] * a,
        y: mid[1] + normal[1] * n + along[1] * a,
    };
    Some(MultiPolygon::new(vec![Polygon::new(
        LineString::new(vec![
            corner(0.0, -big),
            corner(big, -big),
            corner(big, big),
            corner(0.0, big),
            corner(0.0, -big),
        ]),
        Vec::new(),
    )]))
}

fn contains(polygon: &MultiPolygon<f64>, point: Pt) -> bool {
    polygon.contains(&Point::new(point[0], point[1]))
}

/// A vehicle zone is one contiguous area: keep the part holding its stop
/// line (else the largest). A pedestrian zone may legitimately be split by
/// a cut and keeps every part bigger than a sliver.
fn keep_part_with(polygon: &MultiPolygon<f64>, stop_line: Pt, keep_all: bool) -> MultiPolygon<f64> {
    let parts: Vec<Polygon<f64>> = polygon
        .0
        .iter()
        .filter(|p| p.unsigned_area() > 0.5)
        .cloned()
        .collect();
    if keep_all || parts.len() <= 1 {
        return MultiPolygon::new(parts);
    }
    let chosen = parts
        .iter()
        .find(|p| p.contains(&Point::new(stop_line[0], stop_line[1])))
        .or_else(|| {
            parts
                .iter()
                .max_by(|a, b| a.unsigned_area().total_cmp(&b.unsigned_area()))
        })
        .cloned();
    MultiPolygon::new(chosen.into_iter().collect())
}

pub fn suffixed(path: &Path, suffix: &str) -> PathBuf {
    match path.extension().and_then(|ext| ext.to_str()) {
        Some(extension) => path.with_extension(format!("{suffix}.{extension}")),
        None => path.with_extension(suffix),
    }
}

pub fn write(path: &Path, zones: &[Zone], projection: &Projection) -> Result<()> {
    for (suffix, pedestrian) in [("vehicles", false), ("pedestrians", true)] {
        let features = zones
            .iter()
            .filter(|z| (z.class == Class::Pedestrian) == pedestrian)
            .map(|z| feature(z, projection))
            .collect();
        let collection = FeatureCollection {
            bbox: None,
            features,
            foreign_members: None,
        };
        let out = suffixed(path, suffix);
        std::fs::write(&out, serde_json::to_string_pretty(&collection)?)
            .with_context(|| format!("writing {}", out.display()))?;
    }
    Ok(())
}

fn feature(zone: &Zone, projection: &Projection) -> Feature {
    let ring = |line: &LineString<f64>| -> Vec<Vec<f64>> {
        line.coords()
            .map(|c| projection.to_lon_lat([c.x, c.y]).to_vec())
            .collect()
    };
    let polygons: Vec<Vec<Vec<Vec<f64>>>> = zone
        .polygon
        .0
        .iter()
        .map(|part| {
            std::iter::once(ring(part.exterior()))
                .chain(part.interiors().iter().map(ring))
                .collect()
        })
        .collect();
    let geometry = if polygons.len() == 1 {
        Geometry::new_polygon(polygons.into_iter().next().expect("one part"))
    } else {
        Geometry::new_multi_polygon(polygons)
    };
    let mut properties = JsonObject::new();
    properties.insert("waiting_zone_id".into(), zone.id.clone().into());
    properties.insert("intersection_id".into(), zone.intersection.clone().into());
    properties.insert(
        "stop_line".into(),
        serde_json::json!(projection.to_lon_lat(zone.stop_line)),
    );
    properties.insert("modes".into(), serde_json::json!(zone.modes()));
    let mut feature = Feature::from(geometry);
    feature.properties = Some(properties);
    feature
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(x: f64, y: f64, side: f64) -> MultiPolygon<f64> {
        geometry::rectangle([x, y], [1.0, 0.0], side, side)
    }

    fn zone(id: &str, polygon: MultiPolygon<f64>, site: Pt) -> Zone {
        Zone {
            id: id.into(),
            class: Class::Pedestrian,
            polygon,
            stop_line: site,
            site,
            core: MultiPolygon::new(Vec::new()),
            ends: Vec::new(),
            keep_out: MultiPolygon::new(Vec::new()),
            lines: Vec::new(),
            intersection: String::new(),
        }
    }

    #[test]
    fn overlapping_same_class_zones_split_their_shared_ground() {
        let mut zones = vec![
            zone("a", square(0.0, 0.0, 10.0), [0.0, 0.0]),
            zone("b", square(6.0, 0.0, 10.0), [6.0, 0.0]),
        ];
        split_overlaps(&mut zones);
        let shared = zones[0]
            .polygon
            .intersection(&zones[1].polygon)
            .unsigned_area();
        assert!(
            shared < OVERLAP_AREA_THRESHOLD_M2,
            "still sharing {shared}m²"
        );
        // Nothing lost overall: 2 squares of 100m² overlapping by 40m².
        let total = zones[0].polygon.unsigned_area() + zones[1].polygon.unsigned_area();
        assert!((total - 160.0).abs() < 1e-6, "{total}");
        assert!(contains(&zones[0].polygon, [0.0, 0.0]) && contains(&zones[1].polygon, [6.0, 0.0]));
    }
}

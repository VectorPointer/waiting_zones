//! Invariants of the zones and programs generated from the real Barcelona
//! extract.

use std::collections::HashSet;
use std::path::PathBuf;

use geo::{Area, BooleanOps, Distance, Euclidean, Point, Validation};
use waiting_zones::output::Class;
use waiting_zones::zones::Reach;
use waiting_zones::{generate, osm, program};

/// Two zones sharing more ground than this overlap visibly on a map.
const MAX_SHARED_M2: f64 = 0.05;
/// GPS resolves metres; sub-metre residue from a cut isn't a real miss.
const TOLERANCE_METERS: f64 = 1.0;

/// Vertices closer than this are one point: a ring meeting itself this
/// close is touching itself, whatever floating-point residue keeps the two
/// copies apart.
const WELD_METERS: f64 = 0.05;
/// A ring turning back on itself by more than this at a vertex, its two
/// sides staying a hair apart, runs out and straight back — a needle or a
/// slit, never real ground.
const TURN_BACK_DEGREES: f64 = 170.0;
/// A vertex this close to the straight line between its neighbours adds
/// nothing to the shape.
const REDUNDANT_VERTEX_METERS: f64 = 0.01;

/// A zero-width slit or needle in `ring` — valid by OGC's rules when its two
/// sides stay a hair apart, but drawn on a map as a line through the zone.
fn hairline_defect(ring: &geo::LineString<f64>) -> Option<String> {
    let mut points: Vec<geo::Coord<f64>> = Vec::new();
    for &c in ring.coords() {
        if points
            .last()
            .is_none_or(|p: &geo::Coord<f64>| (p.x - c.x).hypot(p.y - c.y) > WELD_METERS)
        {
            points.push(c);
        }
    }
    if points.len() > 1
        && (points[0].x - points[points.len() - 1].x)
            .hypot(points[0].y - points[points.len() - 1].y)
            <= WELD_METERS
    {
        points.pop();
    }
    let n = points.len();
    for i in 0..n {
        let (a, b, c) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
        let (v1, v2) = ((b.x - a.x, b.y - a.y), (c.x - b.x, c.y - b.y));
        let (l1, l2) = (v1.0.hypot(v1.1), v2.0.hypot(v2.1));
        let turn = ((v1.0 * v2.0 + v1.1 * v2.1) / (l1 * l2))
            .clamp(-1.0, 1.0)
            .acos()
            .to_degrees();
        // Only a hairline: the two sides staying within `WELD_METERS` of each
        // other. A sharp corner whose sides spread apart is real ground.
        let to_line = |p: geo::Coord<f64>, from: geo::Coord<f64>, to: geo::Coord<f64>| {
            let (dx, dy) = (to.x - from.x, to.y - from.y);
            ((p.x - from.x) * dy - (p.y - from.y) * dx).abs() / dx.hypot(dy)
        };
        let apart = to_line(a, b, c).min(to_line(c, a, b));
        if turn > TURN_BACK_DEGREES && apart <= WELD_METERS {
            return Some(format!(
                "ring turns back on itself ({turn:.1}°) at ({:.1}, {:.1})",
                b.x, b.y
            ));
        }
        for j in i + 2..n {
            if (i, j) != (0, n - 1)
                && (points[i].x - points[j].x).hypot(points[i].y - points[j].y) <= WELD_METERS
            {
                return Some(format!("ring touches itself at ({:.1}, {:.1})", b.x, b.y));
            }
        }
    }
    None
}

#[test]
fn real_barcelona_zones_and_programs_hold_their_invariants() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/barcelona/barcelona.osm");
    let osm = osm::read(&path).expect("reading the Barcelona extract");
    let generated = generate(&osm, Reach { max_length: None });
    let zones = &generated.zones;

    let vehicles = zones
        .iter()
        .filter(|z| z.class != Class::Pedestrian)
        .count();
    let pedestrians = zones.len() - vehicles;
    println!(
        "{vehicles} vehicle zones, {pedestrians} pedestrian zones ({} unassigned), {} junction programs",
        generated.unassigned_pedestrian_zones,
        generated.programs.len()
    );
    assert!(vehicles > 400 && pedestrians > 300 && generated.programs.len() > 100);

    let cycleways: HashSet<String> = osm
        .ways
        .iter()
        .filter(|w| w.tag("highway") == Some("cycleway"))
        .map(|w| w.id.to_string())
        .collect();
    let bike_only = waiting_zones::network::bike_only_crossings(&osm);
    let mut problems = Vec::new();
    let mut seen = HashSet::new();
    for zone in zones {
        // Cycling infrastructure is left out altogether: no zone on a
        // cycleway, none at a crossing only a cycleway runs through.
        let way = zone
            .id
            .trim_start_matches('-')
            .split(['#', '_'])
            .next()
            .unwrap_or("");
        if zone.class != Class::Pedestrian && cycleways.contains(way) {
            problems.push(format!("{}: a vehicle zone on a cycleway", zone.id));
        }
        if zone.class == Class::Pedestrian
            && way.parse().is_ok_and(|anchor| bike_only.contains(&anchor))
        {
            problems.push(format!("{}: a zone at a bicycle-only crossing", zone.id));
        }
        if !seen.insert(zone.id.as_str()) {
            problems.push(format!("{}: id used by more than one zone", zone.id));
        }
        // A ring touching or crossing itself (a zero-width slit left where
        // two bands meet, say) is invalid GeoJSON, and a map draws it as a
        // line through the middle of the zone.
        if !zone.polygon.is_valid() {
            let why = zone
                .polygon
                .check_validation()
                .err()
                .map(|e| e.to_string())
                .unwrap_or_default();
            problems.push(format!(
                "{}: invalid polygon ({})",
                zone.id,
                why.chars().take(120).collect::<String>()
            ));
        }
        for ring in zone
            .polygon
            .0
            .iter()
            .flat_map(|p| std::iter::once(p.exterior()).chain(p.interiors()))
        {
            if let Some(defect) = hairline_defect(ring) {
                problems.push(format!("{}: {defect}", zone.id));
            }
        }
        if Euclidean.distance(
            &Point::new(zone.stop_line[0], zone.stop_line[1]),
            &zone.polygon,
        ) > TOLERANCE_METERS
        {
            problems.push(format!("{}: stop line outside its own zone", zone.id));
        }
        // Every vertex shapes the outline: none sits on the straight line
        // between its two neighbours.
        for ring in zone
            .polygon
            .0
            .iter()
            .flat_map(|p| std::iter::once(p.exterior()).chain(p.interiors()))
        {
            let points: Vec<geo::Coord<f64>> = ring.coords().copied().collect();
            let n = points.len() - 1;
            let redundant = (0..n)
                .filter(|&i| {
                    let (a, b, c) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
                    let (dx, dy) = (c.x - a.x, c.y - a.y);
                    let t = (((b.x - a.x) * dx + (b.y - a.y) * dy) / (dx * dx + dy * dy))
                        .clamp(0.0, 1.0);
                    (b.x - a.x - dx * t).hypot(b.y - a.y - dy * t) <= REDUNDANT_VERTEX_METERS
                })
                .count();
            if redundant > 0 {
                problems.push(format!(
                    "{}: {redundant} vertex/vertices adding nothing",
                    zone.id
                ));
            }
        }
        // A waiting zone is solid ground: a hole in one is either ground
        // nobody waits on in the middle of a queue, or a sliver two bands
        // failed to close.
        let holes: usize = zone.polygon.0.iter().map(|p| p.interiors().len()).sum();
        if holes > 0 {
            problems.push(format!("{}: {holes} hole(s)", zone.id));
        }
        let lost = zone.core.difference(&zone.polygon).unsigned_area();
        if lost > 1.0 {
            problems.push(format!(
                "{}: lost {lost:.1}m² of its own crosswalk",
                zone.id
            ));
        }
    }
    for (i, a) in zones.iter().enumerate() {
        for b in &zones[i + 1..] {
            // Same-class zones never share ground, and no car waits on a
            // crosswalk: a vehicle zone and a pedestrian zone share only
            // their border. Two zones at different levels (a bridge over a
            // street) cross in plan without meeting, so only equal levels
            // are compared.
            let pedestrian = |c: Class| c == Class::Pedestrian;
            if (a.class == b.class || pedestrian(a.class) != pedestrian(b.class))
                && a.layer == b.layer
            {
                let shared = a.polygon.intersection(&b.polygon).unsigned_area();
                if shared > MAX_SHARED_M2 {
                    problems.push(format!("{} and {} share {shared:.2}m²", a.id, b.id));
                }
            }
        }
    }
    for (program, plan) in generated.programs.iter().zip(&generated.plans) {
        for problem in program::violations(plan) {
            problems.push(format!("junction {}: {problem}", program.tls_id));
        }
        for zone in &program.zones {
            if zone.phases.is_empty() {
                problems.push(format!(
                    "junction {}: zone {} is never green",
                    program.tls_id, zone.detector_id
                ));
            }
            if !seen.contains(zone.detector_id.as_str()) {
                problems.push(format!(
                    "junction {}: zone {} isn't in the GeoJSON",
                    program.tls_id, zone.detector_id
                ));
            }
        }
    }
    assert!(
        problems.is_empty(),
        "{} problem(s):\n{}",
        problems.len(),
        problems.join("\n")
    );
}

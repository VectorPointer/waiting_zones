//! Every real OSM crosswalk that touches a pedestrian zone lies entirely
//! inside one zone — what a user sees on the map: a zebra half outside the
//! zone meant to detect someone standing on it is a defect, whatever the
//! reason the generator missed it.

use std::path::PathBuf;

use geo::{Coord, Distance, Euclidean, LineString, Point};
use waiting_zones::output::Class;
use waiting_zones::zones::Reach;
use waiting_zones::{generate, osm};

/// GPS resolves metres; sub-metre residue from a cut isn't a real miss.
const TOLERANCE_METERS: f64 = 1.0;

#[test]
fn every_real_crosswalk_touching_a_zone_lies_inside_one() {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("data/barcelona/barcelona.osm");
    let osm = osm::read(&path).expect("reading the Barcelona extract");
    let generated = generate(&osm, Reach { max_length: None });
    let pedestrian: Vec<_> = generated
        .zones
        .iter()
        .filter(|z| z.class == Class::Pedestrian)
        .collect();

    let mut outside = Vec::new();
    for way in osm
        .ways
        .iter()
        .filter(|w| w.tag("footway") == Some("crossing"))
    {
        let points: Vec<[f64; 2]> = way
            .refs
            .iter()
            .filter_map(|n| osm.nodes.get(n))
            .map(|n| generated.projection.to_local(n.lon_lat))
            .collect();
        if points.len() < 2 {
            continue;
        }
        let line = LineString::new(points.iter().map(|&[x, y]| Coord { x, y }).collect());
        let touched: Vec<_> = pedestrian
            .iter()
            .filter(|z| Euclidean.distance(&line, &z.polygon) <= TOLERANCE_METERS)
            .collect();
        if touched.is_empty() {
            continue;
        }
        let worst = |z: &&&waiting_zones::output::Zone| {
            points
                .iter()
                .map(|&[x, y]| Euclidean.distance(&Point::new(x, y), &z.polygon))
                .fold(0.0, f64::max)
        };
        let (best, reach) = touched
            .iter()
            .map(|z| (z.id.as_str(), worst(z)))
            .min_by(|a, b| a.1.total_cmp(&b.1))
            .expect("touched");
        if reach > TOLERANCE_METERS {
            outside.push(format!(
                "OSM way {}: reaches {reach:.1}m outside {best}, the best zone it touches",
                way.id
            ));
        }
    }
    assert!(
        outside.is_empty(),
        "{} real crosswalk(s) touch a pedestrian zone but aren't inside one:\n{}",
        outside.len(),
        outside.join("\n")
    );
}

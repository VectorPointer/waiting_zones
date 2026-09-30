//! Every real, surveyed OSM crosswalk that this crate matches to a SUMO
//! crossing lane (see `geojson_output::match_crosswalks`) has to land
//! entirely inside the one pedestrian zone that owns that lane —
//! `unsplit_crossings` widens the owner's own polygon to contain it (see
//! that function's own docs), and this checks the published GeoJSON a
//! client actually reads, not an intermediate this crate never ships.
//!
//! This is `pedestrian_zones_dont_split_a_crossing.rs`'s own sibling, one
//! level more demanding: that test only ever checks SUMO's own synthetic,
//! two-point `EdgeFunction::Crossing` lane shape, a proxy that's fully
//! self-contained within this crate alone. Confirmed on real Barcelona
//! data (`osm_aware_waiting_zones.md`'s own research) that the proxy isn't
//! a safe stand-in for this specific property: dozens of real crosswalks
//! sit measurably outside the zone meant to detect someone standing on
//! them, even though SUMO's own synthetic stand-in for the same crossing
//! is fully contained.

use std::path::PathBuf;

use geo::{Coord, LineString, MultiPolygon, Polygon as GeoPolygon};
use waiting_zones::geojson_output::{
    Reprojector, distance_to_polygon, feature_rings, match_crosswalks,
    to_feature_collection_with_crosswalks,
};

const NET_FILE: &str = "data/barcelona/barcelona.net.xml";
const OSM_FILE: &str = "data/barcelona/barcelona.osm";

/// A GPS fix resolves metres — the same real-client bound this crate's own
/// `pedestrian_zones_dont_split_a_crossing.rs` already uses for the
/// analogous check.
const CROSSWALK_CONTAINMENT_TOLERANCE_METERS: f64 = 1.0;

const METERS_PER_DEGREE_LON: f64 = 84_000.0;
const METERS_PER_DEGREE_LAT: f64 = 111_000.0;

#[test]
fn every_matched_real_crosswalk_is_fully_inside_its_own_zone() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let network =
        sumo_types::read_network(&manifest_dir.join(NET_FILE)).expect("reading the sample network");
    let osm =
        osm_crosswalks::read(&manifest_dir.join(OSM_FILE)).expect("reading the sample OSM extract");
    let crosswalks = osm_crosswalks::crosswalks(&osm);
    assert!(
        !crosswalks.is_empty(),
        "the sample OSM extract has real crosswalks"
    );

    let zones = waiting_zones::zone_generator::generate(&network, None, false);
    let pedestrian: Vec<_> = zones
        .into_iter()
        .filter(|zone| !zone.detect_persons.is_empty())
        .collect();
    assert!(
        !pedestrian.is_empty(),
        "the sample network produces pedestrian zones"
    );

    let reproject =
        Reprojector::new(&network.location).expect("the sample network is georeferenced");
    let matches = match_crosswalks(&network, &pedestrian, &reproject, &crosswalks);

    println!(
        "matched {} lane(s), {} unmatched, {} ambiguous",
        matches.footprints.len(),
        matches.unmatched.len(),
        matches.ambiguous.len()
    );
    println!("ambiguous lanes: {:?}", matches.ambiguous);
    assert!(
        matches.footprints.len() > 100,
        "expected a substantial number of real Barcelona crossings to match \
         (got {}) -- an empty or near-empty match set would let this test \
         pass vacuously",
        matches.footprints.len()
    );

    let collection = to_feature_collection_with_crosswalks(&network, &pedestrian, &crosswalks)
        .expect("building the pedestrian collection with OSM crosswalks folded in");

    let zone_polygons: Vec<(&str, MultiPolygon<f64>)> = collection
        .features
        .iter()
        .map(|feature| {
            let id = feature
                .property("waiting_zone_id")
                .unwrap()
                .as_str()
                .unwrap();
            let polygon = MultiPolygon::new(
                feature_rings(feature)
                    .into_iter()
                    .map(|ring| {
                        let points: Vec<Coord<f64>> = ring
                            .iter()
                            .map(|p| Coord {
                                x: p[0] * METERS_PER_DEGREE_LON,
                                y: p[1] * METERS_PER_DEGREE_LAT,
                            })
                            .collect();
                        GeoPolygon::new(LineString::new(points), Vec::new())
                    })
                    .collect(),
            );
            (id, polygon)
        })
        .collect();

    // Which zone owns each matched lane -- the same "entry, not exit"
    // structural-owner rule `unsplit_crossings` itself uses, restated here
    // because this test needs to know *which* zone to check a crosswalk
    // against, not just that some zone reaches it.
    let owner_of = |lane_id: &str| -> Option<&str> {
        pedestrian.iter().find_map(|zone| {
            let is_exit = zone.exits.iter().any(|gate| gate.lane.0 == lane_id);
            let is_entry = zone.entries.iter().any(|gate| gate.lane.0 == lane_id);
            (is_entry && !is_exit).then_some(zone.id.0.as_str())
        })
    };

    let mut uncovered = Vec::new();
    for (lane_id, matched_local_points) in &matches.footprints {
        let Some(owner_id) = owner_of(lane_id) else {
            continue;
        };
        let Some((_, owner_polygon)) = zone_polygons.iter().find(|(id, _)| *id == owner_id) else {
            uncovered.push(format!(
                "lane {lane_id:?}: owner zone {owner_id:?} not in the published GeoJSON"
            ));
            continue;
        };
        // `matches.footprints` is in this crate's own local network metres
        // (the coordinate space `Reprojector::to_local` produces); the
        // published zone polygons above are lon/lat scaled by the same
        // flat-earth-at-Barcelona's-own-latitude factor
        // `pedestrian_zones_dont_split_a_crossing.rs` uses. Reprojecting
        // each footprint vertex back through `to_lon_lat` puts both sides
        // of the comparison in the same space.
        for part in &matched_local_points.0 {
            for point in part.exterior().coords() {
                let lon_lat = reproject
                    .to_lon_lat(sumo_types::domain::Point {
                        x: point.x,
                        y: point.y,
                        z: 0.0,
                    })
                    .expect("reprojecting a matched footprint vertex");
                let local = Coord {
                    x: lon_lat[0] * METERS_PER_DEGREE_LON,
                    y: lon_lat[1] * METERS_PER_DEGREE_LAT,
                };
                let distance = distance_to_polygon(local, owner_polygon);
                if distance > CROSSWALK_CONTAINMENT_TOLERANCE_METERS {
                    uncovered.push(format!(
                        "lane {lane_id:?}: a point of its matched OSM footprint is {distance:.2}m outside \
                         its own owner zone {owner_id:?}"
                    ));
                }
            }
        }
    }

    assert!(
        uncovered.is_empty(),
        "{} matched crosswalk footprint(s) reach outside their own owner zone:\n{}",
        uncovered.len(),
        uncovered.join("\n")
    );
}

/// What a user actually sees on the map: every real crosswalk that touches
/// a pedestrian zone has to lie entirely inside one — whether or not the
/// matcher paired it with a SUMO crossing lane. The test above only checks
/// the pairs the matcher made, so a crosswalk it failed to pair passed there
/// while still showing half outside its zone.
#[test]
fn every_real_crosswalk_touching_a_zone_lies_inside_one() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let network =
        sumo_types::read_network(&manifest_dir.join(NET_FILE)).expect("reading the sample network");
    let osm =
        osm_crosswalks::read(&manifest_dir.join(OSM_FILE)).expect("reading the sample OSM extract");
    let crosswalks = osm_crosswalks::crosswalks(&osm);
    let pedestrian: Vec<_> = waiting_zones::zone_generator::generate(&network, None, false)
        .into_iter()
        .filter(|zone| !zone.detect_persons.is_empty())
        .collect();
    let collection = to_feature_collection_with_crosswalks(&network, &pedestrian, &crosswalks)
        .expect("building the pedestrian collection with OSM crosswalks folded in");

    let local = |p: &[f64]| Coord {
        x: p[0] * METERS_PER_DEGREE_LON,
        y: p[1] * METERS_PER_DEGREE_LAT,
    };
    let zones: Vec<(String, MultiPolygon<f64>)> = collection
        .features
        .iter()
        .map(|feature| {
            let id = feature.property("waiting_zone_id").unwrap().as_str().unwrap().to_string();
            let rings = feature_rings(feature)
                .into_iter()
                .map(|ring| {
                    GeoPolygon::new(LineString::new(ring.iter().map(|p| local(p.as_slice())).collect()), Vec::new())
                })
                .collect();
            (id, MultiPolygon::new(rings))
        })
        .collect();

    let mut outside = Vec::new();
    for crosswalk in &crosswalks {
        let points: Vec<Coord<f64>> = crosswalk.points.iter().map(|p| local(p)).collect();
        let per_zone: Vec<(&str, f64, f64)> = zones
            .iter()
            .map(|(id, polygon)| {
                let d: Vec<f64> = points.iter().map(|&p| distance_to_polygon(p, polygon)).collect();
                (id.as_str(), d.iter().copied().fold(f64::INFINITY, f64::min), d.iter().copied().fold(0.0, f64::max))
            })
            .collect();
        let touched = per_zone.iter().any(|&(_, closest, _)| closest <= CROSSWALK_CONTAINMENT_TOLERANCE_METERS);
        if !touched {
            continue;
        }
        let (best, _, worst) = per_zone
            .iter()
            .filter(|&&(_, closest, _)| closest <= CROSSWALK_CONTAINMENT_TOLERANCE_METERS)
            .min_by(|a, b| a.2.total_cmp(&b.2))
            .copied()
            .expect("touched");
        if worst > CROSSWALK_CONTAINMENT_TOLERANCE_METERS {
            outside.push(format!(
                "OSM way {} ({:.1}m): reaches {worst:.1}m outside {best}, the best zone it touches",
                crosswalk.way_id,
                points.windows(2).map(|w| (w[1].x - w[0].x).hypot(w[1].y - w[0].y)).sum::<f64>()
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

//! A signalized pedestrian crossing — a real `EdgeFunction::Crossing`
//! lane, the painted zebra stripe `waiting_zones::zone_generator`'s own
//! `pedestrian_zones` includes among a zone's *entries* precisely so the
//! finished polygon covers "the whole crossing" (see that function's own
//! module docs) — has to land entirely inside **one** pedestrian waiting
//! zone. Not "some zone or other" for each of its own endpoints
//! separately: a real crosswalk split down the middle between two
//! neighbouring zones is a client-visible defect (half the stripe drawn
//! in one zone's colour, half in the other's on the panel; a pedestrian
//! standing at one end counts for a different signal than one standing
//! at the other) even though every point of it, taken alone, is inside
//! *a* zone.
//!
//! Checked against this crate's own real, shipped output — the same
//! `to_feature_collection` GeoJSON the panel actually renders — via the
//! crate's own genuinely `pub` `Reprojector`/`feature_rings`/
//! `distance_to_polygon` (not the `#[cfg(test)]`-only re-exports
//! `geojson_output::tests` uses internally, invisible to an integration
//! test that links the library's normal build): a straight rebuild of
//! each zone's own polygon from its own published GeoJSON, and each real
//! crossing lane's own shape reprojected the identical way, so this
//! measures the artefact a client actually consumes rather than an
//! intermediate the pipeline never ships.

use std::collections::HashMap;
use std::path::PathBuf;

use geo::{Coord, LineString, MultiPolygon, Polygon as GeoPolygon};
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{EdgeFunction, Lane};
use waiting_zones::geojson_output::{distance_to_polygon, feature_rings, to_feature_collection, Reprojector};

const NET_FILE: &str = "data/barcelona/barcelona.net.xml";

/// How far a crossing lane's own vertex may sit outside "its" zone
/// before this test cares — the same real-client bound
/// `every_real_barcelona_zone_covers_its_own_published_stop_line` (in
/// `geojson_output`'s own unit tests) already uses for the analogous
/// stop-line check: a GPS fix resolves metres, so sub-metre residue from
/// reprojection or a cut's own boundary isn't a real split.
const CROSSING_CONTAINMENT_TOLERANCE_METERS: f64 = 1.0;

/// `(lon, lat)` -> local, roughly-flat metres, at Barcelona's own
/// latitude — a degree of longitude is ~84km here against a degree of
/// latitude's ~111km, so a single scale factor would stretch one axis by
/// a third. Same conversion `geojson_output`'s own tests use for exactly
/// this reason.
const METERS_PER_DEGREE_LON: f64 = 84_000.0;
const METERS_PER_DEGREE_LAT: f64 = 111_000.0;

#[test]
fn no_real_barcelona_crossing_is_split_across_two_pedestrian_zones() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let net_file = manifest_dir.join(NET_FILE);

    let network = sumo_types::read_network(&net_file).expect("reading the sample Barcelona network");
    let zones = waiting_zones::zone_generator::generate(&network, None, false);
    let pedestrian: Vec<E3Detector> = zones.into_iter().filter(|zone| !zone.detect_persons.is_empty()).collect();
    assert!(!pedestrian.is_empty(), "the sample network should produce at least one pedestrian zone");

    let lanes_by_id: HashMap<&str, &Lane> =
        network.edges.iter().flat_map(|edge| &edge.lanes).map(|lane| (lane.id.0.as_str(), lane)).collect();
    let function_by_lane: HashMap<&str, EdgeFunction> = network
        .edges
        .iter()
        .flat_map(|edge| edge.lanes.iter().map(move |lane| (lane.id.0.as_str(), edge.function)))
        .collect();

    let reproject = Reprojector::new(&network.location).expect("the sample network is georeferenced");
    let collection = to_feature_collection(&network, &pedestrian).expect("building the pedestrian collection");

    // One polygon per zone, in local metres — rebuilt from the same
    // published GeoJSON a real client (and the panel) reads, via
    // `feature_rings` rather than trusting an intermediate this test
    // never sees.
    let zone_polygons: Vec<(&str, MultiPolygon<f64>)> = collection
        .features
        .iter()
        .map(|feature| {
            let id = feature.property("waiting_zone_id").unwrap().as_str().unwrap();
            let polygon = MultiPolygon::new(
                feature_rings(feature)
                    .into_iter()
                    .map(|ring| {
                        let points: Vec<Coord<f64>> = ring
                            .iter()
                            .map(|p| Coord { x: p[0] * METERS_PER_DEGREE_LON, y: p[1] * METERS_PER_DEGREE_LAT })
                            .collect();
                        GeoPolygon::new(LineString::new(points), Vec::new())
                    })
                    .collect(),
            );
            (id, polygon)
        })
        .collect();

    // Every crossing lane at least one pedestrian zone's own entries
    // name — a signalized crossing no zone references at all is a
    // different, pre-existing defect this test doesn't speak to (there's
    // no "split between two zones" to check when no zone ever claimed it
    // in the first place).
    let mut referenced_crossings: Vec<&str> = pedestrian
        .iter()
        .flat_map(|zone| &zone.entries)
        .map(|gate| gate.lane.0.as_str())
        .filter(|lane_id| function_by_lane.get(lane_id) == Some(&EdgeFunction::Crossing))
        .collect();
    referenced_crossings.sort_unstable();
    referenced_crossings.dedup();

    let mut split_crossings = Vec::new();
    let mut uncovered_crossings = Vec::new();
    for lane_id in referenced_crossings {
        let Some(lane) = lanes_by_id.get(lane_id) else { continue };

        // For each of the crossing's own shape vertices, every zone
        // whose own polygon reaches it (within tolerance) — the crossing
        // passes only if the *same* zone shows up for every vertex.
        let mut zones_touched_by_every_vertex: Option<Vec<&str>> = None;
        let mut any_uncovered = false;
        for point in &lane.shape.0 {
            let lon_lat = reproject.to_lon_lat(*point).expect("reprojecting a real network point");
            let local = Coord { x: lon_lat[0] * METERS_PER_DEGREE_LON, y: lon_lat[1] * METERS_PER_DEGREE_LAT };

            let touching: Vec<&str> = zone_polygons
                .iter()
                .filter(|(_, polygon)| distance_to_polygon(local, polygon) <= CROSSING_CONTAINMENT_TOLERANCE_METERS)
                .map(|&(id, _)| id)
                .collect();
            if touching.is_empty() {
                any_uncovered = true;
            }

            zones_touched_by_every_vertex = Some(match zones_touched_by_every_vertex {
                None => touching,
                Some(previous) => previous.into_iter().filter(|id| touching.contains(id)).collect(),
            });
        }

        if any_uncovered {
            uncovered_crossings.push(lane_id.to_string());
            continue;
        }
        if zones_touched_by_every_vertex.is_none_or(|common| common.is_empty()) {
            split_crossings.push(lane_id.to_string());
        }
    }

    assert!(
        uncovered_crossings.is_empty(),
        "{} crossing lane(s) have a vertex no pedestrian zone reaches at all:\n{}",
        uncovered_crossings.len(),
        uncovered_crossings.join("\n")
    );
    assert!(
        split_crossings.is_empty(),
        "{} crossing lane(s) don't land entirely inside one single pedestrian zone — each one is split \
         across two neighbouring zones instead, which a real client (and the panel) would draw as two \
         differently-coloured halves of the same physical crossing:\n{}",
        split_crossings.len(),
        split_crossings.join("\n")
    );
}

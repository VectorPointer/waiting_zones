//! Every zone this crate emits for the real Barcelona network becomes its
//! own GeoJSON polygon (see `geojson_output`'s own module docs: a client
//! geofences against this output from its own GPS). Two *different* zones
//! whose polygons overlap would mean a single GPS point can match both —
//! exactly the "which crossing am I actually waiting at" ambiguity
//! `geojson_output::overlapping_zone_ids`'s own docs describe, and a
//! concrete way it can happen (one physical corner feeding two
//! differently-signalled crossings) is documented on
//! `zone_generator::pedestrian_zones`.
//!
//! `geojson_output`'s own unit tests already prove
//! `overlapping_zone_ids` itself catches a deliberately-overlapping pair
//! and clears a deliberately-adjacent (touching, not overlapping) one —
//! this test applies it to real data instead of a synthetic fixture, the
//! same relationship `sumo_validates_output.rs` has to this crate's other
//! unit tests.

use std::path::PathBuf;

const NET_FILE: &str = "data/barcelona/barcelona.net.xml";

#[test]
fn no_two_barcelona_zones_overlap() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let net_file = manifest_dir.join(NET_FILE);

    let network =
        sumo_types::read_network(&net_file).expect("reading the sample Barcelona network");
    let zones = waiting_zones::zone_generator::generate(&network, None);
    assert!(
        !zones.is_empty(),
        "the sample network should produce at least one waiting zone; \
         if it legitimately doesn't any more, this test needs a different fixture"
    );

    let collection = waiting_zones::geojson_output::to_feature_collection(&network, &zones)
        .expect("building the GeoJSON feature collection");

    let overlaps = waiting_zones::geojson_output::overlapping_zone_ids(&collection);
    assert!(
        overlaps.is_empty(),
        "{} pair(s) of different zones have overlapping polygons — a client geofencing \
         against this output could match both for a single GPS point: {overlaps:?}",
        overlaps.len()
    );
}

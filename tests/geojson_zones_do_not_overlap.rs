//! Every *vehicle* zone this crate emits for the real Barcelona network
//! becomes its own GeoJSON polygon (see `geojson_output`'s own module docs:
//! a client geofences against this output from its own GPS). Two different
//! vehicle zones whose polygons overlap would mean a single GPS point can
//! match both — exactly the "which movement am I actually queueing for"
//! ambiguity `geojson_output::overlapping_zone_ids`'s own docs describe.
//!
//! Pedestrian zones are deliberately exempt. Two crossings meeting at one
//! corner each span both banks, so they share that corner by construction
//! (see `zone_generator::pedestrian_zones`); a pedestrian standing there
//! may be waiting for either crossing and there is no heading filter to
//! tell them apart, so the overlap is the correct model rather than a
//! defect `resolve_overlaps` should cut away — cutting it would strip the
//! shared corner from both and leave neither covering its own stop line.
//!
//! Checked *within the vehicle file*, not across one combined collection —
//! a vehicle zone overlapping a pedestrian one is not a defect at all
//! (different files, different clients), which is the whole reason the
//! output is split.
//!
//! `geojson_output`'s own unit tests already prove
//! `overlapping_zone_ids` itself catches a deliberately-overlapping pair
//! and clears a deliberately-adjacent (touching, not overlapping) one —
//! this test applies it to real data instead of a synthetic fixture, the
//! same relationship `sumo_validates_output.rs` has to this crate's other
//! unit tests.

use std::path::PathBuf;
use sumo_types::additional::domain::E3Detector;

const NET_FILE: &str = "data/barcelona/barcelona.net.xml";

#[test]
fn no_two_barcelona_zones_of_the_same_mode_overlap() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let net_file = manifest_dir.join(NET_FILE);

    let network =
        sumo_types::read_network(&net_file).expect("reading the sample Barcelona network");
    let zones = waiting_zones::zone_generator::generate(&network, None, false);
    assert!(
        !zones.is_empty(),
        "the sample network should produce at least one waiting zone; \
         if it legitimately doesn't any more, this test needs a different fixture"
    );

    // Vehicles only — see this file's own module docs on why pedestrian
    // zones are exempt.
    let vehicle: Vec<E3Detector> =
        zones.into_iter().filter(|zone| zone.detect_persons.is_empty()).collect();

    // Judged at an area a client could actually be positioned inside, not
    // at `overlaps::OVERLAP_AREA_THRESHOLD_M2`'s own square millimetre.
    // That constant is the resolver's *goal* — it keeps shrinking a
    // dispute for as long as it can measure one — but it is not a contract
    // this test should hold it to: a GPS fix resolves metres, so two zones
    // sharing a few mm² cannot make any real client match both, and 20 of
    // Barcelona's own 27 remaining pairs are that small. Failing on them
    // reports arithmetic residue as a product defect.
    const MIN_MEANINGFUL_OVERLAP_M2: f64 = 0.01;

    // Pairs that genuinely share more ground than that today. Every one is
    // two movement groups off the *same source edge*
    // (`239297086#16_straight` against `239297086#16_straight+left`, and so
    // on) or a street against its own opposite direction — cases where both
    // zones extend backward through one shared approach
    // (`zone_generator::extended_entry_lanes`) and so genuinely describe the
    // same asphalt, which `resolve_overlaps` can narrow but not make
    // disjoint without deciding which movement a driver who hasn't picked a
    // lane yet is queueing for. Listed rather than absorbed into a looser
    // threshold, so the number stays visible and has to come down.
    const ZONES_SHARING_ONE_APPROACH: [(&str, &str); 3] = [
        ("-21259582#7_straight", "21259582#5_straight"),
        ("-27641458#2_straight", "27641458#0_straight"),
        ("-870644483#3_straight", "870644483#1_straight"),
    ];

    let collection = waiting_zones::geojson_output::to_feature_collection(&network, &vehicle)
        .expect("building the vehicle GeoJSON feature collection");
    let overlaps =
        waiting_zones::geojson_output::overlapping_zone_ids_larger_than(&collection, MIN_MEANINGFUL_OVERLAP_M2);
    let known: Vec<(String, String)> = ZONES_SHARING_ONE_APPROACH
        .iter()
        .map(|(a, b)| ((*a).to_string(), (*b).to_string()))
        .collect();

    let unexpected: Vec<String> = overlaps
        .iter()
        .filter(|pair| !known.contains(pair))
        .map(|(a, b)| format!("{a} || {b}"))
        .collect();
    let fixed: Vec<String> =
        known.iter().filter(|pair| !overlaps.contains(pair)).map(|(a, b)| format!("{a} || {b}")).collect();

    assert!(
        unexpected.is_empty(),
        "{} pair(s) of same-mode zones overlap by more than {MIN_MEANINGFUL_OVERLAP_M2}m² and \
         aren't listed as known — a client geofencing against that file could match both for \
         one GPS point:\n{}",
        unexpected.len(),
        unexpected.join("\n")
    );
    assert!(
        fixed.is_empty(),
        "{} pair(s) listed in ZONES_SHARING_ONE_APPROACH no longer overlap — delete them from \
         that list so it keeps describing reality:\n{}",
        fixed.len(),
        fixed.join("\n")
    );
}

//! Regression test on real data rather than a hand-built fixture: reversing
//! every `tlLogic` program's phase order must not change which waiting zone
//! any id refers to. This is the failure mode the movement-based id scheme
//! (`zone_generator`) replaced — see that module's own docs — checked here
//! against Barcelona's real signal plans (286 `tlLogic` programs) instead of
//! a synthetic two-phase network.
//!
//! Comparing the *set* of id strings before and after is not enough: the old
//! scheme could swap which of two groups at the same junction got `_0` and
//! which got `_1`, and the set of strings in use ({"_0", "_1"}) would look
//! identical even though every id now names a different physical group. What
//! has to survive is the mapping from an id to the lanes it names.

use std::collections::BTreeMap;
use std::path::PathBuf;
use sumo_types::additional::domain::E3Detector;

const NET_FILE: &str = "data/barcelona/barcelona.net.xml";

#[test]
fn ids_keep_naming_the_same_lanes_after_reversing_every_signal_programs_phase_order() {
    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let net_file = manifest_dir.join(NET_FILE);
    let mut network =
        sumo_types::read_network(&net_file).expect("reading the sample Barcelona network");

    let original = waiting_zones::zone_generator::generate(&network, None);
    let original_lanes_by_id = lanes_by_id(&original);

    for program in &mut network.traffic_light_programs {
        program.phases.reverse();
    }

    let reordered = waiting_zones::zone_generator::generate(&network, None);
    let reordered_lanes_by_id = lanes_by_id(&reordered);

    assert_eq!(
        original.len(),
        reordered.len(),
        "reversing phase order must not change how many waiting zones are produced"
    );
    assert_eq!(
        original_lanes_by_id, reordered_lanes_by_id,
        "reversing every program's phase order must not change which lanes any id names \
         — a same-sized set of ids that got reassigned to different lanes would not show up \
         as a size or membership difference, only as a difference here"
    );
}

/// Each zone's id mapped to the sorted lane refs of its entry gates —
/// what a waiting zone's id actually names, independent of output order.
fn lanes_by_id(zones: &[E3Detector]) -> BTreeMap<String, Vec<String>> {
    zones
        .iter()
        .map(|zone| {
            let mut lanes: Vec<String> = zone
                .entries
                .iter()
                .map(|gate| gate.lane.0.clone())
                .collect();
            lanes.sort();
            (zone.id.0.clone(), lanes)
        })
        .collect()
}

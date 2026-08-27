//! Ground-truth check for a pedestrian waiting zone `zone_generator`
//! produces: poll, directly over TraCI and independent of any detector,
//! who's physically on a zone's own walkingarea edge right now
//! (`edge.getLastStepPersonIDs` — live position, computed from
//! `MSEdge::getSortedPersons`, not an event) and whether they're genuinely
//! halted (`person.getWaitingTime() > 0`). Whenever both hold, the zone's
//! own detector (`multientryexit.getLastStepVehicleIDs`, the call this
//! detector type exposes for person ids too, despite the name) must report
//! them.
//!
//! Not `person.getLanePosition()` against the zone's own entry/exit gate
//! positions, despite that looking like the more direct check: a
//! pedestrian's own reported lane position on a walkingarea can exceed the
//! lane's nominal length (observed here: ~4.0–4.2m reported on a lane whose
//! `.net.xml` length is 3.3m) — SUMO tracks a person's actual walked
//! distance through the walkingarea's real shape, which can be longer than
//! the straight-line lane length the zone's gates are expressed against.
//! Comparing against that would either reject genuinely-inside pedestrians
//! or need an arbitrary tolerance; asking the edge who's on it side-steps
//! the mismatch entirely.
//!
//! `sumo_validates_output.rs` only checks that a generated `.add.xml` loads
//! without errors — a zone can be perfectly well-formed XML, on the right
//! lane, with sane positions, and still be useless for its actual job if
//! the detector underneath never reports who's standing in it. This is the
//! check that would catch that.
//!
//! This currently reproduces a real, upstream SUMO defect rather than a bug
//! in anything this crate generates: `MSE3Collector`'s own per-step person
//! scan (`detectorUpdate` → `notifyMovePerson`) synthesizes a "previous
//! position" from each person's *current* speed
//! (`oldPos = newPos - SPEED2DIST(speed)`), so a halted person
//! (`speed == 0`) always has `oldPos == newPos` — their live position, not
//! a real history. If that live position is already past the entry gate
//! the first time this scan reaches them (which it reliably is, in
//! practice — see `traci::Client::edge_last_step_person_ids`'s own docs for
//! a real reproduction), `MSE3EntryReminder::notifyMove`'s own
//! `if (oldPos > myPosition) return false` — a shortcut that assumes
//! anyone already past the gate must have been entered on an earlier step —
//! wrongly skips them, and they are never entered at all. `control_loop`
//! works around this on the Leave side by never trusting this detector for
//! pedestrians in the first place (see its own `runner::update_occupancy`);
//! this test exists so the underlying defect stays pinned by something
//! that fails loudly once SUMO ships a fix, not by tribal knowledge.
//!
//! Skipped (not failed) when `sumo` isn't on `PATH`, same as
//! `sumo_validates_output.rs`.

use std::collections::HashMap;
use std::net::{SocketAddr, TcpListener};
use std::path::PathBuf;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

use sumo_types::additional::domain::E3Detector;

const NET_FILE: &str = "data/test_4x4_ped/test_4x4_ped.net.xml";
const ROUTE_FILE: &str = "data/test_4x4_ped/test_4x4_ped.pedestrian_only.rou.xml";
const SEED: &str = "42";
const MAX_SIM_SECONDS: u64 = 3600;

fn sumo_is_available() -> bool {
    Command::new("sumo")
        .arg("--version")
        .output()
        .is_ok_and(|output| output.status.success())
}

#[test]
#[ignore = "pins a real, still-open upstream SUMO defect (see this file's own module docs) \
            rather than anything this crate generates — currently fails against every SUMO \
            build that exists, including ones past the eclipse-sumo/sumo#18230 crash fix. \
            Run explicitly with `cargo test -- --ignored` to check whether a newer SUMO has \
            fixed it; once one has, remove this attribute."]
fn pedestrian_waiting_zones_detect_every_genuinely_halted_pedestrian() {
    if !sumo_is_available() {
        eprintln!("skipping: `sumo` not found on PATH");
        return;
    }

    let manifest_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
    let net_file = manifest_dir.join(NET_FILE);
    let route_file = manifest_dir.join(ROUTE_FILE);

    let network = sumo_types::read_network(&net_file).expect("reading test_4x4_ped's net.xml");
    let zones = waiting_zones::zone_generator::generate(&network, None);
    let pedestrian_zones: Vec<&E3Detector> =
        zones.iter().filter(|z| !z.detect_persons.is_empty()).collect();
    assert!(
        !pedestrian_zones.is_empty(),
        "test_4x4_ped's net.xml should produce at least one pedestrian waiting zone \
         (it has signalized crossings) — if it legitimately doesn't any more, this test \
         needs a different fixture"
    );

    // A pedestrian zone's entry lane id ("<edge>_<index>") minus its own
    // "_<index>" suffix is that lane's edge id — true for every internal
    // lane SUMO ever generates, not a fixture-specific assumption. Zone
    // geometry (see `zone_generator`'s own docs) always puts a pedestrian
    // zone's single entry/exit gate pair on one lane, so there's no
    // multi-lane case to fold in here.
    let zone_by_edge: HashMap<String, &E3Detector> = pedestrian_zones
        .iter()
        .map(|&zone| {
            let lane = &zone.entries.first().expect("zone with an entry gate").lane.0;
            let edge = lane
                .rsplit_once('_')
                .map(|(edge, _index)| edge)
                .unwrap_or_else(|| panic!("pedestrian zone lane {lane:?} has no \"_<index>\" suffix to strip"));
            (edge.to_string(), zone)
        })
        .collect();

    let scratch =
        std::env::temp_dir().join(format!("waiting_zones_traci_test_{}", std::process::id()));
    std::fs::create_dir_all(scratch.join("detector_output")).expect("scratch dir");
    let add_file = scratch.join("zones.add.xml");
    waiting_zones::zone_output::write(&add_file, zones.clone()).expect("writing the .add.xml");

    let listener = TcpListener::bind("127.0.0.1:0").expect("bind ephemeral port");
    let addr: SocketAddr = listener.local_addr().expect("local_addr");
    drop(listener);

    let mut command = Command::new("sumo");
    command
        .args(["-n", net_file.to_str().expect("net_file is valid UTF-8")])
        .args(["-a", add_file.to_str().expect("add_file is valid UTF-8")])
        .args(["-r", route_file.to_str().expect("route_file is valid UTF-8")])
        .current_dir(&scratch)
        .args(["--step-length", "1"])
        .args(["--seed", SEED])
        .args(["--remote-port", &addr.port().to_string()])
        .args(["--no-step-log", "--no-warnings"])
        .stdout(Stdio::null())
        .stderr(Stdio::null());

    let mut sumo = command.spawn().expect("spawn sumo");
    let mut client = traci::Client::connect(addr).expect("connect to sumo over traci");

    let mut checked_a_genuinely_halted_pedestrian = false;

    for _ in 0..MAX_SIM_SECONDS {
        client.simulation_step().expect("simulation_step");

        for (edge, &zone) in &zone_by_edge {
            let present = client
                .edge_last_step_person_ids(edge)
                .expect("edge_last_step_person_ids");
            if present.is_empty() {
                continue;
            }

            let reported = client
                .multi_entry_exit_vehicle_ids(&zone.id.0)
                .expect("multi_entry_exit_vehicle_ids");

            for person_id in &present {
                let waiting_time = client
                    .person_waiting_time(person_id)
                    .expect("person_waiting_time");
                if waiting_time <= 0.0 {
                    continue; // moving through, not waiting — not this test's concern
                }

                checked_a_genuinely_halted_pedestrian = true;
                assert!(
                    reported.iter().any(|id| id == person_id),
                    "{person_id} is physically on edge {edge:?} (zone {:?}'s own lane), \
                     genuinely halted ({waiting_time:.1}s and counting per \
                     person.getWaitingTime()) — but the zone's own detector reports \
                     {reported:?}, not including them",
                    zone.id.0,
                );
            }
        }

        if client.min_expected_vehicles().expect("min_expected_vehicles") == 0 {
            break;
        }
    }

    client.close().expect("close traci connection");
    wait_with_timeout(&mut sumo, Duration::from_secs(10));
    let _ = std::fs::remove_dir_all(&scratch);

    assert!(
        checked_a_genuinely_halted_pedestrian,
        "no pedestrian was ever observed halted inside a generated waiting zone during this \
         run — this test needs real contested demand to check anything at all; \
         `pedestrian_only` (seed {SEED}) has produced one reliably before"
    );
}

fn wait_with_timeout(child: &mut Child, timeout: Duration) {
    let deadline = std::time::Instant::now() + timeout;
    loop {
        match child.try_wait() {
            Ok(Some(_)) => return,
            Ok(None) if std::time::Instant::now() < deadline => {
                std::thread::sleep(Duration::from_millis(50));
            }
            _ => {
                let _ = child.kill();
                let _ = child.wait();
                return;
            }
        }
    }
}

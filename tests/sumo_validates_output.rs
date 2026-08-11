//! Loads the `.waiting-zones.add.xml` this crate generates for a real
//! network through SUMO's own engine and asserts it raises no errors.
//!
//! Not `netedit`: it's a GUI application with no scriptable way to load a
//! file and report validation errors headlessly. `sumo` (the simulation
//! engine, run for a single step and thrown away) shares the same core
//! loader and position-validation logic, so it catches the same class of
//! problem — this is the test that would have caught the `friendlyPos` bug
//! `zone_generator::full_lane_boundaries` fixed: every unit test on that
//! function's output was individually plausible (a real lane, a position
//! between 0 and the lane's reported length), and none of them could see
//! that SUMO's own *geometric* lane length can differ from the `.net.xml`
//! `length` attribute by the last few floating-point digits, which is
//! exactly what SUMO's own loader is needed to catch.
//!
//! Skipped (not failed) when `sumo` isn't on `PATH`: this project has no
//! other dependency on a real SUMO install, and this repo has no CI
//! pipeline yet to guarantee one is present, so a hard requirement here
//! would make `cargo test` fail on any machine without SUMO — a heavier
//! default than the rest of this crate asks for. Whoever changes
//! `zone_generator`/`zone_output` and has SUMO installed still gets the
//! real check; everyone else gets a visible skip, not a silent no-op.

use std::path::PathBuf;
use std::process::Command;

const NET_FILE: &str = "data/barcelona/barcelona.net.xml";

fn sumo_is_available() -> bool {
    Command::new("sumo")
        .arg("--version")
        .output()
        .is_ok_and(|output| output.status.success())
}

#[test]
fn generated_output_loads_in_sumo_without_errors() {
    if !sumo_is_available() {
        eprintln!("skipping: `sumo` not found on PATH");
        return;
    }

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

    // A fresh scratch directory per run rather than `tempfile`: this is the
    // only place in the crate that would need the dependency, and a
    // pid-suffixed path under `env::temp_dir()` is enough to avoid clashing
    // with a previous, presumably-cleaned-up run.
    let scratch = std::env::temp_dir().join(format!("waiting_zones_test_{}", std::process::id()));
    std::fs::create_dir_all(scratch.join("detector_output"))
        .expect("creating the scratch output directory");
    let add_file = scratch.join("zones.add.xml");

    waiting_zones::zone_output::write(&add_file, zones).expect("writing the .add.xml");

    // `--end 1`: SUMO validates every loaded element (network, additionals)
    // before the first simulation step runs at all, so there's no need to
    // actually simulate anything — this is the cheapest run that still
    // exercises the real loader.
    let output = Command::new("sumo")
        .arg("-n")
        .arg(&net_file)
        .arg("-a")
        .arg(&add_file)
        .args(["--no-step-log", "--end", "1"])
        .output()
        .expect("running sumo");

    let _ = std::fs::remove_dir_all(&scratch);

    let stderr = String::from_utf8_lossy(&output.stderr);
    assert!(
        !stderr.contains("Error:"),
        "sumo reported an error loading the generated file:\n{stderr}"
    );
}

//! Writes `<fixture-dir>/expected.geojson` from `<fixture-dir>/network.net.xml`,
//! using the crate's own default settings (`max_zone_length: None`,
//! `stop_at_complex_intersections: false`) — the same generation
//! `tests/zone_fixtures.rs`'s own `regenerate_expected_geojson` test does
//! for every fixture at once. This does just the one, for tooling that
//! only has a single fixture to (re)generate (`viz/serve.py`'s own
//! `/api/save_fixture` endpoint — see `viz/README.md`'s "Saving a zone as
//! a test fixture" section) without paying for every other fixture's own
//! network too.
//!
//! Usage: `cargo run --release --example write_expected_geojson -- <fixture-dir>`
//!
//! *Never* commit what this overwrites without visually confirming it
//! first — see `tests/fixtures/README.md`'s own docs.

use anyhow::{Context, Result, bail};
use std::path::Path;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let [_, fixture_dir] = args.as_slice() else {
        bail!("usage: write_expected_geojson <fixture-dir>");
    };
    let fixture_dir = Path::new(fixture_dir);
    let network_path = fixture_dir.join("network.net.xml");

    let network = sumo_types::read_network(&network_path).with_context(|| format!("reading {network_path:?}"))?;
    let zones = waiting_zones::zone_generator::generate(&network, None, false);
    let collection = waiting_zones::geojson_output::to_feature_collection(&network, &zones)
        .with_context(|| format!("building collection for {network_path:?}"))?;

    let output_path = fixture_dir.join("expected.geojson");
    let json = serde_json::to_string_pretty(&collection).context("serializing collection")?;
    std::fs::write(&output_path, json).with_context(|| format!("writing {output_path:?}"))?;
    println!("wrote {output_path:?}");
    Ok(())
}

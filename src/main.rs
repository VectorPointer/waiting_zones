use std::path::PathBuf;

use anyhow::Result;
use clap::Parser;
use waiting_zones::zones::Reach;

/// Generate waiting zones and signal programs from an OpenStreetMap
/// extract (`.osm`), with no SUMO network involved.
#[derive(Parser)]
struct Cli {
    input: PathBuf,

    /// Output path; `.vehicles`/`.pedestrians` are inserted before its
    /// extension, and `<stem>.programs.json` is written beside it. Defaults
    /// to `<input stem>.waiting-zones.geojson`.
    #[arg(long, value_name = "PATH")]
    geojson: Option<PathBuf>,

    /// Cap on a vehicle zone's whole length back from its stop line.
    /// Without it, a zone covers its whole approach and keeps extending
    /// through roads that only lead to it, until a fork or a signal.
    #[arg(long, value_name = "METERS")]
    max_zone_length: Option<f64>,

    /// Also write the split road-network files (`network.edges.json`,
    /// `network.junctions.json`, `network.connections.json`) into this
    /// directory, beside the zone catalogue — what the simulator reads.
    #[arg(long, value_name = "DIR")]
    network: Option<PathBuf>,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let geojson = cli.geojson.unwrap_or_else(|| {
        let stem = cli
            .input
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("output");
        cli.input
            .with_file_name(format!("{stem}.waiting-zones.geojson"))
    });
    let reach = Reach {
        max_length: cli.max_zone_length,
    };
    if let Some(dir) = &cli.network {
        waiting_zones::export_network(&cli.input, dir, reach)?;
    }
    let summary = waiting_zones::run(&cli.input, &geojson, reach)?;
    println!(
        "{} vehicle zone(s), {} pedestrian zone(s) ({} not at a signalized junction), {} junction program(s) -> {}",
        summary.vehicle_zones,
        summary.pedestrian_zones,
        summary.unassigned_pedestrian_zones,
        summary.junctions,
        geojson.display()
    );
    Ok(())
}

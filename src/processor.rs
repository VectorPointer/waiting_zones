//! Drives the actual `.net.xml` -> `.waiting-zones.add.xml` conversion:
//! reads and deserializes the input (layer 1, [`crate::schema`]), converts
//! it to [`crate::domain::Network`] (layer 2), generates the network's
//! [`crate::domain::WaitingZone`]s, and writes them out (see
//! [`crate::zone_output`]).

use crate::config::Config;
use crate::domain::Network;
use crate::schema;
use anyhow::{Context, Result};
use std::fs::{self, File};
use std::io::BufReader;
use xsd_parser_types::quick_xml::{DeserializeSync, IoReader};

/// Reads and deserializes `path` into a [`Network`].
fn read_network(path: &std::path::Path) -> Result<Network> {
    let input_file =
        File::open(path).with_context(|| format!("Could not open input file: {path:?}"))?;
    let mut reader = IoReader::new(BufReader::new(input_file));

    let net = schema::NetType::deserialize(&mut reader)
        .map_err(|error| anyhow::anyhow!("failed to parse {path:?}: {error}"))?;

    Network::try_from(net).with_context(|| format!("invalid SUMO network in {path:?}"))
}

pub fn run(config: Config) -> Result<()> {
    let network = read_network(&config.input)?;
    let zones = crate::zone_generator::generate(&network, config.max_zone_length);
    let xml = crate::zone_output::to_xml(&zones)?;

    fs::write(&config.output, xml)
        .with_context(|| format!("Could not write output file: {:?}", config.output))?;

    println!("Processing completed successfully: {:?}", config.output);
    Ok(())
}
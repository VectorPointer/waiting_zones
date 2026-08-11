//! Drives the actual `.net.xml` -> `.waiting-zones.add.xml` conversion:
//! reads the input into a [`sumo_types::Network`], generates that network's
//! waiting zones (see [`crate::zone_generator`]), and writes them out (see
//! [`crate::zone_output`]).

use crate::config::Config;
use anyhow::Result;

pub fn run(config: Config) -> Result<()> {
    let network = sumo_types::read_network(&config.input)?;
    let zones = crate::zone_generator::generate(&network, config.max_zone_length);
    crate::zone_output::write(&config.output, zones)?;

    println!("Processing completed successfully: {:?}", config.output);
    Ok(())
}

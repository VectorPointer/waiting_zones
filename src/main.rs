//! This program takes a .net.xml input from SUMO and outputs a .waiting-zones.xml
//! The final consumer of .waiting-zones.xml is the user device (no SUMO format knowledge).

use anyhow::Result;
use waiting_zones::{Config, run};

fn main() -> Result<()> {
    let config = Config::build();
    run(config)
}

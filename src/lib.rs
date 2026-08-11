//! Turns a SUMO road network (`.net.xml`) into a `.waiting-zones.add.xml`
//! file describing E3 detector zones.
//!
//! Both halves of the SUMO file handling are the [`sumo_types`] crate's job —
//! reading and modelling the network ([`sumo_types::read_network`]), and
//! modelling and writing the `.add.xml`
//! ([`sumo_types::additional::write_additional`]). There's no
//! project-specific "waiting zone" type in between: what a waiting zone
//! *is*, [`sumo_types::additional::domain::E3Detector`] already models, so
//! what lives here is only the two steps around it —
//! [`zone_generator`], which derives `E3Detector` values from a
//! [`sumo_types::Network`], and [`zone_output`], which writes them out.

pub mod config;
pub mod processor;
/// Generates waiting zones (as `sumo_types` `E3Detector`s) from a
/// [`sumo_types::Network`].
pub mod zone_generator;
/// Writes waiting zones out as `.waiting-zones.add.xml`.
pub mod zone_output;

pub use config::Config;
pub use processor::run;

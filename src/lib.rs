/// Types generated from the SUMO XSDs by `build.rs`.
/// Layer 1: an (almost) literal mirror of the schema, with no domain semantics.
///
/// Enum variants preserve the exact case of the XSD value (see the custom
/// `Naming` in `build.rs`) so information like `state="M"` vs. `state="m"`
/// isn't lost; that's why `non_camel_case_types` is disabled for the whole
/// module.
#[allow(non_camel_case_types, unused_variables, unused_mut)]
pub mod schema {
    include!(concat!(env!("OUT_DIR"), "/generated_schema.rs"));
}

pub mod config;
/// Layer 2: the project's own types, decoupled from SUMO/XSD.
/// Reused across the rest of the project's modules.
pub mod domain;
pub mod processor;
/// Conversion from layer 1 (schema) to layer 2 (domain).
pub mod schema_mapper;
/// Generates [`domain::WaitingZone`]s from a [`domain::Network`].
pub mod zone_generator;
/// Converts [`domain::WaitingZone`]s into `.waiting-zones.add.xml`.
pub mod zone_output;

pub use config::Config;
pub use processor::run;

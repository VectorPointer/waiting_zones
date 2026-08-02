//! Serializes [`WaitingZone`]s into the `.waiting-zones.add.xml` format: a
//! SUMO "additional file" (`<additional>`) containing one `<e3Detector>` per
//! zone.
//!
//! Unlike `schema`/`schema_mapper` (which mirror `net_file.xsd` via
//! xsd-parser to *read* `.net.xml`), the types below are a small,
//! hand-written mirror of the relevant slice of `additional_file.xsd`'s
//! `e3DetectorType`/`detEntryExitType`, annotated for `yaserde`. Generating
//! them from the XSD the way we do for `schema` wouldn't help here:
//! xsd-parser's generator only emits plain struct/enum shapes, not XML
//! (de)serialization code, so we'd still have to annotate whatever it
//! produced with `yaserde` by hand — and `additionalType` also bundles in
//! ~20 unrelated element kinds (rerouters, WAUT, variable speed signs, ...)
//! we don't need.

use crate::domain::{WaitingZone, ZoneBoundary};
use anyhow::{anyhow, Result};
use uom::si::length::meter;
use yaserde::YaSerialize;

/// Subdirectory (relative to the `.waiting-zones.add.xml` file itself, per
/// how SUMO resolves a detector's `file` attribute) that per-detector
/// simulation output is written into. Fixed regardless of the input
/// `.net.xml`, so it can be gitignored with a single pattern instead of
/// having detector output files scattered next to whichever network they
/// came from.
const DETECTOR_OUTPUT_DIR: &str = "detector_output";

/// Root `<additional>` element of a `.waiting-zones.add.xml` file.
#[derive(Debug, YaSerialize)]
#[yaserde(rename = "additional")]
struct Additional {
    #[yaserde(rename = "e3Detector")]
    e3_detectors: Vec<E3Detector>,
}

/// Mirrors SUMO's `e3DetectorType` (`additional_file.xsd`), restricted to
/// the attributes we actually populate.
#[derive(Debug, YaSerialize)]
struct E3Detector {
    #[yaserde(attribute = true)]
    id: String,
    #[yaserde(attribute = true)]
    file: String,
    /// Where editors like netedit draw this detector's icon; see
    /// [`crate::domain::WaitingZone::icon_position`]. Purely cosmetic.
    #[yaserde(attribute = true)]
    pos: String,
    #[yaserde(rename = "detEntry")]
    entries: Vec<DetEntryExit>,
    #[yaserde(rename = "detExit")]
    exits: Vec<DetEntryExit>,
}

/// Mirrors SUMO's `detEntryExitType`.
#[derive(Debug, YaSerialize)]
struct DetEntryExit {
    #[yaserde(attribute = true)]
    lane: String,
    #[yaserde(attribute = true)]
    pos: f64,
}

impl From<&ZoneBoundary> for DetEntryExit {
    fn from(boundary: &ZoneBoundary) -> Self {
        DetEntryExit {
            lane: boundary.lane.0.clone(),
            pos: boundary.position.get::<meter>(),
        }
    }
}

impl From<&WaitingZone> for E3Detector {
    fn from(zone: &WaitingZone) -> Self {
        E3Detector {
            id: zone.id.0.clone(),
            // SUMO requires a `file` attribute (where a running simulation
            // would write live detector stats); we don't run a simulation,
            // but the attribute is still mandatory syntactically.
            file: format!("{DETECTOR_OUTPUT_DIR}/{}.xml", zone.id.0),
            pos: format!("{},{}", zone.icon_position.x, zone.icon_position.y),
            entries: zone.entries.iter().map(DetEntryExit::from).collect(),
            exits: zone.exits.iter().map(DetEntryExit::from).collect(),
        }
    }
}

/// Serializes `zones` into a complete `.waiting-zones.add.xml` document.
pub fn to_xml(zones: &[WaitingZone]) -> Result<String> {
    let additional = Additional {
        e3_detectors: zones.iter().map(E3Detector::from).collect(),
    };

    let config = yaserde::ser::Config {
        perform_indent: true,
        ..Default::default()
    };

    yaserde::ser::to_string_with_config(&additional, &config)
        .map_err(|message| anyhow!("failed to serialize waiting zones to XML: {message}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::WaitingZoneId;
    use uom::si::f64::Length;

    #[test]
    fn serializes_a_zone_with_one_entry_and_one_exit() {
        let zone = WaitingZone {
            id: WaitingZoneId("j0".into()),
            entries: vec![ZoneBoundary {
                lane: crate::domain::LaneId("e0_0".into()),
                position: Length::new::<meter>(10.0),
            }],
            exits: vec![ZoneBoundary {
                lane: crate::domain::LaneId("e1_0".into()),
                position: Length::new::<meter>(2.5),
            }],
            icon_position: crate::domain::Point { x: 0.0, y: 0.0, z: 0.0 },
        };

        let xml = to_xml(&[zone]).unwrap();

        assert!(xml.contains(r#"<e3Detector id="j0" file="detector_output/j0.xml" pos="0,0">"#));
        assert!(xml.contains(r#"<detEntry lane="e0_0" pos="10" />"#));
        assert!(xml.contains(r#"<detExit lane="e1_0" pos="2.5" />"#));
    }

    #[test]
    fn sets_pos_from_the_zones_icon_position() {
        let zone = WaitingZone {
            id: WaitingZoneId("j0_0".into()),
            entries: vec![ZoneBoundary {
                lane: crate::domain::LaneId("e0_0".into()),
                position: Length::new::<meter>(0.0),
            }],
            exits: vec![ZoneBoundary {
                lane: crate::domain::LaneId("e0_0".into()),
                position: Length::new::<meter>(25.0),
            }],
            icon_position: crate::domain::Point { x: 12.5, y: -3.0, z: 0.0 },
        };

        let xml = to_xml(&[zone]).unwrap();

        assert!(xml.contains(r#"pos="12.5,-3""#));
    }
}

//! Writes waiting zones out as `.waiting-zones.add.xml`: a SUMO "additional
//! file" (`<additional>`) containing one `<e3Detector>` per zone.
//!
//! [`crate::zone_generator`] already builds
//! [`E3Detector`](sumo_types::additional::domain::E3Detector) values
//! directly — there's no project-specific type to convert from. The one
//! thing left undone there is `file`, SUMO's mandatory (if unused, since
//! nothing here runs a simulation) output-stats attribute: naming output
//! files is this module's job, not the generator's, so it's filled in here,
//! right before handing everything to [`sumo_types::additional::write_additional`].
//!
//! This module used to carry a hand-written, `yaserde`-annotated mirror of
//! `additional_file.xsd`'s `e3DetectorType`/`detEntryExitType`, from when
//! [`sumo_types`] only *read* SUMO files; that's gone now that it writes
//! `.add.xml` itself.

use anyhow::{Context, Result};
use std::path::Path;
use sumo_types::additional::domain::{Additional, E3Detector};

/// Subdirectory (relative to the `.waiting-zones.add.xml` file itself, per
/// how SUMO resolves a detector's `file` attribute) that per-detector
/// simulation output is written into. Fixed regardless of the input
/// `.net.xml`, so it can be gitignored with a single pattern instead of
/// having detector output files scattered next to whichever network they
/// came from.
const DETECTOR_OUTPUT_DIR: &str = "detector_output";

/// Fills in each detector's `file`, which [`crate::zone_generator`] leaves
/// empty (see that module's own docs), and collects the result into the
/// `.add.xml` document that represents them.
///
/// Split out from [`write`] so tests can assert on the typed value rather
/// than on serialized text.
fn to_additional(zones: Vec<E3Detector>) -> Additional {
    let zones = zones.into_iter().map(|zone| E3Detector {
        file: format!("{DETECTOR_OUTPUT_DIR}/{}.xml", zone.id.0),
        ..zone
    });

    Additional {
        entry_exit_detectors: zones.collect(),
        ..Additional::default()
    }
}

/// Writes `zones` to `path` as a complete `.waiting-zones.add.xml` document.
pub fn write(path: &Path, zones: Vec<E3Detector>) -> Result<()> {
    sumo_types::additional::write_additional(path, &to_additional(zones))
        .with_context(|| format!("Could not write output file: {path:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sumo_types::additional::domain::{DetectorGate, DetectorId, LanePosition, LaneRef};
    use sumo_types::domain::Point;
    use sumo_types::uom::si::f64::Length;
    use sumo_types::uom::si::length::meter;

    fn zone(id: &str, entry: (&str, f64), exit: (&str, f64), icon: Point) -> E3Detector {
        let gate = |lane: &str, position_m: f64| DetectorGate {
            lane: LaneRef(lane.into()),
            position: LanePosition::FromStart(Length::new::<meter>(position_m)),
            friendly_position: None,
        };

        E3Detector {
            id: DetectorId(id.into()),
            entries: vec![gate(entry.0, entry.1)],
            exits: vec![gate(exit.0, exit.1)],
            file: String::new(), // this is exactly what `to_additional` fills in
            icon_position: Some(icon),
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
        }
    }

    #[test]
    fn fills_in_the_output_file_path_from_the_detector_id() {
        let additional = to_additional(vec![zone(
            "j0",
            ("e0_0", 10.0),
            ("e1_0", 2.5),
            Point::default(),
        )]);

        assert_eq!(additional.entry_exit_detectors.len(), 1);
        let detector = &additional.entry_exit_detectors[0];
        assert_eq!(detector.id, DetectorId("j0".into()));
        assert_eq!(detector.file, "detector_output/j0.xml");
        assert_eq!(detector.entries[0].lane, LaneRef("e0_0".into()));
        assert_eq!(
            detector.entries[0].position,
            LanePosition::FromStart(Length::new::<meter>(10.0))
        );
        assert_eq!(
            detector.exits[0].position,
            LanePosition::FromStart(Length::new::<meter>(2.5))
        );
    }

    #[test]
    fn leaves_the_rest_of_the_detector_untouched() {
        let icon = Point {
            x: 12.5,
            y: -3.0,
            z: 0.0,
        };
        let additional = to_additional(vec![zone("j0_0", ("e0_0", 0.0), ("e0_0", 25.0), icon)]);

        assert_eq!(additional.entry_exit_detectors[0].icon_position, Some(icon));
    }

    /// The one test that still goes through the serializer: everything above
    /// checks the mapping, but what SUMO actually consumes is the text, and
    /// this is the only place that would catch `sumo_types` emitting a shape
    /// `netedit` won't open.
    #[test]
    fn writes_the_element_shape_sumo_expects() {
        let icon = Point {
            x: 12.5,
            y: -3.0,
            z: 0.0,
        };
        let additional = to_additional(vec![zone("j0", ("e0_0", 10.0), ("e1_0", 2.5), icon)]);

        let mut buf = Vec::new();
        sumo_types::additional::write_additional_to(&additional, &mut buf).unwrap();
        let xml = String::from_utf8(buf).unwrap();

        assert!(
            xml.contains(r#"<e3Detector id="j0" file="detector_output/j0.xml" pos="12.5,-3">"#),
            "{xml}"
        );
        // Gates are asserted as opening tags only: `sumo_types` writes a
        // childless element as `<detEntry ...></detEntry>` rather than
        // self-closing it the way `netedit` does. The two are the same
        // document to any XML parser, SUMO's included, so this test pins the
        // attributes rather than the spelling.
        assert!(xml.contains(r#"<detEntry lane="e0_0" pos="10">"#), "{xml}");
        assert!(xml.contains(r#"<detExit lane="e1_0" pos="2.5">"#), "{xml}");
    }
}

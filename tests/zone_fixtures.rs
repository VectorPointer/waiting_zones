//! Table-driven: every subdirectory of `tests/fixtures/` holding a
//! `fixture.json` (`{"dataset", "junction_id"}`) and an `expected.geojson`
//! is one junction's zones as a human reviewed them in `viz/viz.html`
//! ("Guardar esta zona" / "Guardar intersección", see `viz/serve.py`),
//! possibly hand-edited into the shape they should have. The dataset is
//! regenerated from its own `data/<dataset>/<dataset>.osm` and that
//! junction's zones compared against the saved ones. See
//! `tests/fixtures/README.md`.
//!
//! # Why this compares areas, not vertices
//!
//! A zone's polygon is the end of a chain of floating-point boolean
//! operations: stable, but not canonical — the same ground can come out
//! with a different vertex list. What every consumer cares about is the
//! ground a zone claims, so that is what's compared: the area the two
//! shapes disagree about, as a fraction of the zone's own size. The *set*
//! of zone ids is compared exactly: a zone appearing, vanishing or being
//! renamed always deserves a look.
//!
//! # Excluded zones
//!
//! `tests/fixtures/<name>/excluded_zones.json` (optional; a JSON array of
//! `waiting_zone_id`s) names zones this fixture doesn't vouch for, removed
//! from both sides of the comparison — written by `viz.html`'s "quitar de
//! expected" button.

use std::collections::{HashMap, HashSet};
use std::path::{Path, PathBuf};

use geo::{Area, BooleanOps, Coord, LineString, MultiPolygon, Polygon};
use serde_json::Value;
use waiting_zones::zones::Reach;
use waiting_zones::{Generated, generate, osm};

/// How much of a zone's own area the saved and freshly generated shapes
/// may disagree about before that's a real change: well below anything a
/// GPS fix could resolve, well above floating-point residue.
const MAX_DIFFERING_AREA_FRACTION: f64 = 0.005;

fn fixture_dirs() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let Ok(entries) = std::fs::read_dir(&root) else {
        return Vec::new();
    };
    let mut dirs: Vec<PathBuf> = entries
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.join("fixture.json").is_file())
        .collect();
    dirs.sort();
    dirs
}

fn read_json(path: &Path) -> Result<Value, String> {
    let text = std::fs::read_to_string(path).map_err(|e| format!("reading {path:?}: {e}"))?;
    serde_json::from_str(&text).map_err(|e| format!("parsing {path:?}: {e}"))
}

fn excluded_zone_ids(dir: &Path) -> HashSet<String> {
    let path = dir.join("excluded_zones.json");
    if !path.is_file() {
        return HashSet::new();
    }
    let value = read_json(&path).unwrap_or_else(|e| panic!("{e}"));
    serde_json::from_value(value)
        .unwrap_or_else(|e| panic!("{path:?} isn't a JSON array of strings: {e}"))
}

fn generated(dataset: &str) -> Generated {
    let path = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("data")
        .join(dataset)
        .join(format!("{dataset}.osm"));
    let osm = osm::read(&path).unwrap_or_else(|e| panic!("reading {path:?}: {e:#}"));
    generate(&osm, Reach { max_length: None })
}

/// A GeoJSON Polygon/MultiPolygon, projected to the dataset's local metres.
fn polygon_in_meters(geometry: &Value, generated: &Generated) -> MultiPolygon<f64> {
    let ring = |positions: &Value| {
        LineString::new(
            positions
                .as_array()
                .into_iter()
                .flatten()
                .map(|p| {
                    let [x, y] = generated.projection.to_local([
                        p[0].as_f64().unwrap_or_default(),
                        p[1].as_f64().unwrap_or_default(),
                    ]);
                    Coord { x, y }
                })
                .collect(),
        )
    };
    let polygon = |rings: &Value| {
        let rings = rings.as_array()?;
        let (exterior, interiors) = rings.split_first()?;
        Some(Polygon::new(
            ring(exterior),
            interiors.iter().map(ring).collect(),
        ))
    };
    let coordinates = &geometry["coordinates"];
    let parts = match geometry["type"].as_str() {
        Some("Polygon") => polygon(coordinates).into_iter().collect(),
        Some("MultiPolygon") => coordinates
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(polygon)
            .collect(),
        _ => Vec::new(),
    };
    MultiPolygon::new(parts)
}

fn differing_area_fraction(expected: &MultiPolygon<f64>, actual: &MultiPolygon<f64>) -> f64 {
    let reference = expected.unsigned_area().max(actual.unsigned_area());
    if reference <= 0.0 {
        return if actual.unsigned_area() > 0.0 {
            1.0
        } else {
            0.0
        };
    }
    expected
        .difference(actual)
        .unsigned_area()
        .max(actual.difference(expected).unsigned_area())
        / reference
}

#[test]
fn every_fixture_matches_its_own_expected_geojson() {
    let mut runs: HashMap<String, Generated> = HashMap::new();
    let mut failures = Vec::new();
    for dir in fixture_dirs() {
        let name = dir.file_name().unwrap().to_string_lossy().into_owned();
        let (fixture, expected) = match (
            read_json(&dir.join("fixture.json")),
            read_json(&dir.join("expected.geojson")),
        ) {
            (Ok(f), Ok(e)) => (f, e),
            (Err(e), _) | (_, Err(e)) => {
                failures.push(format!("{name}: {e}"));
                continue;
            }
        };
        let (Some(dataset), Some(junction)) =
            (fixture["dataset"].as_str(), fixture["junction_id"].as_str())
        else {
            failures.push(format!(
                "{name}: fixture.json needs dataset and junction_id"
            ));
            continue;
        };
        let generated = runs
            .entry(dataset.to_string())
            .or_insert_with(|| generated(dataset));

        let excluded = excluded_zone_ids(&dir);
        let expected_by_id: HashMap<String, MultiPolygon<f64>> = expected["features"]
            .as_array()
            .into_iter()
            .flatten()
            .filter_map(|f| {
                let id = f["properties"]["waiting_zone_id"].as_str()?.to_string();
                Some((id, polygon_in_meters(&f["geometry"], generated)))
            })
            .filter(|(id, _)| !excluded.contains(id))
            .collect();
        let actual_by_id: HashMap<&str, &MultiPolygon<f64>> = generated
            .zones
            .iter()
            .filter(|z| z.intersection == junction && !excluded.contains(&z.id))
            .map(|z| (z.id.as_str(), &z.polygon))
            .collect();

        let mut missing: Vec<&String> = expected_by_id
            .keys()
            .filter(|id| !actual_by_id.contains_key(id.as_str()))
            .collect();
        let mut unexpected: Vec<&&str> = actual_by_id
            .keys()
            .filter(|id| !expected_by_id.contains_key(**id))
            .collect();
        if !missing.is_empty() || !unexpected.is_empty() {
            missing.sort();
            unexpected.sort();
            failures.push(format!(
                "{name}: zone set of junction {junction} changed — missing {missing:?}, \
                 unexpected new {unexpected:?}"
            ));
        }

        let mut changed: Vec<(String, f64, f64)> = expected_by_id
            .iter()
            .filter_map(|(id, expected)| {
                let actual = actual_by_id.get(id.as_str())?;
                let fraction = differing_area_fraction(expected, actual);
                (fraction > MAX_DIFFERING_AREA_FRACTION)
                    .then(|| (id.clone(), fraction, expected.unsigned_area()))
            })
            .collect();
        if !changed.is_empty() {
            changed.sort_by(|a, b| b.1.total_cmp(&a.1));
            let detail: Vec<String> = changed
                .iter()
                .map(|(id, fraction, area)| {
                    format!(
                        "    {id}: {:.1}% of its own {area:.1}m² differs",
                        fraction * 100.0
                    )
                })
                .collect();
            failures.push(format!(
                "{name}: shape changed for {} zone(s):\n{}",
                changed.len(),
                detail.join("\n")
            ));
        }
    }
    let failed: HashSet<&str> = failures
        .iter()
        .filter_map(|f| f.split(':').next())
        .collect();
    assert!(
        failures.is_empty(),
        "{} fixture(s) failed:\n{}",
        failed.len(),
        failures.join("\n\n")
    );
}

//! Table-driven: every subdirectory of `tests/fixtures/` holding both a
//! `network.net.xml` and an `expected.geojson` gets compared against a
//! fresh run automatically. Adding a new fixture never touches this file —
//! see `tests/fixtures/README.md` for the exact steps.
//!
//! Real regression coverage for "is this still the same zone, in the same
//! place, covering the same ground", which none of `waiting_zones`' own
//! property-style unit tests (simple, no self-intersection, roughly the
//! right size) can give: two visibly different shapes can both pass a
//! property check.
//!
//! # Why this compares areas, not vertices
//!
//! It used to compare every coordinate of every ring, to within 1e-9
//! degrees — about a tenth of a millimetre. That is not a contract this
//! crate can keep, or should try to. A zone's polygon is the end of a long
//! chain of boolean operations over floating-point geometry
//! (`geojson_output`), and its output is *stable* but not *canonical*: a
//! `HashMap` iteration order feeding a union in a different sequence, a
//! `simplify` pass keeping a neighbouring point instead, a cleanup pass
//! dropping a vertex that contributed a square millimetre — each changes
//! the vertex list while describing the same piece of road. Under exact
//! comparison every one of those reads as a regression, so the fixtures
//! failed constantly for reasons nobody could act on, and the only
//! available response was to regenerate them — which silently accepts
//! whatever else changed too, and is exactly how a golden-file suite stops
//! being worth reading.
//!
//! What actually matters to every consumer of this output is the *ground a
//! zone claims*: a client geofences a GPS position against the polygon
//! (`resolver::catalogue`), and two polygons that enclose the same area to
//! within a fraction of a percent are the same zone to it, whatever their
//! vertex lists look like. So that is what's compared — the area the two
//! shapes disagree about, as a fraction of the zone's own size. A moved
//! vertex is invisible to it; a zone that grew an extra block, lost its
//! stop-line end to a cut, or shifted onto the next lane is not, because
//! all three move real area.
//!
//! The *set* of zone ids is still compared exactly: an id is a discrete
//! name, not a measurement, and a zone appearing, vanishing, or being
//! renamed is always worth a human's attention.
//!
//! # Excluded zones
//!
//! `tests/fixtures/<name>/excluded_zones.json` (optional; a bare JSON
//! array of `waiting_zone_id` strings) names zones this fixture's own
//! `expected.geojson` should not be trusted for at all — removed from
//! both sides of the comparison, rather than expected to keep matching
//! or flagged as a permanent, growing "zone set changed" failure.
//!
//! This exists for a real, structural gap `tests/fixtures/README.md`'s
//! own "Adding a fixture" step 2 already warns about but has no tooling
//! for: "Guardar intersección" (see `viz/README.md`) captures *every*
//! zone near a junction, not just the one a human actually reviewed, and
//! the small-radius extract behind a fixture is only ever verified
//! faithful for that one deliberate target — confirmed on real Barcelona
//! data, `-37442552#5_straight` swept into `-207322888_10_straight`'s own
//! fixture as an incidental bystander, where the extract's radius doesn't
//! reach far enough for `resolve_overlaps` to see the same neighbours the
//! full network does, producing a spurious hole (45m² instead of the
//! real network's own 23m², confirmed identical whether regenerated from
//! this fixture's own network.net.xml or freshly from
//! `data/barcelona/barcelona.net.xml`). That isn't a `generate()` bug to
//! fix here — it's this one fixture's own extract not being faithful for
//! this one bystander — so the fix is to stop vouching for it, not to
//! edit its geometry (which `viz.html`'s "quitar de expected" button
//! does, via `serve.py`'s own `/api/remove_expected_zone`).

use anyhow::{Context, Result};
use geo::{Area, BooleanOps, Coord, LineString, MultiPolygon, Polygon};
use std::collections::HashMap;
use std::path::{Path, PathBuf};

/// Every subdirectory of `tests/fixtures/` that looks like a fixture (has
/// its own `network.net.xml`), sorted for a deterministic failure order.
fn fixture_dirs() -> Vec<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/fixtures");
    let mut dirs: Vec<PathBuf> = std::fs::read_dir(&root)
        .expect("reading tests/fixtures")
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.is_dir() && path.join("network.net.xml").exists())
        .collect();
    dirs.sort();
    dirs
}

/// The exact `FeatureCollection` `geojson_output::write` itself would
/// produce for `dir`'s own `network.net.xml`, run through the CLI's own
/// default settings (`max_zone_length: None`, `stop_at_complex_intersections:
/// false`) — a fixture is about the *geometry* pipeline, not about
/// exercising a non-default flag; add a second `network.net.xml`-derived
/// fixture if a flag itself ever needs its own regression coverage.
fn generate_collection(dir: &Path) -> Result<geojson::FeatureCollection> {
    let network_path = dir.join("network.net.xml");
    let network = sumo_types::read_network(&network_path)
        .with_context(|| format!("reading {network_path:?}"))?;
    let zones = waiting_zones::zone_generator::generate(&network, None, false);
    waiting_zones::geojson_output::to_feature_collection(&network, &zones)
        .with_context(|| format!("building collection for {network_path:?}"))
}

fn by_zone_id(collection: &geojson::FeatureCollection) -> HashMap<String, &geojson::Feature> {
    collection
        .features
        .iter()
        .map(|feature| {
            let id = feature
                .property("waiting_zone_id")
                .and_then(|v| v.as_str())
                .expect("every feature has a waiting_zone_id property")
                .to_string();
            (id, feature)
        })
        .collect()
}

/// Every `waiting_zone_id` `dir`'s own `excluded_zones.json` names, or
/// empty if that file doesn't exist — see this file's own module docs
/// ("Excluded zones") for what it's for. Absent for every fixture that
/// predates this mechanism, so this is a no-op everywhere except a
/// fixture `viz.html`'s "quitar de expected" button has actually touched.
fn excluded_zone_ids(dir: &Path) -> std::collections::HashSet<String> {
    let path = dir.join("excluded_zones.json");
    let Ok(json) = std::fs::read_to_string(&path) else {
        return std::collections::HashSet::new();
    };
    serde_json::from_str(&json)
        .unwrap_or_else(|error| panic!("{path:?} doesn't parse as a JSON array of strings: {error}"))
}

/// Metres per degree of longitude and of latitude at Barcelona's own
/// latitude. Areas are compared in real m², not in square degrees: a
/// degree of longitude is ~84km here against a degree of latitude's
/// ~111km, so a square-degree area is stretched by a third in one axis
/// and means nothing physical.
const METERS_PER_DEGREE_LON: f64 = 84_000.0;
const METERS_PER_DEGREE_LAT: f64 = 111_000.0;

/// `feature`'s own geometry as a `geo::MultiPolygon` in locally flat
/// metres.
///
/// Accepts a `Polygon` or a `MultiPolygon`. `geojson_output` emits only
/// the former today, but fixtures committed before that change hold the
/// latter, and a fixture whose geometry is genuinely unchanged should not
/// fail over how it happens to be spelled — that would be exactly the
/// kind of unactionable failure this file exists to stop producing.
fn geometry_in_meters(feature: &geojson::Feature) -> MultiPolygon<f64> {
    let ring = |positions: &[geojson::Position]| {
        LineString::from(
            positions
                .iter()
                .map(|p| Coord { x: p[0] * METERS_PER_DEGREE_LON, y: p[1] * METERS_PER_DEGREE_LAT })
                .collect::<Vec<_>>(),
        )
    };
    let polygon = |rings: &[Vec<geojson::Position>]| {
        rings.split_first().map(|(exterior, interiors)| {
            Polygon::new(ring(exterior), interiors.iter().map(|r| ring(r)).collect())
        })
    };

    let parts = match feature.geometry.as_ref().map(|g| &g.value) {
        Some(geojson::GeometryValue::Polygon { coordinates }) => {
            polygon(coordinates).into_iter().collect()
        }
        Some(geojson::GeometryValue::MultiPolygon { coordinates }) => {
            coordinates.iter().filter_map(|rings| polygon(rings)).collect()
        }
        _ => Vec::new(),
    };
    MultiPolygon::new(parts)
}

/// How much of a zone's own area the committed and freshly generated
/// shapes may disagree about before that's a real change.
///
/// Set against what actually reaches a consumer rather than against
/// floating-point noise: a client's geofence (`resolver::catalogue`) asks
/// whether one GPS position is inside one polygon, and moving half a
/// percent of a zone's area cannot change that answer for any position a
/// GPS fix could even resolve. Meanwhile every regression worth catching
/// here moves far more than half a percent — an extension reaching a block
/// further, an overlap cut taking the stop-line end, a zone landing on the
/// neighbouring lane all move tens of percent, and show up as
/// unmistakable numbers in the failure message rather than as a
/// coordinate diff nobody can read.
const MAX_DIFFERING_AREA_FRACTION: f64 = 0.005;

/// The share of `expected`'s own area that `expected` and `actual`
/// disagree about — `0.0` for identical ground, `1.0` for shapes that
/// don't overlap at all. Measured symmetrically (ground in one and not
/// the other, either way round), so a zone that only *grew* is caught
/// exactly as a zone that only shrank.
fn differing_area_fraction(expected: &MultiPolygon<f64>, actual: &MultiPolygon<f64>) -> f64 {
    let reference = expected.unsigned_area().max(actual.unsigned_area());
    if reference <= 0.0 {
        // Two empty shapes agree; one empty and one not disagree totally.
        return if actual.unsigned_area() > 0.0 { 1.0 } else { 0.0 };
    }
    expected.difference(actual).unsigned_area().max(actual.difference(expected).unsigned_area())
        / reference
}

#[test]
fn every_fixture_matches_its_own_expected_geojson() {
    let mut failures = Vec::new();
    for dir in fixture_dirs() {
        let name = dir.file_name().unwrap().to_string_lossy().into_owned();
        let expected_path = dir.join("expected.geojson");
        let Ok(expected_json) = std::fs::read_to_string(&expected_path) else {
            failures.push(format!(
                "{name}: no expected.geojson (run `cargo test --release -- --ignored \
                 regenerate_expected_geojson` after adding this fixture, then visually \
                 confirm it in viz/viz.html before committing it — see tests/fixtures/README.md)"
            ));
            continue;
        };
        let expected: geojson::FeatureCollection = match serde_json::from_str(&expected_json) {
            Ok(collection) => collection,
            Err(error) => {
                failures.push(format!("{name}: expected.geojson doesn't parse: {error}"));
                continue;
            }
        };
        let actual = match generate_collection(&dir) {
            Ok(collection) => collection,
            Err(error) => {
                failures.push(format!("{name}: {error:#}"));
                continue;
            }
        };

        let excluded = excluded_zone_ids(&dir);
        let mut expected_by_id = by_zone_id(&expected);
        let mut actual_by_id = by_zone_id(&actual);
        expected_by_id.retain(|id, _| !excluded.contains(id));
        actual_by_id.retain(|id, _| !excluded.contains(id));

        let mut missing: Vec<&String> =
            expected_by_id.keys().filter(|id| !actual_by_id.contains_key(*id)).collect();
        let mut unexpected: Vec<&String> =
            actual_by_id.keys().filter(|id| !expected_by_id.contains_key(*id)).collect();
        if !missing.is_empty() || !unexpected.is_empty() {
            missing.sort();
            unexpected.sort();
            failures.push(format!(
                "{name}: zone set changed — missing {missing:?}, unexpected new {unexpected:?}"
            ));
            continue;
        }

        let mut changed: Vec<(String, f64, f64)> = expected_by_id
            .iter()
            .filter_map(|(id, expected_feature)| {
                let expected_geometry = geometry_in_meters(expected_feature);
                let actual_geometry = geometry_in_meters(actual_by_id[id]);
                let fraction = differing_area_fraction(&expected_geometry, &actual_geometry);
                (fraction > MAX_DIFFERING_AREA_FRACTION).then(|| {
                    (id.clone(), fraction, expected_geometry.unsigned_area())
                })
            })
            .collect();
        if !changed.is_empty() {
            changed.sort_by(|a, b| b.1.total_cmp(&a.1));
            let detail = changed
                .iter()
                .map(|(id, fraction, area)| {
                    format!("    {id}: {:.1}% of its own {area:.1}m² differs", fraction * 100.0)
                })
                .collect::<Vec<_>>()
                .join("\n");
            failures.push(format!("{name}: shape changed for {} zone(s):\n{detail}", changed.len()));
        }
    }
    assert!(failures.is_empty(), "{} fixture(s) failed:\n{}", failures.len(), failures.join("\n\n"));
}

/// Regenerates every fixture's own `expected.geojson` from the current
/// code. Not run by a plain `cargo test` (see this test's own `#[ignore]`)
/// — run explicitly with `cargo test --release -- --ignored
/// regenerate_expected_geojson`, and *visually confirm* the result in
/// `viz/viz.html` (`viz/README.md`'s own "visualizing a test fixture"
/// section has the exact steps) before ever committing what this
/// overwrites, the same as any other golden file: this only proves the
/// output is deterministic, never that it's correct.
#[test]
#[ignore]
fn regenerate_expected_geojson() {
    for dir in fixture_dirs() {
        let collection = generate_collection(&dir).expect("generating collection");
        let json = serde_json::to_string_pretty(&collection).expect("serializing collection");
        let path = dir.join("expected.geojson");
        std::fs::write(&path, json).expect("writing expected.geojson");
        println!("regenerated {path:?}");
    }
}

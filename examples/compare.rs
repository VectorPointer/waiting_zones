//! Compares two waiting-zone catalogues (e.g. this OSM-only generator's
//! output against the SUMO-based one) for one class of zone.
//!
//!   cargo run --release --example compare -- <candidate.geojson> <reference.geojson>
//!
//! Reports, within a GPS-scale tolerance:
//! - recall: reference zones whose stop line some candidate zone covers,
//! - precision: candidate zones whose stop line some reference zone covers,
//! - ground: how much of the reference's total area the candidate covers,
//!   and how much of the candidate's lies outside the reference.

use std::path::Path;

use anyhow::{Context, Result};
use geo::{Area, BooleanOps, Coord, Distance, Euclidean, LineString, MultiPolygon, Point, Polygon};
use geojson::{FeatureCollection, GeoJson, GeometryValue, Position};

const TOLERANCE_METERS: f64 = 3.0;

struct Zone {
    polygon: MultiPolygon<f64>,
    stop_line: Point<f64>,
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let [_, candidate, reference] = args.as_slice() else {
        anyhow::bail!("usage: compare <candidate.geojson> <reference.geojson>");
    };
    let candidate = load(Path::new(candidate))?;
    let reference = load(Path::new(reference))?;

    let covered = |point: Point<f64>, zones: &[Zone]| {
        zones
            .iter()
            .any(|z| Euclidean.distance(&point, &z.polygon) <= TOLERANCE_METERS)
    };
    let recall = reference
        .iter()
        .filter(|z| covered(z.stop_line, &candidate))
        .count();
    let precision = candidate
        .iter()
        .filter(|z| covered(z.stop_line, &reference))
        .count();

    let union = |zones: &[Zone]| {
        zones.iter().fold(MultiPolygon::new(Vec::new()), |acc, z| {
            acc.union(&z.polygon)
        })
    };
    let (candidate_ground, reference_ground) = (union(&candidate), union(&reference));
    let shared = candidate_ground
        .intersection(&reference_ground)
        .unsigned_area();

    println!(
        "zones: candidate {}, reference {}",
        candidate.len(),
        reference.len()
    );
    println!(
        "recall:    {recall}/{} reference stop lines covered ({:.0}%)",
        reference.len(),
        100.0 * recall as f64 / reference.len().max(1) as f64
    );
    println!(
        "precision: {precision}/{} candidate stop lines covered ({:.0}%)",
        candidate.len(),
        100.0 * precision as f64 / candidate.len().max(1) as f64
    );
    println!(
        "ground:    candidate covers {:.0}% of reference area ({:.0}m² of {:.0}m²); {:.0}% of candidate area is outside the reference",
        100.0 * shared / reference_ground.unsigned_area().max(1.0),
        shared,
        reference_ground.unsigned_area(),
        100.0 * (1.0 - shared / candidate_ground.unsigned_area().max(1.0)),
    );
    Ok(())
}

/// Lon/lat to roughly-flat metres at Barcelona's latitude.
fn local(position: &[f64]) -> Coord<f64> {
    Coord {
        x: position[0] * 84_000.0,
        y: position[1] * 111_000.0,
    }
}

fn load(path: &Path) -> Result<Vec<Zone>> {
    let text =
        std::fs::read_to_string(path).with_context(|| format!("reading {}", path.display()))?;
    let collection = FeatureCollection::try_from(text.parse::<GeoJson>()?)?;
    let polygon = |rings: &Vec<Vec<Position>>| {
        let ring =
            |r: &Vec<Position>| LineString::new(r.iter().map(|p| local(p.as_slice())).collect());
        Polygon::new(ring(&rings[0]), rings[1..].iter().map(ring).collect())
    };
    collection
        .features
        .iter()
        .map(|feature| {
            let geometry = feature
                .geometry
                .as_ref()
                .context("feature without geometry")?;
            let polygon = match &geometry.value {
                GeometryValue::Polygon { coordinates } => {
                    MultiPolygon::new(vec![polygon(coordinates)])
                }
                GeometryValue::MultiPolygon { coordinates } => {
                    MultiPolygon::new(coordinates.iter().map(polygon).collect())
                }
                _ => anyhow::bail!("unexpected geometry"),
            };
            let stop: Vec<f64> = serde_json::from_value(
                feature
                    .property("stop_line")
                    .context("no stop_line")?
                    .clone(),
            )?;
            Ok(Zone {
                polygon,
                stop_line: Point::from(local(&stop)),
            })
        })
        .collect()
}

use anyhow::{Context, Result};
use geo::{Area, Coord, LineString, MultiPolygon, Polygon as GeoPolygon};
use geojson::{Feature, FeatureCollection, Geometry, JsonObject, Position};
use std::{collections::HashMap, path::Path};
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{Lane, Network, Point};
use crate::geojson_output::geometry::{centroid, single_successors, zone_modes, zone_polygon};
use crate::geojson_output::overlaps::resolve_overlaps;
use crate::geojson_output::reprojection::{distance_from_start, point_and_tangent_at, Reprojector, MIN_DRAWN_LANE_LENGTH_METERS};

const MIN_INTERIOR_RING_AREA_M2: f64 = 1.0;

pub fn stop_line_point(zone: &E3Detector, lanes: &HashMap<&str, &Lane>) -> Result<Point> {
    let mut stop_points = Vec::with_capacity(zone.exits.len());
    for exit in &zone.exits {
        let lane = lanes.get(exit.lane.0.as_str()).copied().with_context(|| {
            format!("zone {:?} references lane {:?}, which isn't in the network", zone.id, exit.lane)
        })?;
        let exit_distance = distance_from_start(exit.position, lane.length);
        stop_points.push(point_and_tangent_at(&lane.shape, exit_distance).0);
    }
    Ok(centroid(&stop_points))
}

pub fn build_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    polygon: &MultiPolygon<f64>,
    reproject: &Reprojector,
) -> Result<Feature> {
    let stop_line = reproject.to_lon_lat(stop_line_point(zone, lanes)?)?;

    let ring = |line: &LineString<f64>| -> Result<Vec<Position>> {
        line.coords().map(|c| reproject.to_lon_lat(Point { x: c.x, y: c.y, z: 0.0 }).map(Position::from)).collect()
    };
    let polygons = polygon
        .0
        .iter()
        .map(|part| {
            let mut rings = vec![ring(part.exterior())?];
            for interior in part.interiors() {
                if GeoPolygon::new(interior.clone(), Vec::new()).unsigned_area() >= MIN_INTERIOR_RING_AREA_M2 {
                    rings.push(ring(interior)?);
                }
            }
            Ok(rings)
        })
        .collect::<Result<Vec<_>>>()?;

    let mut properties = JsonObject::new();
    properties.insert("waiting_zone_id".to_string(), zone.id.0.clone().into());
    properties.insert("stop_line".to_string(), serde_json::json!(stop_line));
    properties.insert("modes".to_string(), serde_json::json!(zone_modes(zone, lanes)));

    let mut feature = Feature::from(Geometry::new_multi_polygon(polygons));
    feature.properties = Some(properties);
    Ok(feature)
}

pub fn zone_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    reproject: &Reprojector,
    pad_meters: f64,
) -> Result<Feature> {
    build_feature(zone, lanes, &zone_polygon(zone, lanes, successors, pad_meters)?, reproject)
}

pub fn to_feature_collection(network: &Network, zones: &[E3Detector]) -> Result<FeatureCollection> {
    let reproject = Reprojector::new(&network.location)?;
    let lanes: HashMap<&str, &Lane> = network
        .edges
        .iter()
        .flat_map(|edge| &edge.lanes)
        .map(|lane| (lane.id.0.as_str(), lane))
        .collect();
    let successors = single_successors(network);

    let mut polygons = zones
        .iter()
        .map(|zone| zone_polygon(zone, &lanes, &successors, MIN_DRAWN_LANE_LENGTH_METERS))
        .collect::<Result<Vec<_>>>()?;
    // `resolve_overlaps`'s own reference point per zone — see
    // `keep_part_near`'s own docs for what it's for.
    let stop_points = zones
        .iter()
        .map(|zone| stop_line_point(zone, &lanes).map(|p| Coord { x: p.x, y: p.y }))
        .collect::<Result<Vec<_>>>()?;

    resolve_overlaps(zones, &lanes, &successors, &stop_points, &mut polygons)?;

    let features = zones
        .iter()
        .zip(&polygons)
        .map(|(zone, polygon)| build_feature(zone, &lanes, polygon, &reproject))
        .collect::<Result<Vec<_>>>()?;

    Ok(FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    })
}

pub fn write(path: &Path, network: &Network, zones: &[E3Detector]) -> Result<()> {
    let collection = to_feature_collection(network, zones)?;
    let json = serde_json::to_string_pretty(&collection)
        .context("could not serialize waiting zones as GeoJSON")?;
    std::fs::write(path, json).with_context(|| format!("could not write output file: {path:?}"))
}

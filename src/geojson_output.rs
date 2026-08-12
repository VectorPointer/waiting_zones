//! Writes waiting zones out as a client-facing GeoJSON `FeatureCollection`,
//! reprojected to WGS84 lon/lat, for a client that geofences against it
//! from its own GPS.
//!
//! Scope, deliberately: this emits geometry (`perimeter`, `stop_line`) and
//! the id [`crate::zone_generator`] already assigns
//! (`waiting_zone_id`). `intersection_id`, `modes` and `heading` aren't
//! modeled anywhere in this crate or in `sumo_types` yet, so they aren't
//! invented here either — seeding a property with data nobody computed
//! would be worse than leaving it out.
//!
//! ## Reprojection
//!
//! A `.net.xml`'s coordinates are **not** lon/lat: `netconvert` projects the
//! source `.osm` into a local, usually metric CRS (declared as a PROJ4
//! string in `location/@projParameter`) and then offsets it
//! (`location/@netOffset`) so the network sits near the origin. GeoJSON — and
//! a client's GPS — assume WGS84 lon/lat, so every coordinate here is
//! reprojected: the `netOffset` translation is undone first, then
//! [`proj4rs`] inverts the projection. A network with no projection at all
//! (`projParameter="!"`, e.g. a synthetic/test network) has nothing to
//! invert — [`write`] fails outright rather than emit coordinates dressed up
//! as lon/lat that aren't.
//!
//! ## Geometry
//!
//! [`E3Detector`]'s gates carry a lane and a linear position on it, not a
//! polygon — so each lane in a zone contributes its own rectangle (the
//! lane's shape between the entry and exit stations, offset left/right by
//! half the lane's width), and a zone spanning several lanes (one shared
//! signal group) becomes a `MultiPolygon` of those rectangles rather than a
//! single merged polygon. Merging them into one exact ring is a polygon
//! union — real computational geometry this module does not do — and a
//! `MultiPolygon` already answers the question a client actually asks
//! (point-in-*any*-of-these-polygons) without it.

use anyhow::{Context, Result, bail};
use geojson::{Feature, FeatureCollection, Geometry, JsonObject, Position};
use proj4rs::proj::Proj;
use proj4rs::transform::transform;
use std::collections::HashMap;
use std::path::Path;
use sumo_types::additional::domain::{E3Detector, LanePosition};
use sumo_types::domain::{Lane, Location, Network, Point, Projection, Shape};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

/// The one CRS every coordinate in the output is reprojected into: WGS84
/// lon/lat, in that axis order (matching GeoJSON's `[lon, lat]`, not
/// `[lat, lon]`).
const WGS84_LONLAT: &str = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";

/// Reprojects a network's local/projected coordinates to WGS84 lon/lat.
/// Built once per network — parsing `location/@projParameter` and the WGS84
/// target is the expensive part, not the transform itself — and reused for
/// every point.
struct Reprojector {
    from: Proj,
    to: Proj,
    net_offset: Point,
}

impl Reprojector {
    fn new(location: &Location) -> Result<Self> {
        let Projection::Proj4(proj_string) = &location.projection else {
            bail!(
                "network is not georeferenced (location/@projParameter is \"!\"): \
                 GeoJSON needs real-world coordinates, and an unprojected network has none to give"
            );
        };
        let from = Proj::from_proj_string(proj_string).with_context(|| {
            format!("invalid PROJ4 string in .net.xml location: {proj_string:?}")
        })?;
        let to =
            Proj::from_proj_string(WGS84_LONLAT).expect("WGS84_LONLAT is a valid PROJ4 string");
        Ok(Self {
            from,
            to,
            net_offset: location.net_offset,
        })
    }

    /// `point` is in the network's own coordinates, straight from a
    /// `.net.xml` shape. `netOffset` is undone before the inverse
    /// projection, mirroring how `netconvert` applied it going the other
    /// way when it built the network.
    fn to_lon_lat(&self, point: Point) -> Result<[f64; 2]> {
        let mut coords = (
            point.x - self.net_offset.x,
            point.y - self.net_offset.y,
            0.0,
        );
        transform(&self.from, &self.to, &mut coords)
            .with_context(|| format!("could not reproject network point {point:?} to WGS84"))?;
        Ok([coords.0.to_degrees(), coords.1.to_degrees()])
    }
}

/// The point on `shape` at arc-length `distance` from its start (clamped to
/// the shape's own length), together with the unit tangent of the segment
/// it falls on — used to offset left/right by half a lane's width.
///
/// Falls back to an arbitrary tangent `(1.0, 0.0)` on a degenerate
/// (empty or single-point) shape rather than panicking: real `netconvert`
/// output always writes a proper polyline, so this only guards a malformed
/// `.net.xml`, not a case this crate is expected to handle precisely.
fn point_and_tangent_at(shape: &Shape, distance: Length) -> (Point, (f64, f64)) {
    let target = distance.get::<meter>().max(0.0);
    let mut travelled = 0.0;

    for window in shape.0.windows(2) {
        let [a, b] = window else {
            unreachable!("windows(2) always yields length-2 slices")
        };
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let segment_length = dx.hypot(dy);
        if segment_length > 0.0 && travelled + segment_length < target {
            travelled += segment_length;
            continue;
        }

        let t = if segment_length > 0.0 {
            (target - travelled) / segment_length
        } else {
            0.0
        };
        let point = Point {
            x: a.x + dx * t,
            y: a.y + dy * t,
            z: a.z + (b.z - a.z) * t,
        };
        let tangent = if segment_length > 0.0 {
            (dx / segment_length, dy / segment_length)
        } else {
            (1.0, 0.0)
        };
        return (point, tangent);
    }

    (shape.0.last().copied().unwrap_or_default(), (1.0, 0.0))
}

/// `position` resolved to a distance from the lane's start, given the
/// lane's own `length` (needed to resolve [`LanePosition::FromEnd`]).
fn distance_from_start(position: LanePosition, lane_length: Length) -> Length {
    match position {
        LanePosition::FromStart(distance) => distance,
        LanePosition::FromEnd(distance) => lane_length - distance,
    }
}

/// `point` offset perpendicular to `tangent` by `distance` — to the left of
/// the direction of travel when `distance` is positive, to the right when
/// negative. Only `x`/`y` move: `z` (elevation) isn't touched by a
/// perpendicular offset.
fn offset_perpendicular(point: Point, tangent: (f64, f64), distance: Length) -> Point {
    let (dx, dy) = tangent;
    let d = distance.get::<meter>();
    Point {
        x: point.x - dy * d,
        y: point.y + dx * d,
        z: point.z,
    }
}

/// The rectangle a single lane contributes to a zone's `MultiPolygon`: the
/// lane's shape between `entry`/`exit` (arc-length from the lane's start),
/// offset left/right by half of `lane`'s width, reprojected to lon/lat and
/// closed (first position repeated last, as GeoJSON linear rings require).
fn lane_ring(
    lane: &Lane,
    entry: Length,
    exit: Length,
    reproject: &Reprojector,
) -> Result<Vec<Position>> {
    let half_width = lane.width / 2.0;
    let (entry_point, entry_tangent) = point_and_tangent_at(&lane.shape, entry);
    let (exit_point, exit_tangent) = point_and_tangent_at(&lane.shape, exit);

    let corners = [
        offset_perpendicular(entry_point, entry_tangent, half_width),
        offset_perpendicular(exit_point, exit_tangent, half_width),
        offset_perpendicular(exit_point, exit_tangent, -half_width),
        offset_perpendicular(entry_point, entry_tangent, -half_width),
    ];

    let mut ring = corners
        .into_iter()
        .map(|corner| reproject.to_lon_lat(corner).map(Position::from))
        .collect::<Result<Vec<_>>>()?;
    ring.push(ring[0].clone());
    Ok(ring)
}

/// The average of `points`, in the network's own local coordinates — a
/// single representative point for a zone's `stop_line`, even when the zone
/// spans several lanes (and so several individual stop lines).
fn centroid(points: &[Point]) -> Point {
    let count = points.len().max(1) as f64;
    let sum = points.iter().fold(Point::default(), |acc, p| Point {
        x: acc.x + p.x,
        y: acc.y + p.y,
        z: acc.z + p.z,
    });
    Point {
        x: sum.x / count,
        y: sum.y / count,
        z: sum.z / count,
    }
}

/// Builds `zone`'s `Feature`: a `MultiPolygon` geometry (one rectangle per
/// lane in the zone) and a `waiting_zone_id`/`stop_line` pair of properties.
fn zone_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    reproject: &Reprojector,
) -> Result<Feature> {
    let mut polygons = Vec::with_capacity(zone.entries.len());
    let mut stop_points = Vec::with_capacity(zone.entries.len());

    for (entry, exit) in zone.entries.iter().zip(&zone.exits) {
        let lane = *lanes.get(entry.lane.0.as_str()).with_context(|| {
            format!(
                "zone {:?} references lane {:?}, which isn't in the network",
                zone.id, entry.lane
            )
        })?;
        let entry_distance = distance_from_start(entry.position, lane.length);
        let exit_distance = distance_from_start(exit.position, lane.length);

        polygons.push(vec![lane_ring(
            lane,
            entry_distance,
            exit_distance,
            reproject,
        )?]);
        stop_points.push(point_and_tangent_at(&lane.shape, exit_distance).0);
    }

    let stop_line = reproject.to_lon_lat(centroid(&stop_points))?;

    let mut properties = JsonObject::new();
    properties.insert("waiting_zone_id".to_string(), zone.id.0.clone().into());
    properties.insert("stop_line".to_string(), serde_json::json!(stop_line));

    let mut feature = Feature::from(Geometry::new_multi_polygon(polygons));
    feature.properties = Some(properties);
    Ok(feature)
}

/// Converts `zones` (as generated by [`crate::zone_generator`] from
/// `network`) into a GeoJSON `FeatureCollection`, one feature per zone.
///
/// Fails if `network` isn't georeferenced (see the module docs) or if a
/// zone references a lane `network` doesn't have — the latter would mean
/// `zones` wasn't actually generated from this `network`.
pub fn to_feature_collection(network: &Network, zones: &[E3Detector]) -> Result<FeatureCollection> {
    let reproject = Reprojector::new(&network.location)?;
    let lanes: HashMap<&str, &Lane> = network
        .edges
        .iter()
        .flat_map(|edge| &edge.lanes)
        .map(|lane| (lane.id.0.as_str(), lane))
        .collect();

    let features = zones
        .iter()
        .map(|zone| zone_feature(zone, &lanes, &reproject))
        .collect::<Result<Vec<_>>>()?;

    Ok(FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    })
}

/// Writes `zones` to `path` as a complete GeoJSON `FeatureCollection`.
pub fn write(path: &Path, network: &Network, zones: &[E3Detector]) -> Result<()> {
    let collection = to_feature_collection(network, zones)?;
    let json = serde_json::to_string_pretty(&collection)
        .context("could not serialize waiting zones as GeoJSON")?;
    std::fs::write(path, json).with_context(|| format!("could not write output file: {path:?}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use sumo_types::additional::domain::{DetectorGate, DetectorId, LaneRef};
    use sumo_types::domain::{Boundary, Edge, EdgeFunction, EdgeId, LaneId, LaneIndex};
    use sumo_types::uom::si::velocity::meter_per_second;

    /// A straight, north-pointing lane 20m long, its shape running from
    /// `(0, 0)` to `(0, 20)` in a UTM-like local CRS.
    fn straight_lane(id: &str, width_m: f64) -> Lane {
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(10.0),
            length: Length::new::<meter>(20.0),
            width: Length::new::<meter>(width_m),
            end_offset: Length::new::<meter>(0.0),
            shape: Shape(vec![
                Point {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                },
                Point {
                    x: 0.0,
                    y: 20.0,
                    z: 0.0,
                },
            ]),
            allow: vec![],
            disallow: vec![],
        }
    }

    fn utm_31n_network(lanes: Vec<Lane>) -> Network {
        Network {
            location: Location {
                // Chosen so the local shape above (x in [0, 0], y in [0,
                // 20]) sits near Barcelona rather than off the coast of
                // Ghana (UTM 31N's origin): offset applied on top of a real
                // UTM 31N easting/northing for a point close to it.
                net_offset: Point {
                    x: -430_000.0,
                    y: -4_582_000.0,
                    z: 0.0,
                },
                converted_boundary: Boundary::default(),
                original_boundary: Boundary::default(),
                projection: Projection::Proj4(
                    "+proj=utm +zone=31 +ellps=WGS84 +datum=WGS84 +units=m +no_defs".into(),
                ),
            },
            edges: vec![Edge {
                id: EdgeId("e0".into()),
                function: EdgeFunction::Normal,
                from: None,
                to: None,
                name: None,
                priority: None,
                length: None,
                shape: None,
                spread_type: None,
                lanes,
            }],
            junctions: vec![],
            connections: vec![],
            roundabouts: vec![],
            traffic_light_programs: vec![],
        }
    }

    fn gate(lane: &str, position: LanePosition) -> DetectorGate {
        DetectorGate {
            lane: LaneRef(lane.into()),
            position,
            friendly_position: Some(true),
        }
    }

    fn zone(id: &str, lane: &str) -> E3Detector {
        E3Detector {
            id: DetectorId(id.into()),
            entries: vec![gate(
                lane,
                LanePosition::FromStart(Length::new::<meter>(0.0)),
            )],
            exits: vec![gate(
                lane,
                LanePosition::FromStart(Length::new::<meter>(20.0)),
            )],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
        }
    }

    #[test]
    fn rejects_an_unprojected_network() {
        let network = Network {
            location: Location::default(), // Projection::None
            ..utm_31n_network(vec![])
        };

        let err = to_feature_collection(&network, &[]).unwrap_err();
        assert!(err.to_string().contains("not georeferenced"), "{err}");
    }

    #[test]
    fn reprojects_the_lane_shape_into_a_plausible_lon_lat_box() {
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(collection.features.len(), 1);

        let feature = &collection.features[0];
        assert_eq!(
            feature.property("waiting_zone_id").unwrap(),
            &serde_json::json!("j0_0")
        );

        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &feature.geometry.as_ref().unwrap().value
        else {
            panic!(
                "expected a MultiPolygon geometry, got {:?}",
                feature.geometry
            );
        };
        assert_eq!(coordinates.len(), 1, "one lane -> one polygon");
        let ring = &coordinates[0][0];
        assert_eq!(ring.first(), ring.last(), "a linear ring must close");

        // Barcelona-ish: UTM 31N puts it around 2°E, 41°N.
        for position in ring {
            assert!(
                (1.0..3.0).contains(&position[0]),
                "lon {} out of range",
                position[0]
            );
            assert!(
                (40.0..42.0).contains(&position[1]),
                "lat {} out of range",
                position[1]
            );
        }

        let stop_line = feature.property("stop_line").unwrap().as_array().unwrap();
        assert!((1.0..3.0).contains(&stop_line[0].as_f64().unwrap()));
        assert!((40.0..42.0).contains(&stop_line[1].as_f64().unwrap()));
    }

    #[test]
    fn offsets_the_ring_by_roughly_half_the_lane_width() {
        // A lane running due north: the perpendicular offset is purely in
        // x (easting), so converting the resulting lon/lat difference back
        // to metres at this latitude should land close to `width / 2` on
        // each side -- not an exact match (that needs the local metres/degree
        // scale factor), but within the same order of magnitude for a
        // narrow lane.
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        // corners[0] and corners[3] are the two entry-side corners, on
        // opposite sides of the centreline -> their longitudes should
        // differ measurably (not collapse onto a single centreline point).
        let lon_span = (ring[0][0] - ring[3][0]).abs();
        assert!(
            lon_span > 0.00001,
            "expected a visible offset, got {lon_span}"
        );
        assert!(
            lon_span < 0.001,
            "offset implausibly large for a 3.2m lane: {lon_span}"
        );
    }
}

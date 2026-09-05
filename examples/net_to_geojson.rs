//! Debug-only tool for `viz/viz.html`: dumps every lane of a `.net.xml` as
//! GeoJSON in the same WGS84 lon/lat frame `geojson_output` reprojects
//! waiting zones into, so the viewer can overlay the real network geometry
//! next to the generated zones and compare them directly instead of trusting
//! OSM tiles to agree with what SUMO actually modelled.
//!
//! Emits two features per lane: its centerline (`geometry_kind: "centerline"`,
//! a `LineString`) and its true footprint (`geometry_kind: "footprint"`, a
//! `Polygon` buffered by the lane's own `width` — the same
//! `i_overlay` stroke-offset technique `geometry.rs::buffer_shape` uses for
//! zones themselves) — the footprint is what actually answers "does the
//! waiting zone span the lane's full width", which a bare centerline can't:
//! two adjacent lanes' centerlines can sit close enough together to look
//! like "the two edges of one lane" at a glance.
//!
//! Also emits one `geometry_kind: "traffic_light"` `Point` per
//! signal-controlled *lane* — same rule `zone_generator`'s own
//! `signal_controlled_lanes` uses (`connection.traffic_light.is_some() &&
//! connection.link_index.is_some()`, real edge to real edge, i.e. not
//! through a junction's own internal turning geometry) — placed at the
//! *end* of the lane's own shape, where SUMO puts the stop line. Every
//! zone's own "core" lane is exactly one of these; seeing them lets you
//! check a zone was built for the signal you expect, not a different one
//! at the same junction.
//!
//! Grouped by lane, not emitted one-per-connection: a single approach
//! lane routinely carries more than one signal-controlled movement at
//! once (a shared straight+left+right lane, several parallel phases of
//! the same turn), and every one of those connections shares the exact
//! same stop point — a lane's own `shape.0.last()` depends only on the
//! lane, never on which connection is asking. A first version of this
//! emitted one feature per connection instead, which didn't draw one
//! marker per movement so much as the *same* marker several times, right
//! on top of itself — confirmed on real Eixample data, 2194 pairs of
//! markers sitting 0.0mm apart (one lane, `-1095330712#1_0`, had six
//! stacked on the same point), which read on the map as a "double row" of
//! traffic lights a human correctly noticed didn't correspond to
//! anything physically real. Every movement that used to get its own
//! indistinguishable stacked point instead lives in the one marker's own
//! `link_indices`/`states` arrays now.
//!
//! Not part of the library's own public API (this reprojects with `proj4rs`
//! directly, and re-implements the buffering rather than reusing
//! `geojson_output`'s own `Reprojector`/`buffer_shape`, both `pub(crate)`) —
//! this is a standalone viz aid, not a promise about `waiting_zones`' own
//! output.
//!
//! Usage: `cargo run --release --example net_to_geojson -- <input.net.xml> <output.geojson>`

use anyhow::{Context, Result, bail};
use geo::{Coord, LineString, MultiPolygon, Polygon as GeoPolygon};
use geojson::{Feature, FeatureCollection, Geometry, GeometryValue, JsonObject};
use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, LineJoin, StrokeStyle};
use proj4rs::proj::Proj;
use proj4rs::transform::transform;
use std::collections::HashMap;
use std::env;
use std::path::Path;
use sumo_types::domain::{
    Connection, Edge, EdgeFunction, EdgeId, Lane, LaneId, LaneIndex, LinkIndex, LinkState, Point, Projection,
    TrafficLightId, VClass,
};

const WGS84_LONLAT: &str = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";
// Matches `geometry.rs::ROUND_JOIN_SEGMENT_ANGLE_RADIANS` — not load-bearing
// here (this never needs to match zone geometry bit-for-bit, just look like
// a real lane), but there's no reason to pick a different curve fidelity.
const ROUND_JOIN_SEGMENT_ANGLE_RADIANS: f64 = 0.3;

/// Coarse vehicle-class bucket for `lane`, matching the same
/// pedestrian/vehicle split `viz.html`'s own zone colors already use — so a
/// lane's footprint can be colored to match the *kind* of zone it should
/// underlie, making "which of these many overlapping lanes is the one this
/// zone was built from" visible at a glance instead of requiring a
/// lane_id-by-lane_id lookup.
fn vclass_name(lane: &Lane) -> &'static str {
    if lane.permits(VClass::Passenger) {
        "vehicle"
    } else if lane.permits(VClass::Pedestrian) {
        "pedestrian"
    } else {
        "other"
    }
}

fn link_state_name(state: LinkState) -> &'static str {
    match state {
        LinkState::Major => "major",
        LinkState::Minor => "minor",
        LinkState::TlsOffNoSignal => "tls_off_no_signal",
        LinkState::TlsOffBlinking => "tls_off_blinking",
        LinkState::Equal => "equal",
        LinkState::Stop => "stop",
        LinkState::AllWayStop => "all_way_stop",
        LinkState::Zipper => "zipper",
        LinkState::DeadEnd => "dead_end",
    }
}

fn function_name(function: EdgeFunction) -> &'static str {
    match function {
        EdgeFunction::Normal => "normal",
        EdgeFunction::Internal => "internal",
        EdgeFunction::Connector => "connector",
        EdgeFunction::Crossing => "crossing",
        EdgeFunction::Walkingarea => "walkingarea",
    }
}

/// `lane`'s own true footprint in local network coordinates: its centerline
/// buffered by half its own `width` on each side, exactly like
/// `geometry.rs::buffer_shape` builds a zone's polygon from a lane — so this
/// is directly comparable to a zone's own drawn width, not an approximation
/// of it.
fn lane_footprint(lane: &Lane) -> MultiPolygon<f64> {
    let path: Vec<[f64; 2]> = lane.shape.0.iter().map(|p| [p.x, p.y]).collect();
    if path.len() < 2 {
        return MultiPolygon::new(Vec::new());
    }
    let style = StrokeStyle::new(lane.width.get::<sumo_types::uom::si::length::meter>())
        .line_join(LineJoin::Round(ROUND_JOIN_SEGMENT_ANGLE_RADIANS))
        .start_cap(LineCap::Butt)
        .end_cap(LineCap::Butt);
    let close = |points: Vec<[f64; 2]>| -> LineString<f64> {
        let mut coords: Vec<Coord<f64>> = points.into_iter().map(|[x, y]| Coord { x, y }).collect();
        if !coords.is_empty() && coords.first() != coords.last() {
            coords.push(coords[0]);
        }
        LineString::new(coords)
    };
    MultiPolygon::new(
        path.stroke(style, false)
            .into_iter()
            .filter_map(|mut contours| {
                if contours.is_empty() {
                    return None;
                }
                let exterior = close(contours.remove(0));
                let interiors = contours.into_iter().map(close).collect();
                Some(GeoPolygon::new(exterior, interiors))
            })
            .collect(),
    )
}

fn main() -> Result<()> {
    let args: Vec<String> = env::args().collect();
    let [_, input, output] = args.as_slice() else {
        bail!("usage: net_to_geojson <input.net.xml> <output.geojson>");
    };

    let network = sumo_types::read_network(Path::new(input))?;

    let Projection::Proj4(proj_string) = &network.location.projection else {
        bail!("network is not georeferenced (location/@projParameter is \"!\")");
    };
    let from = Proj::from_proj_string(proj_string)
        .with_context(|| format!("invalid PROJ4 string in .net.xml location: {proj_string:?}"))?;
    let to = Proj::from_proj_string(WGS84_LONLAT).expect("WGS84_LONLAT is a valid PROJ4 string");
    let net_offset = network.location.net_offset;

    let to_lon_lat = |point: Point| -> Result<Vec<f64>> {
        let mut coords = (point.x - net_offset.x, point.y - net_offset.y, 0.0);
        transform(&from, &to, &mut coords)
            .with_context(|| format!("could not reproject network point {point:?} to WGS84"))?;
        Ok(vec![coords.0.to_degrees(), coords.1.to_degrees()])
    };
    let reproject_coord = |c: &Coord<f64>| -> Result<Vec<f64>> { to_lon_lat(Point { x: c.x, y: c.y, z: 0.0 }) };
    let reproject_ring = |ring: &LineString<f64>| -> Result<Vec<Vec<f64>>> {
        ring.coords().map(reproject_coord).collect()
    };

    let mut features = Vec::new();
    for edge in &network.edges {
        for lane in &edge.lanes {
            let mut base_properties = JsonObject::new();
            base_properties.insert("lane_id".to_string(), lane.id.0.clone().into());
            base_properties.insert("edge_id".to_string(), edge.id.0.clone().into());
            base_properties.insert("function".to_string(), function_name(edge.function).into());
            base_properties.insert("vclass".to_string(), vclass_name(lane).into());
            base_properties.insert("width".to_string(), lane.width.get::<sumo_types::uom::si::length::meter>().into());

            let centerline = lane.shape.0.iter().map(|&point| to_lon_lat(point)).collect::<Result<Vec<_>>>()?;
            let mut centerline_properties = base_properties.clone();
            centerline_properties.insert("geometry_kind".to_string(), "centerline".into());
            let mut centerline_feature = Feature::from(Geometry::new(GeometryValue::new_line_string(centerline)));
            centerline_feature.properties = Some(centerline_properties);
            features.push(centerline_feature);

            for part in lane_footprint(lane).0 {
                let rings = std::iter::once(part.exterior())
                    .chain(part.interiors())
                    .map(reproject_ring)
                    .collect::<Result<Vec<_>>>()?;
                let mut footprint_properties = base_properties.clone();
                footprint_properties.insert("geometry_kind".to_string(), "footprint".into());
                let mut footprint_feature = Feature::from(Geometry::new(GeometryValue::new_polygon(rings)));
                footprint_feature.properties = Some(footprint_properties);
                features.push(footprint_feature);
            }
        }
    }

    // Same lookups `zone_generator::generate` builds for itself (see that
    // module's own docs on `signal_controlled_lanes` and `is_real_to_real`)
    // — duplicated here for the same reason the reprojection/buffering
    // above is: this is a standalone binary, with no access to that
    // module's private helpers.
    let internal_edges: std::collections::HashSet<&EdgeId> =
        network.edges.iter().filter(|edge| edge.function == EdgeFunction::Internal).map(|edge| &edge.id).collect();
    let mut edge_by_id: HashMap<&EdgeId, &Edge> = HashMap::new();
    let mut lane_by_edge_and_index: HashMap<(&EdgeId, LaneIndex), &Lane> = HashMap::new();
    for edge in &network.edges {
        edge_by_id.insert(&edge.id, edge);
        for lane in &edge.lanes {
            lane_by_edge_and_index.insert((&edge.id, lane.index), lane);
        }
    }

    // Grouped by the resolved *lane*, not emitted one-per-connection: a
    // single approach lane routinely carries more than one signal-
    // controlled movement at once (a shared straight+left+right lane, or
    // several parallel phases of the same turn) — real, legitimate
    // `.net.xml` data, not a bug there — and every one of those
    // connections shares the exact same stop point (`lane.shape.0.last()`
    // depends only on the lane, never on which connection is asking).
    // Emitting one feature per connection therefore doesn't draw one
    // marker per movement so much as it draws the *same* marker several
    // times, exactly on top of itself: confirmed on real Eixample data,
    // 2194 pairs of markers sitting 0.0mm apart (one lane,
    // `-1095330712#1_0`, alone had six stacked on the same point) — a
    // "double row of traffic lights" a human reviewing the map correctly
    // read as not corresponding to anything physically real, since there
    // is only one real stop line there, not several. Grouping first means
    // the one marker this crate's own module docs already promised
    // ("one marker per signal-controlled approach") is the one it
    // actually draws; every movement it used to spread across separate,
    // indistinguishable points now lives in that single marker's own
    // `link_indices`/`states` arrays instead.
    // A lane's own signal-controlled movements: one `(traffic_light_id,
    // link_index, state)` triple per connection, alongside one connection
    // (arbitrarily the first found — every field this function still
    // reads off it, `from_edge`/`from_lane`, is identical across every
    // connection sharing this lane by construction) to resolve `edge_id`
    // and `junction_id` from afterward.
    type LaneMovements<'a> = (&'a Connection, Vec<(&'a TrafficLightId, LinkIndex, LinkState)>);
    let mut movements_by_lane: HashMap<&LaneId, LaneMovements<'_>> = HashMap::new();
    for connection in &network.connections {
        if internal_edges.contains(&connection.from_edge) || internal_edges.contains(&connection.to_edge) {
            continue;
        }
        let (Some(traffic_light_id), Some(link_index)) = (&connection.traffic_light, connection.link_index) else {
            continue;
        };
        let Some(lane_id) = lane_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane)).map(|l| &l.id)
        else {
            continue;
        };
        movements_by_lane
            .entry(lane_id)
            .or_insert_with(|| (connection, Vec::new()))
            .1
            .push((traffic_light_id, link_index, connection.state));
    }

    for (lane_id, (first_connection, movements)) in movements_by_lane {
        let Some(&lane) = lane_by_edge_and_index.get(&(&first_connection.from_edge, first_connection.from_lane))
        else {
            continue;
        };
        // SUMO puts the stop line at the very end of the approach lane's
        // own shape, right at the junction border — not at some point
        // partway along it, and not at the junction's own (potentially
        // far-off-center) `position`.
        let Some(&stop_point) = lane.shape.0.last() else { continue };

        // Every distinct traffic light id this lane's own movements name —
        // almost always exactly one (every movement of one approach is
        // governed by the same program), kept as its own array rather than
        // a single value on the unusual chance a lane's connections are
        // genuinely split across more than one.
        let mut traffic_light_ids: Vec<&str> = movements.iter().map(|(tl, _, _)| tl.0.as_str()).collect();
        traffic_light_ids.sort_unstable();
        traffic_light_ids.dedup();

        let mut properties = JsonObject::new();
        properties.insert("geometry_kind".to_string(), "traffic_light".into());
        properties.insert("lane_id".to_string(), lane_id.0.clone().into());
        properties.insert("edge_id".to_string(), first_connection.from_edge.0.clone().into());
        properties.insert("traffic_light_ids".to_string(), serde_json::json!(traffic_light_ids));
        properties.insert(
            "link_indices".to_string(),
            serde_json::json!(movements.iter().map(|(_, index, _)| index.0).collect::<Vec<_>>()),
        );
        properties.insert(
            "states".to_string(),
            serde_json::json!(movements.iter().map(|(_, _, state)| link_state_name(*state)).collect::<Vec<_>>()),
        );
        if let Some(junction_id) = edge_by_id.get(&first_connection.from_edge).and_then(|edge| edge.to.as_ref()) {
            properties.insert("junction_id".to_string(), junction_id.0.clone().into());
        }

        let mut feature = Feature::from(Geometry::new(GeometryValue::new_point(to_lon_lat(stop_point)?)));
        feature.properties = Some(properties);
        features.push(feature);
    }

    let collection = FeatureCollection { bbox: None, features, foreign_members: None };
    let json = serde_json::to_string_pretty(&collection)
        .context("could not serialize network as GeoJSON")?;
    std::fs::write(output, json).with_context(|| format!("could not write output file: {output:?}"))?;
    println!("Network GeoJSON written successfully: {output}");
    Ok(())
}

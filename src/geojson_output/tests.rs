#[cfg(test)]
mod tests {
    use super::*;
    use anyhow::Result;
    use geo::{Area, BooleanOps, Coord, LineString, MultiPolygon, Polygon as GeoPolygon};
    use geojson::{Feature, FeatureCollection, Position};
    use std::collections::HashMap;
    use crate::geojson_output::{
        MIN_DRAWN_LANE_LENGTH_METERS, Reprojector, build_feature, feature_rings, overlapping_zone_ids,
        resolve_overlaps, single_successors, stop_line_point, to_feature_collection, zone_feature,
        zone_modes, zone_polygon,
    };
    use sumo_types::additional::domain::{DetectorGate, DetectorId, E3Detector, LanePosition, LaneRef};
    use sumo_types::domain::{
        Boundary, Connection, ConnectionDirection, Edge, EdgeFunction, EdgeId, Lane, LaneId, LaneIndex,
        LinkState, Location, Network, Point, Projection, Shape, VClass,
    };
    use sumo_types::uom::si::f64::Length;
    use sumo_types::uom::si::length::meter;
    use sumo_types::uom::si::velocity::meter_per_second;

    /// A straight, north-pointing lane 20m long, its shape running from
    /// `(0, 0)` to `(0, 20)` in a UTM-like local CRS.
    fn straight_lane(id: &str, width_m: f64) -> Lane {
        parallel_lane(id, width_m, 0.0)
    }

    /// [`straight_lane`], shifted `x_offset_m` east — two of these `x_offset_m`
    /// apart produce two zones whose rectangles overlap when
    /// `x_offset_m < width_m` (their half-widths reach past the gap between
    /// centrelines) and don't when `x_offset_m` is comfortably larger.
    fn parallel_lane(id: &str, width_m: f64, x_offset_m: f64) -> Lane {
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(10.0),
            length: Length::new::<meter>(20.0),
            width: Length::new::<meter>(width_m),
            end_offset: Length::new::<meter>(0.0),
            shape: Shape(vec![
                Point {
                    x: x_offset_m,
                    y: 0.0,
                    z: 0.0,
                },
                Point {
                    x: x_offset_m,
                    y: 20.0,
                    z: 0.0,
                },
            ]),
            allow: vec![],
            disallow: vec![],
        }
    }

    /// [`parallel_lane`], with `index` set instead of the default 0 —
    /// needed for a multi-lane zone's own tests, which build a real
    /// same-edge group of lanes rather than just one lane repeated.
    fn indexed_parallel_lane(id: &str, index: usize, width_m: f64, x_offset_m: f64) -> Lane {
        Lane {
            index: LaneIndex(index),
            ..parallel_lane(id, width_m, x_offset_m)
        }
    }

    /// A straight, north-pointing lane `length_m` long — [`padded_entry`]'s
    /// own tests need a length shorter than [`MIN_DRAWN_LANE_LENGTH_METERS`],
    /// unlike every other lane helper here, which is always the fixed 20m.
    fn short_lane(id: &str, length_m: f64, width_m: f64) -> Lane {
        Lane {
            length: Length::new::<meter>(length_m),
            shape: Shape(vec![
                Point { x: 0.0, y: 0.0, z: 0.0 },
                Point { x: 0.0, y: length_m, z: 0.0 },
            ]),
            ..parallel_lane(id, width_m, 0.0)
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

    /// [`utm_31n_network`], but with `edges` (each `(edge_id, lanes)`)
    /// instead of a single fixed `"e0"` — needed for
    /// [`zone_feature`]'s own tests, where an extended ancestor entry
    /// (see its own docs) has to be a lane of a genuinely different edge
    /// from the zone's own controlled one.
    fn utm_31n_network_multi_edge(edges: Vec<(&str, Vec<Lane>)>) -> Network {
        Network {
            edges: edges
                .into_iter()
                .map(|(id, lanes)| Edge {
                    id: EdgeId(id.into()),
                    function: EdgeFunction::Normal,
                    from: None,
                    to: None,
                    name: None,
                    priority: None,
                    length: None,
                    shape: None,
                    spread_type: None,
                    lanes,
                })
                .collect(),
            ..utm_31n_network(vec![])
        }
    }

    /// A real, single-lane (`fromLane`/`toLane` both `0`) `<connection>`
    /// from `from_edge` to `to_edge`, optionally bridged by an internal
    /// `via` lane — [`single_successors`]'s own tests need real
    /// connections to re-derive chain adjacency from, unlike every other
    /// fixture here, which leaves `Network::connections` empty since
    /// nothing before this needed it.
    fn plain_connection(from_edge: &str, to_edge: &str, via: Option<&str>) -> Connection {
        Connection {
            from_edge: EdgeId(from_edge.into()),
            to_edge: EdgeId(to_edge.into()),
            from_lane: LaneIndex(0),
            to_lane: LaneIndex(0),
            direction: ConnectionDirection::Straight,
            state: LinkState::Major,
            via: via.map(|via| LaneId(via.into())),
            traffic_light: None,
            link_index: None,
            pass: false,
            keep_clear: true,
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
        zone_spanning(id, lane, Length::new::<meter>(20.0))
    }

    /// [`zone`], with the exit gate at `exit` metres from the lane's start
    /// instead of the fixed 20m [`straight_lane`]'s own length happens to
    /// match — for a lane whose real length isn't 20m.
    fn zone_spanning(id: &str, lane: &str, exit: Length) -> E3Detector {
        E3Detector {
            id: DetectorId(id.into()),
            entries: vec![gate(
                lane,
                LanePosition::FromStart(Length::new::<meter>(0.0)),
            )],
            exits: vec![gate(lane, LanePosition::FromStart(exit))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        }
    }

    /// [`zone`], but spanning every lane in `lane_ids` at once — a zone
    /// whose waiting area covers more than one lane of the same edge, the
    /// shape [`merged_core_polygon`] exists for. Every lane's own entry/exit
    /// is the fixed `[0, 20]`m span [`straight_lane`]/[`parallel_lane`]
    /// both use.
    fn zone_multi(id: &str, lane_ids: &[&str]) -> E3Detector {
        E3Detector {
            id: DetectorId(id.into()),
            entries: lane_ids
                .iter()
                .map(|lane| gate(lane, LanePosition::FromStart(Length::new::<meter>(0.0))))
                .collect(),
            exits: lane_ids
                .iter()
                .map(|lane| gate(lane, LanePosition::FromStart(Length::new::<meter>(20.0))))
                .collect(),
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
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

    /// The `modes` a `Feature`'s own `"modes"` property lists, as plain
    /// `&str`s — shared by every `zone_modes` test below so each one reads
    /// as a one-line assertion instead of repeating the same unwrap chain.
    fn modes_of(feature: &Feature) -> Vec<String> {
        feature
            .property("modes")
            .unwrap()
            .as_array()
            .unwrap()
            .iter()
            .map(|m| m.as_str().unwrap().to_string())
            .collect()
    }

    #[test]
    fn a_default_lane_with_no_allow_or_disallow_is_a_generic_vehicle_zone() {
        // Permits everything, including bicycles -- but not a *dedicated*
        // bike lane (it permits cars too), so this is still the generic
        // CAR/MOTORCYCLE case, not BICYCLE.
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_dedicated_bike_lane_is_bicycle_only() {
        let lane = Lane {
            allow: vec!["bicycle".to_string()],
            ..straight_lane("e0_0", 3.2)
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["BICYCLE"]);
    }

    #[test]
    fn a_bus_only_lane_still_counts_as_a_generic_vehicle_zone() {
        // The infrastructure a `.net.xml` encodes never creates a
        // waiting-zone-level distinction this fine (see `zone_modes`'s own
        // docs) -- a bus-only restriction doesn't turn this into a third
        // kind of waiting zone or an empty admission, it's still the
        // generic driving case.
        let lane = Lane {
            allow: vec!["bus".to_string()],
            ..straight_lane("e0_0", 3.2)
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_lane_disallowing_bicycles_is_still_a_generic_vehicle_zone() {
        let lane = Lane {
            disallow: vec!["bicycle".to_string()],
            ..straight_lane("e0_0", 3.2)
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_pedestrian_zone_is_always_on_foot_regardless_of_its_lanes_own_vclass() {
        // `zone_modes` checks `detect_persons`, not the lane's own
        // permissions, for a pedestrian zone -- real walkingarea lanes
        // always carry `allow="pedestrian"` anyway (`sumo_types` doesn't
        // model anything finer there), but this pins that the *zone's* own
        // marker decides, not a lane lookup that would happen to agree.
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let ped_zone = E3Detector {
            detect_persons: vec!["walk".to_string()],
            ..zone("j0_0", "e0_0")
        };

        let collection = to_feature_collection(&network, &[ped_zone]).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["ON_FOOT"]);
    }

    #[test]
    fn a_multi_lane_zone_with_only_one_dedicated_bike_lane_is_still_a_generic_vehicle_zone() {
        // `e0_1` alone is a dedicated bike lane, but `e0_0` shares the same
        // physical queue and admits ordinary traffic too -- the group as a
        // whole isn't a dedicated bike facility, so this is CAR/MOTORCYCLE,
        // not BICYCLE. *Every* lane in the group has to be bike-dedicated
        // for the zone to count as one -- see the next test.
        let network = utm_31n_network(vec![
            indexed_parallel_lane("e0_0", 0, 3.2, 0.0),
            Lane {
                allow: vec!["bicycle".to_string()],
                ..indexed_parallel_lane("e0_1", 1, 3.2, 3.2)
            },
        ]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_1"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["CAR", "MOTORCYCLE"]);
    }

    #[test]
    fn a_multi_lane_zone_where_every_lane_is_a_dedicated_bike_lane_is_bicycle_only() {
        let bike_lane = |id: &str, index: usize, x_offset_m: f64| Lane {
            allow: vec!["bicycle".to_string()],
            ..indexed_parallel_lane(id, index, 3.2, x_offset_m)
        };
        let network = utm_31n_network(vec![bike_lane("e0_0", 0, 0.0), bike_lane("e0_1", 1, 3.2)]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_1"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(modes_of(&collection.features[0]), vec!["BICYCLE"]);
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
        // The lane runs due north, so the perpendicular offset is purely
        // in x (easting): the ring's own widest point-to-point longitude
        // spread is the full lane width, not just half of it — not tied to
        // any particular vertex's own index (self-intersection removal can
        // reorder/collapse points), just the shape's own extent.
        let lons = ring.iter().map(|p| p[0]);
        let lon_span = lons.clone().fold(f64::MIN, f64::max) - lons.fold(f64::MAX, f64::min);
        assert!(
            lon_span > 0.00001,
            "expected a visible offset, got {lon_span}"
        );
        assert!(
            lon_span < 0.001,
            "offset implausibly large for a 3.2m lane: {lon_span}"
        );
    }

    #[test]
    fn pads_a_lane_shorter_than_a_car_out_to_a_reasonable_minimum_length() {
        // e0 alone, 1m long -- shorter than any real vehicle, and with
        // nothing else in the network at all (no connections, so nothing
        // for `zone_generator::extended_entry_lanes` to have walked
        // through even if this fixture went through it). `padded_entry`
        // still pads it out by extrapolating its own first segment
        // backward in a straight line -- see its own docs for why that,
        // not a real neighbouring lane's own shape, is deliberate: a real
        // predecessor lane is, in general, ground a *different* zone's own
        // polygon already draws.
        let network = utm_31n_network(vec![short_lane("e0_0", 1.0, 3.2)]);
        let zones = vec![zone_spanning("j0_0", "e0_0", Length::new::<meter>(1.0))];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];

        // e0 runs due north, so its own length shows up as latitude span;
        // a bare 1m lane would span roughly 1m worth of latitude, nowhere
        // near MIN_DRAWN_LANE_LENGTH_METERS's own 5m.
        let lats = ring.iter().map(|p| p[1]);
        let lat_span_m =
            (lats.clone().fold(f64::MIN, f64::max) - lats.fold(f64::MAX, f64::min)) * 111_320.0;
        assert!(
            lat_span_m > MIN_DRAWN_LANE_LENGTH_METERS - 0.5,
            "expected the padded span to reach close to {MIN_DRAWN_LANE_LENGTH_METERS}m, \
             got {lat_span_m:.2}m"
        );
    }

    #[test]
    fn flags_two_zones_whose_rectangles_actually_overlap() {
        // 3.2m-wide lanes, centrelines 2m apart: each rectangle reaches
        // 1.6m either side of its own centreline, so they cover a shared
        // 1.6+1.6-2=1.2m-wide strip down the middle.
        //
        // Built directly via `zone_feature`, not `to_feature_collection`:
        // that function now resolves an overlap like this one itself (see
        // `resolve_overlaps`), so going through it here would test the
        // resolver, not the detector this test actually means to check in
        // isolation -- `to_feature_collection_bisects_...`, below, is the
        // one that exercises the resolver on this exact fixture.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 2.0),
        ]);
        let lanes: HashMap<&str, &Lane> =
            network.edges.iter().flat_map(|edge| &edge.lanes).map(|lane| (lane.id.0.as_str(), lane)).collect();
        let successors = single_successors(&network);
        let reproject = Reprojector::new(&network.location).unwrap();
        let collection = FeatureCollection {
            bbox: None,
            features: vec![
                zone_feature(&zone("j0_0", "e0_0"), &lanes, &successors, &reproject, MIN_DRAWN_LANE_LENGTH_METERS)
                    .unwrap(),
                zone_feature(&zone("j0_1", "e0_1"), &lanes, &successors, &reproject, MIN_DRAWN_LANE_LENGTH_METERS)
                    .unwrap(),
            ],
            foreign_members: None,
        };

        let overlaps = overlapping_zone_ids(&collection);
        assert_eq!(
            overlaps,
            vec![("j0_0".to_string(), "j0_1".to_string())],
            "these two rectangles are built to overlap by construction"
        );
    }

    #[test]
    fn to_feature_collection_shrinks_padding_just_enough_to_clear_a_real_neighbour() {
        // `e0_0` is a 2m-long north-pointing stub at x=0 -- short enough
        // that `padded_entry` extrapolates its southern end back to y=-3
        // (see `pads_a_lane_shorter_than_a_car_out_to_a_reasonable_minimum_length`).
        // `e1_0` is a real, full-length east-west lane whose own northern
        // edge sits at y=-0.4 (half its 3.2m width above its y=-2
        // centreline): unpadded, the two don't come anywhere near each
        // other (`e0_0`'s real span is y in [0, 2]), but the *full*
        // default pad (reaching to y=-3) overlaps it by a wide margin.
        // Anywhere in between -- down to y=-0.4, a pad of ~2.4m -- is
        // exactly as safe as no padding at all, which is what
        // `max_safe_pad`'s own binary search should find instead of
        // dropping straight to zero. With `OVERLAP_AREA_THRESHOLD_DEG2`
        // now correctly sized (its own docs on the earlier, ~9300x too
        // permissive value), the search converges tightly to that real
        // ~2.4m geometric boundary rather than drifting well past it.
        let lane_a = short_lane("e0_0", 2.0, 3.2);
        let lane_b = Lane {
            shape: Shape(vec![
                Point { x: -5.0, y: -2.0, z: 0.0 },
                Point { x: 5.0, y: -2.0, z: 0.0 },
            ]),
            ..short_lane("e1_0", 10.0, 3.2)
        };
        let network = utm_31n_network_multi_edge(vec![("e0", vec![lane_a]), ("e1", vec![lane_b])]);
        let zones = vec![
            zone_spanning("j0_0", "e0_0", Length::new::<meter>(2.0)),
            zone_spanning("j0_1", "e1_0", Length::new::<meter>(10.0)),
        ];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(
            overlapping_zone_ids(&collection),
            Vec::new(),
            "shrinking the stub's own padding should have been enough to clear e1_0"
        );

        let stub = collection.features.iter().find(|f| f.property("waiting_zone_id").unwrap() == "j0_0").unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } = &stub.geometry.as_ref().unwrap().value else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        let lats = ring.iter().map(|p| p[1]);
        let lat_span_m = (lats.clone().fold(f64::MIN, f64::max) - lats.fold(f64::MAX, f64::min)) * 111_320.0;
        assert!(
            lat_span_m > 2.0,
            "expected some padding to survive (more than e0_0's own real 2m span), got {lat_span_m:.2}m"
        );
        assert!(
            lat_span_m < 2.45,
            "expected convergence tight against the real ~2.4m geometric boundary, not \
             drifting toward the full 5m default, got {lat_span_m:.2}m"
        );
    }

    #[test]
    fn to_feature_collection_bisects_a_genuine_geometric_overlap_padding_cant_explain() {
        // The exact fixture `flags_two_zones_whose_rectangles_actually_overlap` uses to
        // prove the *detector* catches a real overlap -- both lanes are a full 20m
        // long, well past `MIN_DRAWN_LANE_LENGTH_METERS`, so dropping padding can't
        // do anything here at all (both zones are already unpadded); only bisecting
        // can, and this proves `to_feature_collection` reaches for it.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 2.0),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(
            overlapping_zone_ids(&collection),
            Vec::new(),
            "the two rings should have been bisected apart"
        );
        for feature in &collection.features {
            let geojson::GeometryValue::MultiPolygon { coordinates } =
                &feature.geometry.as_ref().unwrap().value
            else {
                panic!("expected a MultiPolygon geometry");
            };
            assert!(
                !coordinates.is_empty(),
                "{:?} lost all of its geometry to the cut",
                feature.property("waiting_zone_id")
            );
        }
    }

    #[test]
    fn to_feature_collection_clips_a_ring_contested_by_two_neighbours_in_one_pass() {
        // Three parallel 3.2m lanes 2m apart: `e0_1` (the middle one)
        // overlaps *both* `e0_0` and `e0_2` (each 2m away, well inside the
        // combined 3.2m of half-widths), while `e0_0` and `e0_2` (4m apart)
        // don't overlap each other at all. `e0_1`'s own zone therefore
        // needs two simultaneous constraints applied to its *one* original
        // shape -- not two sequential cuts against whatever the first one
        // left behind, which is exactly the bug `resolve_overlaps`'s own
        // docs describe fragmenting a real Barcelona corner contested by
        // several neighbours into a disconnected shred.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 2.0),
            parallel_lane("e0_2", 3.2, 4.0),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1"), zone("j0_2", "e0_2")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(overlapping_zone_ids(&collection), Vec::new());

        let middle = collection
            .features
            .iter()
            .find(|f| f.property("waiting_zone_id").unwrap() == "j0_1")
            .unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } = &middle.geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(coordinates.len(), 1, "j0_1 should still be one simple, connected shape");
        assert!(
            !ring_self_intersects(&coordinates[0][0]),
            "clipping against two neighbours in one pass should stay a single simple \
             polygon, not a self-intersecting shred: {:?}",
            coordinates[0][0]
        );
    }

    /// A straight lane of `width_m`, running from `from` to `to` in the
    /// network's own local coordinates — [`single_successors`]'s and
    /// [`chain_shape`]'s own tests need real, differently-positioned
    /// connected segments, unlike [`parallel_lane`]'s fixed north-south
    /// 20m shape.
    fn segment_lane(id: &str, width_m: f64, from: (f64, f64), to: (f64, f64)) -> Lane {
        let length = (to.0 - from.0).hypot(to.1 - from.1);
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(10.0),
            length: Length::new::<meter>(length),
            width: Length::new::<meter>(width_m),
            end_offset: Length::new::<meter>(0.0),
            shape: Shape(vec![
                Point { x: from.0, y: from.1, z: 0.0 },
                Point { x: to.0, y: to.1, z: 0.0 },
            ]),
            allow: vec![],
            disallow: vec![],
        }
    }

    #[test]
    fn extended_ancestor_chain_becomes_one_seamless_polygon_not_one_rectangle_per_lane() {
        // Three real, connected 20m lanes end to end: e2 (furthest back)
        // into e1 into e0 (the zone's own controlled lane). Both e1 and e2
        // are extended-ancestor entries, exactly as
        // `zone_generator::extended_entry_lanes` would produce for a real
        // fork-free chain like this -- before `chain_shape` existed, each
        // drew its own independent 20m rectangle, with a real Barcelona
        // gap between them often enough to be worth fixing; now the two
        // should concatenate into one seamless 40m polygon instead.
        let lane_e0 = segment_lane("e0_0", 3.2, (0.0, 0.0), (0.0, 20.0));
        let lane_e1 = segment_lane("e1_0", 3.2, (0.0, -20.0), (0.0, 0.0));
        let lane_e2 = segment_lane("e2_0", 3.2, (0.0, -40.0), (0.0, -20.0));
        let network = Network {
            connections: vec![plain_connection("e2", "e1", None), plain_connection("e1", "e0", None)],
            ..utm_31n_network_multi_edge(vec![
                ("e0", vec![lane_e0]),
                ("e1", vec![lane_e1]),
                ("e2", vec![lane_e2]),
            ])
        };

        let zone = E3Detector {
            id: DetectorId("j0_0".into()),
            entries: vec![
                gate("e0_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e1_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e2_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
            ],
            exits: vec![gate("e0_0", LanePosition::FromStart(Length::new::<meter>(20.0)))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        };

        let collection = to_feature_collection(&network, &[zone]).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        // e0, e1 and e2 are collinear and meet exactly end to end, so their
        // union is genuinely one seamless 60m ribbon, not three independent
        // rectangles -- nor even two separately-tracked "core" and "chain"
        // polygons that merely happen to touch: a real `union` merges
        // anything that touches into one connected polygon, which is a
        // strictly better result than the former design's own "list of
        // rings, however each one was built" ever guaranteed.
        assert_eq!(coordinates.len(), 1, "expected one seamless polygon for e0+e1+e2 combined");

        let ring = &coordinates[0][0];
        assert!(!ring_self_intersects(ring), "{ring:?}");
        let lats = ring.iter().map(|p| p[1]);
        let lat_span_m = (lats.clone().fold(f64::MIN, f64::max) - lats.fold(f64::MAX, f64::min)) * 111_320.0;
        assert!(
            lat_span_m > 55.0,
            "expected the merged e0+e1+e2 ribbon to span close to their combined 60m, \
             got {lat_span_m:.1}m"
        );
    }

    #[test]
    fn two_leaves_merging_into_a_shared_ancestor_produce_no_duplicated_ground() {
        // A real Y-merge: `e_leaf_a` and `e_leaf_b` are two independent
        // real streets that both feed into `e_mid`, which then feeds into
        // the zone's own core (`e0`) -- the exact shape a real Barcelona
        // corner hit (see `chain_shape`'s own module docs). `e_mid` has two
        // real predecessors (`in_degree` 2), so it has to start its *own*
        // segment rather than being drawn a second time by each branch: an
        // earlier, hand-rolled version of this pipeline drew every branch's
        // own segment as an independent ring, and Leaflet's own default
        // `evenodd` fill rule turns ground covered by two of a zone's own
        // rings into a rendered hole -- a real, visible defect on real
        // Barcelona data. `chain_shape` still stops each branch's own walk
        // at `e_mid` for that reason (see its own docs), but now that every
        // segment is unioned into one `MultiPolygon` rather than kept as
        // independent rings regardless of whether they touch, duplicated
        // ground isn't just avoided at the `chain_shape` level any more --
        // `union` couldn't double-count it even if it were still handed
        // twice, which is what this test actually checks: the merged
        // area is real coverage, not gap nor duplication.
        let lane_e0 = segment_lane("e0_0", 3.2, (0.0, 0.0), (0.0, 20.0));
        let lane_mid = segment_lane("e_mid_0", 3.2, (0.0, -20.0), (0.0, 0.0));
        let lane_leaf_a = segment_lane("e_leaf_a_0", 3.2, (-20.0, -40.0), (0.0, -20.0));
        let lane_leaf_b = segment_lane("e_leaf_b_0", 3.2, (20.0, -40.0), (0.0, -20.0));
        let network = Network {
            connections: vec![
                plain_connection("e_leaf_a", "e_mid", None),
                plain_connection("e_leaf_b", "e_mid", None),
                plain_connection("e_mid", "e0", None),
            ],
            ..utm_31n_network_multi_edge(vec![
                ("e0", vec![lane_e0]),
                ("e_mid", vec![lane_mid]),
                ("e_leaf_a", vec![lane_leaf_a]),
                ("e_leaf_b", vec![lane_leaf_b]),
            ])
        };

        let zone = E3Detector {
            id: DetectorId("j0_0".into()),
            entries: vec![
                gate("e0_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e_mid_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e_leaf_a_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e_leaf_b_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
            ],
            exits: vec![gate("e0_0", LanePosition::FromStart(Length::new::<meter>(20.0)))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        };

        let collection = to_feature_collection(&network, &[zone]).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        // Every segment (core, e_mid, both leaves) touches at least one
        // other at a real junction corner, so their union is one connected
        // polygon -- there's no "was e_mid drawn twice" question left to
        // ask ring-by-ring any more, since a real union structurally can't
        // double-count the ground two of its own inputs share.
        assert_eq!(coordinates.len(), 1, "expected one polygon covering the whole merged shape");
        let ring = &coordinates[0][0];
        assert!(!ring_self_intersects(ring), "{ring:?}");

        // Real, non-overlapping area: e0 (20m) + e_mid (20m) + leaf_a/b
        // (√(20²+20²) ≈ 28.28m each), all 3.2m wide ⇒ 64 + 64 + 90.5 + 90.5
        // ≈ 309m². If e_mid's own ~64m² were still counted twice (the bug
        // `chain_shape` itself guards against — see its own docs), the
        // total would jump to ~373m²; if the merge lost real ground at the
        // Y instead, it would fall well short of 309m².
        let points: Vec<(f64, f64)> = ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
        assert!(
            (270.0..340.0).contains(&area_m2),
            "expected the merged Y-shape's own real area close to 309m² (64 + 64 + 90.5 \
             + 90.5, minus a little for the sharp `via`-less corners' own approximation), \
             got {area_m2:.1}m2 -- too high looks like e_mid's own ground duplicated, too \
             low looks like real ground lost at the merge"
        );
    }

    #[test]
    fn does_not_flag_two_zones_that_share_an_edge_exactly() {
        // Centrelines exactly `width` apart: the rectangles' edges coincide
        // (the line straight between the two centrelines) without any
        // actual area in common — adjacent, not overlapping. This is
        // deliberately the *exact* zero-gap case, not a hairline gap: real
        // adjacent lanes on the same road share their boundary exactly, in
        // the network's own local CRS, and reprojecting each rectangle to
        // WGS84 independently (see the module docs) can make that shared
        // edge land on slightly different coordinates for each — a real
        // scenario `polygon_overlap_area`'s own docs cover, which this
        // proves doesn't false-positive.
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 3.2),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(
            overlapping_zone_ids(&collection),
            Vec::new(),
            "these rectangles are built to share an edge exactly, not to overlap"
        );
    }

    #[test]
    fn does_not_flag_two_well_separated_zones() {
        let network = utm_31n_network(vec![
            parallel_lane("e0_0", 3.2, 0.0),
            parallel_lane("e0_1", 3.2, 50.0),
        ]);
        let zones = vec![zone("j0_0", "e0_0"), zone("j0_1", "e0_1")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(overlapping_zone_ids(&collection), Vec::new());
    }

    /// Twice the signed area of triangle `a`, `b`, `c` — positive when `c`
    /// is left of the ray `a -> b`, negative when right, zero when
    /// collinear. Test-only: production code answers every question this
    /// used to answer (a lane's own bend, a ring's own self-intersection,
    /// two rings' own overlap) through [`geo::BooleanOps`] instead — this
    /// still backs a couple of tests that check *that* replacement against
    /// a hand-computed geometric primitive directly, rather than trusting
    /// `geo` circularly.
    fn orientation(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
        (b.0 - a.0) * (c.1 - a.1) - (b.1 - a.1) * (c.0 - a.0)
    }

    /// The signed area enclosed by `ring` (the shoelace formula, no closing
    /// repeat) — positive for a counterclockwise winding, negative for
    /// clockwise. Test-only, for the same reason as [`orientation`].
    fn signed_area(ring: &[(f64, f64)]) -> f64 {
        let n = ring.len();
        if n < 3 {
            return 0.0;
        }
        (0..n)
            .map(|i| {
                let (x1, y1) = ring[i];
                let (x2, y2) = ring[(i + 1) % n];
                x1 * y2 - x2 * y1
            })
            .sum::<f64>()
            / 2.0
    }

    /// Whether any two non-adjacent edges of closed ring `ring` (GeoJSON
    /// style: first position repeated last) properly cross, via
    /// [`orientation`]. A self-intersecting ("bowtie") polygon is invalid
    /// GeoJSON a client's point-in-polygon test can't reason about at all —
    /// every coherence test in this module checks real output against this
    /// directly, rather than trusting that [`geo::BooleanOps`] alone is
    /// enough to guarantee it (it should be; this is the check that would
    /// catch it if it somehow weren't).
    fn ring_self_intersects(ring: &[Position]) -> bool {
        let n = ring.len().saturating_sub(1); // last position repeats the first
        let edge = |i: usize| ((ring[i][0], ring[i][1]), (ring[i + 1][0], ring[i + 1][1]));
        for i in 0..n {
            for j in (i + 2)..n {
                if i == 0 && j == n - 1 {
                    continue; // adjacent via the closing wrap-around
                }
                let (p1, p2) = edge(i);
                let (p3, p4) = edge(j);
                let d1 = orientation(p3, p4, p1);
                let d2 = orientation(p3, p4, p2);
                let d3 = orientation(p1, p2, p3);
                let d4 = orientation(p1, p2, p4);
                if (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0) {
                    return true;
                }
            }
        }
        false
    }

    /// A short, sharply zigzagging, *wide* lane — mirroring a real
    /// Barcelona walkingarea found while fixing this (2.9m long over 8
    /// shape points, 4m wide: the width bigger than the whole path). A
    /// naive per-vertex or per-segment offset self-intersects on a shape
    /// like this — the width exceeds the path's own local turning radius —
    /// exactly the shape [`buffer_shape`]'s per-segment quads, unioned
    /// rather than joined and cleaned up by hand, are meant to handle
    /// correctly regardless of how tight the turn is.
    fn zigzag_lane(id: &str, width_m: f64) -> Lane {
        let shape = Shape(vec![
            Point { x: 0.0, y: 0.0, z: 0.0 },
            Point { x: -2.0, y: 1.0, z: 0.0 },
            Point { x: -1.5, y: 2.0, z: 0.0 },
            Point { x: 1.0, y: 3.5, z: 0.0 },
            Point { x: 2.0, y: 2.5, z: 0.0 },
            Point { x: 0.5, y: 1.0, z: 0.0 },
            Point { x: 1.8, y: -0.5, z: 0.0 },
        ]);
        let length: f64 = shape
            .0
            .windows(2)
            .map(|w| {
                let [a, b] = w else { unreachable!() };
                (b.x - a.x).hypot(b.y - a.y)
            })
            .sum();
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(2.0),
            length: Length::new::<meter>(length),
            width: Length::new::<meter>(width_m),
            end_offset: Length::new::<meter>(0.0),
            shape,
            allow: vec![],
            disallow: vec![],
        }
    }

    #[test]
    fn merges_two_adjacent_lanes_of_the_same_zone_into_a_single_polygon() {
        // Same setup `flags_two_zones_whose_rectangles_actually_overlap`
        // uses for two *different* zones, but as one zone spanning both
        // lanes instead — the real shape this exists for (e.g. two
        // adjacent Barcelona vehicle lanes both going straight).
        let network = utm_31n_network(vec![
            indexed_parallel_lane("e0_0", 0, 3.2, 0.0),
            indexed_parallel_lane("e0_1", 1, 3.2, 3.2),
        ]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_1"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            1,
            "two physically contiguous lanes of the same zone should merge into one \
             polygon, not the seam-prone one-rectangle-per-lane MultiPolygon"
        );
        assert!(!ring_self_intersects(&coordinates[0][0]));
    }

    #[test]
    fn draws_an_extended_ancestor_entry_as_its_own_polygon_alongside_the_core_one() {
        // e0 is the zone's own controlled lane (has an exit); e1 is an
        // ancestor `zone_generator::extended_entry_lanes` walked back onto
        // -- present only as an entry, on a *different* edge, so it can't
        // merge with e0's own polygon (see `zone_feature`'s own docs).
        let network = utm_31n_network_multi_edge(vec![
            ("e0", vec![indexed_parallel_lane("e0_0", 0, 3.2, 0.0)]),
            ("e1", vec![indexed_parallel_lane("e1_0", 0, 3.2, 50.0)]),
        ]);
        let zone = E3Detector {
            id: DetectorId("j0_0".into()),
            entries: vec![
                gate("e0_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
                gate("e1_0", LanePosition::FromStart(Length::new::<meter>(0.0))),
            ],
            exits: vec![gate("e0_0", LanePosition::FromStart(Length::new::<meter>(20.0)))],
            file: String::new(),
            icon_position: None,
            period: None,
            name: None,
            speed_threshold: None,
            time_threshold: None,
            open_entry: None,
            detect_persons: Vec::new(),
        };

        let collection = to_feature_collection(&network, &[zone]).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            2,
            "one merged/core polygon for e0 (the zone's own controlled lane) plus one \
             independent polygon for the extended ancestor e1 -- not merged together, \
             since they're lanes of different edges"
        );
        for polygon in coordinates {
            assert!(!ring_self_intersects(&polygon[0]));
        }
    }

    #[test]
    fn never_bridges_across_a_lane_gap_the_zone_does_not_actually_claim() {
        // Lane 1 sits physically between lanes 0 and 2, but this zone only
        // claims 0 and 2 (as if lane 1 belonged to some other movement) --
        // exactly the shape a non-contiguous group `merged_core_polygon`'s
        // own docs describe. An earlier, hand-rolled version of this
        // pipeline picked the group's own leftmost and rightmost lane and
        // joined them directly regardless, which silently claimed lane 1's
        // own ground (not part of this zone) as if it belonged here too.
        // `merged_core_polygon` doesn't assume anything about lane order or
        // contiguity: lanes 0 and 2 don't actually touch, so their union is
        // honestly two separate polygons, with the real, unclaimed gap
        // between them left alone.
        let network = utm_31n_network(vec![
            indexed_parallel_lane("e0_0", 0, 3.2, 0.0),
            indexed_parallel_lane("e0_1", 1, 3.2, 3.2),
            indexed_parallel_lane("e0_2", 2, 3.2, 6.4),
        ]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_2"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            2,
            "lanes 0 and 2 don't touch (lane 1's own gap sits between them, unclaimed) -- \
             expected two separate polygons, not one bridged fraudulently across the gap"
        );
        for polygon in coordinates {
            assert!(!ring_self_intersects(&polygon[0]), "{:?}", polygon[0]);
        }
    }

    #[test]
    fn buffer_shape_never_self_intersects_on_a_short_wide_zigzag() {
        let lane = zigzag_lane("e0_0", 4.0);
        let length = lane.length;
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone_spanning("j0_0", "e0_0", length)];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        assert!(
            !ring_self_intersects(ring),
            "buffer_shape produced a self-intersecting polygon for a short, wide, \
             sharply zigzagging lane: {ring:?}"
        );
    }

    #[test]
    fn buffer_shape_does_not_collapse_to_a_sliver_on_a_short_lane_with_tight_bends() {
        // A real Barcelona lane (`1395130587_2`, found while fixing an
        // earlier, hand-rolled version of this pipeline) shifted to the
        // origin: five segments, none longer than ~1.8m, with two real
        // direction changes -- short enough, and tight enough relative to a
        // 3.2m-wide lane, that a splice-based self-intersection cleanup
        // used to cut away real, non-crossing area along with the actual
        // self-crossing loop, collapsing a ~21.6m² ribbon down to 0.003m²
        // (a few points a few centimetres apart). `buffer_shape`'s union of
        // per-segment quads has no such collapse risk at all: `union`
        // never removes real, non-overlapping area, so there's nothing
        // here for a fallback to guard against any more.
        let shape = Shape(vec![
            Point { x: 0.0, y: 0.0, z: 0.0 },
            Point { x: -0.14, y: -0.14, z: 0.0 },
            Point { x: -0.95, y: -1.73, z: 0.0 },
            Point { x: -0.77, y: -3.19, z: 0.0 },
            Point { x: -0.57, y: -4.67, z: 0.0 },
            Point { x: -1.35, y: -6.29, z: 0.0 },
        ]);
        let length: f64 = shape
            .0
            .windows(2)
            .map(|w| {
                let [a, b] = w else { unreachable!() };
                (b.x - a.x).hypot(b.y - a.y)
            })
            .sum();
        let lane = Lane {
            id: LaneId("e0_0".into()),
            index: LaneIndex(0),
            speed: sumo_types::uom::si::f64::Velocity::new::<meter_per_second>(2.0),
            length: Length::new::<meter>(length),
            width: Length::new::<meter>(3.2),
            end_offset: Length::new::<meter>(0.0),
            shape,
            allow: vec![],
            disallow: vec![],
        };
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone_spanning("j0_0", "e0_0", Length::new::<meter>(length))];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::MultiPolygon { coordinates } =
            &collection.features[0].geometry.as_ref().unwrap().value
        else {
            panic!("expected a MultiPolygon geometry");
        };
        let ring = &coordinates[0][0];
        assert!(!ring_self_intersects(ring), "collapsed ring should still be simple: {ring:?}");

        let points: Vec<(f64, f64)> = ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
        let naive_area_m2 = length * 3.2;
        assert!(
            area_m2 > naive_area_m2 * 0.5,
            "ring area {area_m2:.3}m2 is far below the naive expectation of \
             {naive_area_m2:.3}m2 for a {length:.2}m lane 3.2m wide: {ring:?}"
        );
    }

    /// The area two closed rings `a` and `b` (GeoJSON style: first position
    /// repeated last) actually have in common, via [`geo::BooleanOps::intersection`]
    /// — the test-only counterpart of [`overlap_area_m2`]/[`overlapping_zone_ids`]'s
    /// own production use of `intersection`, built directly from `Position`s
    /// rather than a [`MultiPolygon`] a caller already has, for tests that
    /// only have raw ring coordinates on hand.
    fn ring_overlap_area(a: &[Position], b: &[Position]) -> f64 {
        let polygon = |ring: &[Position]| {
            MultiPolygon::new(vec![GeoPolygon::new(
                LineString::new(ring.iter().map(|p| Coord { x: p[0], y: p[1] }).collect()),
                Vec::new(),
            )])
        };
        polygon(a).intersection(&polygon(b)).unsigned_area()
    }

    #[test]
    fn ring_overlap_area_handles_a_non_convex_ring_correctly() {
        // An L-shape (non-convex) and a 1x1 square entirely inside its own
        // vertical leg (x in [0,1], y in [1,2]) -- real, unambiguous
        // overlap of exactly 1.0. A hand-rolled clipper that assumes one
        // side of an overlap test is convex (an earlier version of this
        // module did) can silently give `0.0` for a real overlap like this
        // depending on which side lands where -- confirmed on real
        // Barcelona zones sharing several real square metres that measured
        // `0.0` one direction and multiple m² the other. `geo::BooleanOps`
        // makes no such assumption about either side.
        let l_shape: Vec<Position> = [
            (0.0, 0.0),
            (4.0, 0.0),
            (4.0, 1.0),
            (1.0, 1.0),
            (1.0, 4.0),
            (0.0, 4.0),
            (0.0, 0.0),
        ]
        .into_iter()
        .map(|(x, y)| Position::from([x, y]))
        .collect();

        let square: Vec<Position> = [(0.0, 1.0), (1.0, 1.0), (1.0, 2.0), (0.0, 2.0), (0.0, 1.0)]
            .into_iter()
            .map(|(x, y)| Position::from([x, y]))
            .collect();

        let forward = ring_overlap_area(&square, &l_shape);
        let backward = ring_overlap_area(&l_shape, &square);
        assert!(
            (forward - 1.0).abs() < 1e-9,
            "expected the square's own full 1.0 area, entirely inside the L, got {forward}"
        );
        assert!(
            (backward - 1.0).abs() < 1e-9,
            "the same overlap measured with the arguments swapped should agree, got {backward}"
        );
    }

    /// Below this, in real m², a ring can't plausibly be a real waiting
    /// area — not even standing room for a single pedestrian — so it has to
    /// be a construction artifact rather than real ground `zone_generator`
    /// actually meant to claim. Chosen with real headroom below the
    /// smallest *legitimate* ring in real Barcelona data (a genuinely short
    /// zone at ~0.22m², well above this).
    const MIN_PLAUSIBLE_RING_AREA_M2: f64 = 0.05;

    /// Below this, in degrees, a vertex is judged a convex spike — a
    /// needle-thin protrusion a client's own rendering (and a human looking
    /// at the map) reads as visibly wrong — rather than a real corner a
    /// waiting area's own shape can legitimately have. A waiting zone's
    /// boundary is either a straight lane edge (interior angle 180°, dead
    /// flat) or a real bend a round join (see `buffer_shape`'s own docs)
    /// turns smoothly; both stay well clear of 80° on any real street
    /// geometry, so a survivor below it is exactly the shape of defect this
    /// crate's own bug history is full of (a self-intersection a boolean op
    /// left uncleaned, a sliver from a `resolve_overlaps` cut, ...), not a
    /// legitimate acute corner this crate has any reason to draw.
    const MIN_INTERIOR_ANGLE_DEGREES: f64 = 80.0;

    /// Above this, in degrees, a vertex is judged a *reflex* spike — the
    /// concave mirror image of [`MIN_INTERIOR_ANGLE_DEGREES`]'s own convex
    /// one: a needle-thin notch cut *into* the polygon's own interior
    /// rather than protruding out of it, which an undirected angle-between-
    /// edges measure (the angle between two rays, always folded into
    /// `[0°, 180°]`) can't even represent as a value near 360° to catch —
    /// it reads a spike like this as the same small number a genuine convex
    /// spike would give, which is *usually* still caught by
    /// `MIN_INTERIOR_ANGLE_DEGREES` (a bad enough reflex spike folds well
    /// under 80° too) but not reliably close to the boundary, and reports a
    /// misleading angle either way. [`interior_angles_degrees`]'s own
    /// signed computation avoids folding in the first place, so this can be
    /// checked (and reported) directly, symmetric with the convex case
    /// around 180° (`360° - 280° = 80°`).
    const MAX_INTERIOR_ANGLE_DEGREES: f64 = 280.0;

    /// Below this, in degrees (roughly a centimetre at Barcelona's own
    /// latitude — see [`OVERLAP_AREA_THRESHOLD_DEG2`]'s own deg-to-metre
    /// conversion), an edge is too short for its direction to mean
    /// anything — a stray near-duplicate vertex from a boolean op can land
    /// this close to its own neighbour, and the angle either one of them
    /// forms with its *other*, real-length neighbour is noise, not a
    /// spike — skipped by [`interior_angles_degrees`] rather than reported
    /// as a false `0.0`.
    const MIN_EDGE_LENGTH_DEGREES: f64 = 1e-7;

    /// The interior angle, in degrees (`[0°, 360°)`), at every vertex of
    /// `points` (a ring's own distinct vertices — no repeated closing
    /// point) whose two neighbouring edges are both at least
    /// [`MIN_EDGE_LENGTH_DEGREES`] long: `180°` for a vertex a straight
    /// line already passes straight through, sliding down toward `0°` as
    /// the two segments fold back on themselves into a convex spike (the
    /// material pinching to a point), or up toward `360°` as they fold
    /// back the *other* way into a reflex spike (a thin notch cut into the
    /// material instead). Signed via the ring's own overall winding
    /// ([`signed_area`]'s own sign) rather than the plain undirected angle
    /// between the two edge vectors: that alternative can't distinguish
    /// "180° short of a full turn the convex way" from "180° short the
    /// reflex way" at all — both fold to the same value — which is exactly
    /// the distinction [`MAX_INTERIOR_ANGLE_DEGREES`]'s own check needs to
    /// mean anything.
    fn interior_angles_degrees(points: &[(f64, f64)]) -> Vec<f64> {
        let n = points.len();
        let counterclockwise = signed_area(points) >= 0.0;
        (0..n)
            .filter_map(|i| {
                let (prev, cur, next) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
                // Edge directions, not the "pointing away from the vertex"
                // rays an undirected measure would use: the sign of the
                // turn from the incoming edge to the outgoing one is what
                // tells convex from reflex apart.
                let (inx, iny) = (cur.0 - prev.0, cur.1 - prev.1);
                let (outx, outy) = (next.0 - cur.0, next.1 - cur.1);
                let (lin, lout) = (inx.hypot(iny), outx.hypot(outy));
                if lin < MIN_EDGE_LENGTH_DEGREES || lout < MIN_EDGE_LENGTH_DEGREES {
                    return None;
                }
                let cross = inx * outy - iny * outx;
                let dot = inx * outx + iny * outy;
                let turn_degrees = cross.atan2(dot).to_degrees(); // signed, (-180°, 180°]
                let interior = if counterclockwise { 180.0 - turn_degrees } else { 180.0 + turn_degrees };
                Some(((interior % 360.0) + 360.0) % 360.0)
            })
            .collect()
    }

    #[test]
    fn interior_angles_degrees_reports_a_square_corner_as_90_degrees_either_winding() {
        let ccw = vec![(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)];
        assert_eq!(interior_angles_degrees(&ccw), vec![90.0, 90.0, 90.0, 90.0]);

        let cw: Vec<(f64, f64)> = ccw.into_iter().rev().collect();
        assert_eq!(interior_angles_degrees(&cw), vec![90.0, 90.0, 90.0, 90.0]);
    }

    #[test]
    fn interior_angles_degrees_reports_the_true_reflex_angle_not_its_convex_fold() {
        // The L-shape `ring_overlap_area_handles_a_non_convex_ring_correctly`
        // also uses: a 4x4 square with its own top-right 3x3 corner cut
        // out. The inner corner of that notch, at (1.0, 1.0), is a real
        // 270° reflex vertex -- an undirected angle-between-edges measure
        // would report this as 90° (360° - 270°), indistinguishable from a
        // real 90° convex corner and nowhere near `MAX_INTERIOR_ANGLE_DEGREES`.
        let l_shape = vec![(0.0, 0.0), (4.0, 0.0), (4.0, 1.0), (1.0, 1.0), (1.0, 4.0), (0.0, 4.0)];
        let angles = interior_angles_degrees(&l_shape);
        assert_eq!(
            angles,
            vec![90.0, 90.0, 90.0, 270.0, 90.0, 90.0],
            "expected every outer corner at 90° and the notch's own inner corner at a true 270° reflex"
        );
    }

    #[test]
    fn every_real_barcelona_zone_ring_is_simple_and_has_a_plausible_shape() {
        // An end-to-end coherence sweep over real Barcelona output, not
        // just the specific fixtures above: every ring this crate would
        // actually ship has to be a simple polygon (no self-crossing a
        // client's point-in-polygon test can't reason about) enclosing a
        // physically plausible amount of ground with no spike in its own
        // outline, not just *some* of the narrower failure modes those
        // fixtures each target individually.
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None);
        let collection = to_feature_collection(&network, &zones).expect("building collection");

        let mut failures = Vec::new();
        for feature in &collection.features {
            let id = feature.property("waiting_zone_id").unwrap().as_str().unwrap();
            for (ri, ring) in feature_rings(feature).into_iter().enumerate() {
                let points: Vec<(f64, f64)> =
                    ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
                if ring_self_intersects(ring) {
                    failures.push(format!("{id} ring {ri} self-intersects: {points:?}"));
                    continue;
                }
                let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
                if area_m2 < MIN_PLAUSIBLE_RING_AREA_M2 {
                    failures.push(format!(
                        "{id} ring {ri} has an implausibly small area of {area_m2:.6}m2 \
                         (below {MIN_PLAUSIBLE_RING_AREA_M2}m2): {points:?}"
                    ));
                }
                for (vi, angle) in interior_angles_degrees(&points).into_iter().enumerate() {
                    if angle < MIN_INTERIOR_ANGLE_DEGREES {
                        failures.push(format!(
                            "{id} ring {ri} vertex {vi} has a {angle:.1}° interior angle \
                             (below {MIN_INTERIOR_ANGLE_DEGREES}°) — looks like a convex spike: {points:?}"
                        ));
                    } else if angle > MAX_INTERIOR_ANGLE_DEGREES {
                        failures.push(format!(
                            "{id} ring {ri} vertex {vi} has a {angle:.1}° interior angle \
                             (above {MAX_INTERIOR_ANGLE_DEGREES}°) — looks like a reflex spike: {points:?}"
                        ));
                    }
                }
            }
        }

        assert!(
            failures.is_empty(),
            "{} incoherent ring(s) found among real Barcelona zones:\n{}",
            failures.len(),
            failures.join("\n"),
        );
    }

    #[test]
    fn debug_diagnose_50926861_spike() {
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None);
        let lanes: HashMap<&str, &Lane> = network
            .edges
            .iter()
            .flat_map(|edge| &edge.lanes)
            .map(|lane| (lane.id.0.as_str(), lane))
            .collect();
        let successors = single_successors(&network);
        for id in ["50926861#0_straight", "50926859#0_straight"] {
            let zone = zones.iter().find(|z| z.id.0 == id).unwrap();
            let pre = zone_polygon(zone, &lanes, &successors, MIN_DRAWN_LANE_LENGTH_METERS).unwrap();
            println!("=== {id} PRE-cut: {} part(s) ===", pre.0.len());
            for part in &pre.0 {
                println!("  ring ({} pts): {:?}", part.exterior().0.len(), part.exterior().0);
            }
            let stop = stop_line_point(zone, &lanes).unwrap();
            println!("  stop_line_point: ({}, {})", stop.x, stop.y);
            for entry in &zone.entries {
                let lane = lanes.get(entry.lane.0.as_str()).unwrap();
                println!(
                    "  entry lane {:?} width={:?} first_pt={:?} last_pt={:?}",
                    entry.lane.0,
                    lane.width,
                    lane.shape.0.first(),
                    lane.shape.0.last(),
                );
            }
        }
        let collection = to_feature_collection(&network, &zones).unwrap();
        for id in ["50926861#0_straight", "50926859#0_straight"] {
            let feature = collection
                .features
                .iter()
                .find(|f| f.property("waiting_zone_id").unwrap().as_str().unwrap() == id)
                .unwrap();
            println!("=== {id} POST-cut ===");
            for ring in feature_rings(feature) {
                println!("  ring ({} pts): {:?}", ring.len(), ring);
            }
        }

        // Same as `to_feature_collection`'s internals, but printed in local
        // (un-reprojected) meters so it lines up 1:1 with the PRE-cut dump
        // above -- lets us see exactly which vertices the cut introduced.
        let mut polygons = zones
            .iter()
            .map(|zone| zone_polygon(zone, &lanes, &successors, MIN_DRAWN_LANE_LENGTH_METERS))
            .collect::<Result<Vec<_>>>()
            .unwrap();
        let stop_points = zones
            .iter()
            .map(|zone| stop_line_point(zone, &lanes).map(|p| Coord { x: p.x, y: p.y }))
            .collect::<Result<Vec<_>>>()
            .unwrap();
        resolve_overlaps(&zones, &lanes, &successors, &stop_points, &mut polygons).unwrap();
        for id in ["50926861#0_straight", "50926859#0_straight"] {
            let idx = zones.iter().position(|z| z.id.0 == id).unwrap();
            println!("=== {id} POST-cut (local meters) ===");
            for part in &polygons[idx].0 {
                println!("  ring ({} pts): {:?}", part.exterior().0.len(), part.exterior().0);
            }
        }
    }

    /// A closed rectangle polygon, corners in `(x, y)` — built directly as a
    /// [`MultiPolygon`] rather than a [`Position`] ring, since production
    /// code now carries geometry that way throughout (see the module docs)
    /// and only ever converts to `Position`s at the very end, in
    /// [`build_feature`].
    fn rect(min: (f64, f64), max: (f64, f64)) -> MultiPolygon<f64> {
        MultiPolygon::new(vec![GeoPolygon::new(
            LineString::from(vec![
                (min.0, min.1),
                (max.0, min.1),
                (max.0, max.1),
                (min.0, max.1),
                (min.0, min.1),
            ]),
            Vec::new(),
        )])
    }

    #[test]
    fn subtracting_the_real_overlap_removes_only_the_locally_contested_ground() {
        // A long, straight zone whose real conflict is entirely at its far
        // end (x in [95, 100]) -- the same shape of problem as a real
        // Barcelona bug this guards against (an 80.9m ribbon contested by a
        // neighbour clustered around one small bend near its own far end).
        // A half-plane cut aimed by the two polygons' own overall centroids
        // (an earlier version of `resolve_overlaps` worked this way) could
        // remove most of `a`, far past the real, local overlap; subtracting
        // `a.intersection(b)` directly can only ever remove exactly the
        // disputed ground, wherever it actually is.
        let a = rect((0.0, 0.0), (100.0, 10.0));
        let b = rect((95.0, 3.0), (105.0, 7.0));

        let remaining = a.difference(&a.intersection(&b));
        assert!(
            remaining.unsigned_area() > 900.0,
            "expected to keep almost all of a's own 1000 units\u{b2}, losing only the small \
             overlap with b near its far end -- got {:.1}",
            remaining.unsigned_area()
        );
        assert_eq!(
            remaining.intersection(&b).unsigned_area(),
            0.0,
            "a and b must not overlap any more after a gives back their real overlap"
        );
    }

    #[test]
    fn a_long_bent_ribbon_keeps_most_of_its_own_area_when_only_one_end_is_contested() {
        // The real Barcelona ribbon this bug was found on: `-27641458#20_1`
        // is an 80.9m lane with one gentle bend, contested near that bend
        // by three separate neighbouring zones at once. A hand-rolled
        // clipper that anchored on the two rings' own centroids instead of
        // the real overlap, and that couldn't safely apply more than one
        // cutting constraint at once against a non-convex base, used to
        // collapse this to a ~53m² disconnected notch nowhere near any of
        // the three real overlaps; `bisecting_cut` (anchored on the real
        // overlap) and `geo::BooleanOps::difference` (exact regardless of
        // convexity or how many cuts are applied) should instead keep the
        // great majority of the ribbon's own ~260m² and lose only a small
        // piece near the bend.
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None);
        let collection = to_feature_collection(&network, &zones).expect("building collection");

        let feature = collection
            .features
            .iter()
            .find(|f| f.property("waiting_zone_id").unwrap().as_str().unwrap() == "-27641458#20_straight")
            .expect("zone found");
        let rings = feature_rings(feature);
        assert_eq!(rings.len(), 1, "this zone is a single lane's own core ring, no ancestor extension");
        let points: Vec<(f64, f64)> = rings[0][..rings[0].len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
        assert!(
            area_m2 > 130.0,
            "expected to keep most of this 80.9m-lane ribbon's own ~260m², losing only a \
             small piece near its bend to the neighbours contesting it there -- got \
             {area_m2:.1}m2, suspiciously close to the old collapsed-notch bug's ~53m²: \
             {points:?}"
        );
    }

}

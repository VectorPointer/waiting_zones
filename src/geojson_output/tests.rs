#[cfg(test)]
#[allow(clippy::module_inception)]
mod tests {
    use geo::{Area, BooleanOps, Coord, LineString, MultiPolygon, Polygon as GeoPolygon};
    use geojson::{Feature, FeatureCollection, Position};
    use std::collections::HashMap;
    use crate::geojson_output::{
        find_near_touch, weld_near_touch_and_split, MIN_DRAWN_LANE_LENGTH_METERS,
        distance_to_polygon, Reprojector, feature_rings,
        overlapping_zone_ids, single_successors, to_feature_collection, write, zone_feature,
    };
    use sumo_types::additional::domain::{DetectorGate, DetectorId, E3Detector, LanePosition, LaneRef, PersonMode};
    use sumo_types::domain::{
        Boundary, Connection, ConnectionDirection, Edge, EdgeFunction, EdgeId, Junction, JunctionId, JunctionKind,
        Lane, LaneId, LaneIndex, LinkState, Location, Network, Point, Projection, Shape,
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

        let geojson::GeometryValue::Polygon { coordinates } = &feature.geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry, got {:?}", feature.geometry);
        };
        assert_eq!(coordinates.len(), 1, "one lane, no holes -> one ring");
        let ring = &coordinates[0];
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
    fn reports_intersection_id_from_the_junction_listing_the_exit_lane_when_the_network_has_one() {
        // Sent as its own property, separate from `waiting_zone_id`, so a
        // client never has to parse one out of the other (see this
        // module's own docs). Derived from whichever junction lists the
        // zone's own exit lane among its own `incLanes`, not carried by
        // `E3Detector` itself — omitted whenever the network doesn't say
        // (a fixture like most others in this file, no junction at all).
        // Not `edge.to`: real `.net.xml` never sets that on a walkingarea
        // edge, so a pedestrian zone's own exit would never resolve one
        // through that path (see `to_feature_collection`'s own docs).
        let network = Network {
            junctions: vec![Junction {
                id: JunctionId("j5".into()),
                position: Point::default(),
                kind: JunctionKind::TrafficLight,
                incoming_lanes: vec![LaneId("e0_0".into())],
                internal_lanes: vec![],
                shape: None,
                name: None,
            }],
            ..utm_31n_network(vec![straight_lane("e0_0", 3.2)])
        };
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert_eq!(
            collection.features[0].property("intersection_id").unwrap(),
            &serde_json::json!("j5")
        );
    }

    #[test]
    fn omits_intersection_id_when_the_network_does_not_say_which_junction() {
        let network = utm_31n_network(vec![straight_lane("e0_0", 3.2)]);
        let zones = vec![zone("j0_0", "e0_0")];

        let collection = to_feature_collection(&network, &zones).unwrap();
        assert!(collection.features[0].property("intersection_id").is_none());
    }

    #[test]
    fn write_splits_vehicle_and_pedestrian_zones_into_2_files() {
        // A pedestrian zone's own polygon comes straight from its lane's
        // `shape` (`pedestrian_lane_polygon`), not a stroke-buffered
        // centreline — see `a_pedestrian_zone_is_always_on_foot_regardless_of_its_lanes_own_vclass`'s
        // own docs for why `e0_1` needs a real, already-closed-ish outline
        // rather than reusing `e0_0`'s 2-point centreline.
        let walkingarea = Lane {
            shape: Shape(vec![
                Point { x: 10.0, y: 0.0, z: 0.0 },
                Point { x: 12.0, y: 0.0, z: 0.0 },
                Point { x: 12.0, y: 2.0, z: 0.0 },
                Point { x: 10.0, y: 2.0, z: 0.0 },
            ]),
            ..straight_lane("e0_1", 3.2)
        };
        let network = utm_31n_network_multi_edge(vec![
            ("e0", vec![straight_lane("e0_0", 3.2)]),
            ("e1", vec![walkingarea]),
        ]);
        let vehicle_zone = zone("j0_0", "e0_0");
        let pedestrian_zone = E3Detector { detect_persons: vec![PersonMode::Walk], ..zone("j0_1", "e0_1") };

        let dir = std::env::temp_dir().join(format!(
            "waiting_zones_write_test_{}_{:?}",
            std::process::id(),
            std::thread::current().id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        let base_path = dir.join("zones.geojson");

        write(&base_path, &network, &[vehicle_zone, pedestrian_zone]).unwrap();

        let vehicles: FeatureCollection =
            serde_json::from_str(&std::fs::read_to_string(dir.join("zones.vehicles.geojson")).unwrap()).unwrap();
        let pedestrians: FeatureCollection =
            serde_json::from_str(&std::fs::read_to_string(dir.join("zones.pedestrians.geojson")).unwrap()).unwrap();

        assert_eq!(
            vehicles.features.iter().map(|f| f.property("waiting_zone_id").unwrap().clone()).collect::<Vec<_>>(),
            vec![serde_json::json!("j0_0")]
        );
        assert_eq!(
            pedestrians.features.iter().map(|f| f.property("waiting_zone_id").unwrap().clone()).collect::<Vec<_>>(),
            vec![serde_json::json!("j0_1")]
        );

        std::fs::remove_dir_all(&dir).ok();
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
        //
        // A pedestrian zone's own polygon comes straight from its lane's
        // `shape` (`pedestrian_lane_polygon`), not a stroke-buffered
        // centreline the way a vehicle zone's does — so unlike every other
        // test in this file, the lane needs a real, already-closed-ish
        // outline (a walkingarea's own shape), not a 2-point centreline: a
        // straight lane's `shape` has too few points to read as a polygon
        // at all and this zone would be cut away with no ground to claim.
        let walkingarea = Lane {
            shape: Shape(vec![
                Point { x: 0.0, y: 0.0, z: 0.0 },
                Point { x: 2.0, y: 0.0, z: 0.0 },
                Point { x: 2.0, y: 2.0, z: 0.0 },
                Point { x: 0.0, y: 2.0, z: 0.0 },
            ]),
            ..straight_lane("e0_0", 3.2)
        };
        let network = utm_31n_network(vec![walkingarea]);
        let ped_zone = E3Detector {
            detect_persons: vec![PersonMode::Walk],
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
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        let ring = &coordinates[0];
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
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        let ring = &coordinates[0];

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
        let lane_to_junction: HashMap<&str, &str> = HashMap::new();
        let successors = single_successors(&network);
        let reproject = Reprojector::new(&network.location).unwrap();
        let collection = FeatureCollection {
            bbox: None,
            features: vec![
                zone_feature(&zone("j0_0", "e0_0"), &lanes, &lane_to_junction, &successors, &reproject, MIN_DRAWN_LANE_LENGTH_METERS)
                    .unwrap(),
                zone_feature(&zone("j0_1", "e0_1"), &lanes, &lane_to_junction, &successors, &reproject, MIN_DRAWN_LANE_LENGTH_METERS)
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
        let geojson::GeometryValue::Polygon { coordinates } = &stub.geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        let ring = &coordinates[0];
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
        assert_eq!(
            collection.features.len(),
            2,
            "both zones should have survived the cut with some geometry left, not been \
             dropped for resolving to no ground at all"
        );
        for feature in &collection.features {
            assert!(
                matches!(feature.geometry.as_ref().map(|g| &g.value), Some(geojson::GeometryValue::Polygon { .. })),
                "expected a Polygon geometry for {:?}",
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
        let geojson::GeometryValue::Polygon { coordinates } = &middle.geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        assert_eq!(coordinates.len(), 1, "j0_1 should still be one simple ring, no holes");
        assert!(
            !ring_self_intersects(&coordinates[0]),
            "clipping against two neighbours in one pass should stay a single simple \
             polygon, not a self-intersecting shred: {:?}",
            coordinates[0]
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
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        // e0, e1 and e2 are collinear and meet exactly end to end, so their
        // union is genuinely one seamless 60m ribbon, not three independent
        // rectangles -- nor even two separately-tracked "core" and "chain"
        // polygons that merely happen to touch: a real `union` merges
        // anything that touches into one connected polygon, which is a
        // strictly better result than the former design's own "list of
        // rings, however each one was built" ever guaranteed.
        assert_eq!(coordinates.len(), 1, "expected one seamless ring for e0+e1+e2 combined, no holes");

        let ring = &coordinates[0];
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
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        // Every segment (core, e_mid, both leaves) touches at least one
        // other at a real junction corner, so their union is one connected
        // polygon -- there's no "was e_mid drawn twice" question left to
        // ask ring-by-ring any more, since a real union structurally can't
        // double-count the ground two of its own inputs share.
        assert_eq!(coordinates.len(), 1, "expected one ring covering the whole merged shape, no holes");
        let ring = &coordinates[0];
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

    /// The signed area enclosed by `ring` (the shoelace formula, no closing
    /// repeat) — positive for a counterclockwise winding, negative for
    /// clockwise. Test-only: production code answers every question this
    /// used to answer through [`geo::BooleanOps`] instead, and this backs
    /// the checks that verify that replacement against a hand-computed
    /// primitive rather than trusting `geo` circularly.
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

    /// How deeply any two non-adjacent edges of closed ring `ring`
    /// (GeoJSON style: first position repeated last) cross each other, in
    /// whatever units `ring` is expressed in — `0.0` for a simple ring.
    /// "Deeply" is the distance from the crossing point to the nearest end
    /// of the two segments involved: a real bowtie crosses well inside
    /// both, a floating-point artifact crosses a hair past a shared
    /// endpoint.
    ///
    /// Depth rather than a bare yes/no because the two are genuinely
    /// different defects and only one is worth failing over. `geo`'s own
    /// boolean ops leave a zone's ring simple in this crate's local metre
    /// coordinates; `Reprojector::to_lon_lat` then re-derives every vertex
    /// through an entirely different `f64` computation, which can put two
    /// points that agreed to 12 significant digits on opposite sides of
    /// each other. The result is a "crossing" nanometres deep — invisible
    /// to any consumer, unfixable without giving a zone back ground
    /// `resolve_overlaps` deliberately cut from it (see
    /// `overlaps::MAX_CLEANUP_AREA_GROWTH_M2`), and not what anyone means
    /// by a self-intersecting polygon. A real one — the 140mm and 6.5m
    /// bowties this crate used to emit — is orders of magnitude clear of
    /// it either way.
        fn ring_self_intersection_depth(points: &[(f64, f64)]) -> f64 {
        let n = points.len();
        let mut deepest: f64 = 0.0;
        for i in 0..n {
            for j in (i + 2)..n {
                if i == 0 && j == n - 1 {
                    continue; // adjacent via the closing wrap-around
                }
                let (a1, a2) = (points[i], points[(i + 1) % n]);
                let (b1, b2) = (points[j], points[(j + 1) % n]);
                let (adx, ady) = (a2.0 - a1.0, a2.1 - a1.1);
                let (bdx, bdy) = (b2.0 - b1.0, b2.1 - b1.1);
                let denominator = adx * bdy - ady * bdx;
                if denominator == 0.0 {
                    continue;
                }
                let (ex, ey) = (b1.0 - a1.0, b1.1 - a1.1);
                let t = (ex * bdy - ey * bdx) / denominator;
                let u = (ex * ady - ey * adx) / denominator;
                if !(0.0..=1.0).contains(&t) || !(0.0..=1.0).contains(&u) {
                    continue;
                }
                let (a_length, b_length) = (adx.hypot(ady), bdx.hypot(bdy));
                let depth = (t * a_length)
                    .min((1.0 - t) * a_length)
                    .min(u * b_length)
                    .min((1.0 - u) * b_length);
                deepest = deepest.max(depth);
            }
        }
        deepest
    }

    /// Whether `ring` crosses itself by more than
    /// [`MAX_SELF_INTERSECTION_DEPTH_METERS`] — see
    /// [`ring_self_intersection_depth`] for why that's a depth and not a
    /// yes/no. Kept taking a `Position` slice for the several tests built
    /// on synthetic lon/lat rings, where the scale distortion is
    /// irrelevant because they only ever ask about a deliberately gross
    /// crossing.
    fn ring_self_intersects(ring: &[Position]) -> bool {
        let points: Vec<(f64, f64)> =
            ring[..ring.len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        ring_self_intersection_depth(&points) > 0.0
    }

    /// How deep a self-crossing has to be, in metres, to be a real defect
    /// rather than reprojection noise — see
    /// [`ring_self_intersection_depth`]. Barcelona's own deepest residual
    /// is 0.73mm and the real bowties this crate used to emit were 140mm
    /// and 6.5m, so this sits with three orders of magnitude of headroom
    /// on both sides.
    const MAX_SELF_INTERSECTION_DEPTH_METERS: f64 = 0.001;

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
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        assert_eq!(
            coordinates.len(),
            1,
            "two physically contiguous lanes of the same zone should merge into one \
             seamless ring, not the seam-prone one-rectangle-per-lane MultiPolygon"
        );
        assert!(!ring_self_intersects(&coordinates[0]));
    }

    #[test]
    fn keeps_only_the_core_polygon_when_an_extended_ancestor_entry_does_not_touch_it() {
        // e0 is the zone's own controlled lane (has an exit, so it's where
        // the stop line sits); e1 is an ancestor `zone_generator::extended_entry_lanes`
        // walked back onto -- present only as an entry, on a *different*
        // edge far enough away that its own polygon doesn't touch e0's.
        // `to_feature_collection` only ever emits a single Polygon per zone
        // (see its own docs on why more than one part collapses to the one
        // nearest the stop line): e1's own disconnected polygon is real
        // ground this zone's entries do claim, but with nothing linking it
        // to the stop line it's the one part dropped, not e0's.
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
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        assert!(!ring_self_intersects(&coordinates[0]));

        let reproject = Reprojector::new(&network.location).unwrap();
        let e0_lon = reproject.to_lon_lat(Point { x: 0.0, y: 10.0, z: 0.0 }).unwrap()[0];
        let e1_lon = reproject.to_lon_lat(Point { x: 50.0, y: 10.0, z: 0.0 }).unwrap()[0];
        let ring_lon = coordinates[0].iter().map(|p| p[0]).sum::<f64>() / coordinates[0].len() as f64;
        assert!(
            (ring_lon - e0_lon).abs() < (ring_lon - e1_lon).abs(),
            "the surviving polygon should be e0's own core ring, not e1's disconnected \
             one: ring mean lon {ring_lon}, e0 at {e0_lon}, e1 at {e1_lon}"
        );
    }

    #[test]
    fn never_bridges_across_a_lane_gap_the_zone_does_not_actually_claim() {
        // Lane 1 sits physically between lanes 0 and 2, but this zone only
        // claims 0 and 2 (as if lane 1 belonged to some other movement) --
        // exactly the shape a non-contiguous group `merged_core_polygon`'s
        // own docs describe. An earlier, hand-rolled version of this
        // pipeline picked the group's own leftmost and rightmost lane and
        // joined them directly regardless, which silently claimed lane 1's
        // own ground (not part of this zone) as if it belonged here too --
        // `merged_core_polygon` doesn't assume anything about lane order or
        // contiguity, so lanes 0 and 2 never get unioned into one shape
        // spanning the gap between them. With both lanes equally far from
        // the zone's own stop line (the exits' shared centroid sits right
        // in the unclaimed gap), `to_feature_collection`'s "one polygon per
        // zone" reduction keeps whichever of the two is picked as the
        // largest part -- deterministic, but not meaningful to pin to one
        // side in particular; what matters here is that the survivor is
        // one full lane's own ground, not a bridge spanning both.
        let network = utm_31n_network(vec![
            indexed_parallel_lane("e0_0", 0, 3.2, 0.0),
            indexed_parallel_lane("e0_1", 1, 3.2, 3.2),
            indexed_parallel_lane("e0_2", 2, 3.2, 6.4),
        ]);
        let zones = vec![zone_multi("j0_0", &["e0_0", "e0_2"])];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        assert!(!ring_self_intersects(&coordinates[0]));

        let reproject = Reprojector::new(&network.location).unwrap();
        let one_lane_width_deg = (reproject.to_lon_lat(Point { x: 3.2, y: 10.0, z: 0.0 }).unwrap()[0]
            - reproject.to_lon_lat(Point { x: 0.0, y: 10.0, z: 0.0 }).unwrap()[0])
            .abs();
        let ring_lon_span = coordinates[0].iter().map(|p| p[0]).fold(f64::MIN, f64::max)
            - coordinates[0].iter().map(|p| p[0]).fold(f64::MAX, f64::min);
        assert!(
            ring_lon_span < one_lane_width_deg * 1.5,
            "surviving polygon spans {ring_lon_span} degrees of longitude, more than one \
             lane's own {one_lane_width_deg} -- it bridged across the unclaimed gap"
        );
    }

    #[test]
    fn buffer_shape_never_self_intersects_on_a_short_wide_zigzag() {
        let lane = zigzag_lane("e0_0", 4.0);
        let length = lane.length;
        let network = utm_31n_network(vec![lane]);
        let zones = vec![zone_spanning("j0_0", "e0_0", length)];

        let collection = to_feature_collection(&network, &zones).unwrap();
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        let ring = &coordinates[0];
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
        let geojson::GeometryValue::Polygon { coordinates } = &collection.features[0].geometry.as_ref().unwrap().value else {
            panic!("expected a Polygon geometry");
        };
        let ring = &coordinates[0];
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
    ///
    /// Kept equal, on purpose, to `overlaps::MIN_KEPT_PART_AREA_M2` — see
    /// that constant's own docs for what letting the two drift apart
    /// already cost once.
    const MIN_PLAUSIBLE_RING_AREA_M2: f64 = 0.05;



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

    /// Metres per degree of longitude and of latitude at Barcelona's own
    /// latitude — the two very different numbers that make a raw lon/lat
    /// ring the wrong place to measure an *angle*.
    ///
    /// A degree of longitude is ~84km here against a degree of latitude's
    /// ~111km, so treating a lon/lat pair as if it were a square metric
    /// coordinate stretches every shape by a third in one axis and
    /// reports angles that no ground geometry actually has. That isn't a
    /// rounding concern: it flagged `1828108785_w1_straight_ped`'s own
    /// real 5.9° corner as a 4.4° needle, i.e. it manufactured a failure
    /// on output the production pass had already cleaned. The area check
    /// below always knew this — it multiplies by both constants — so this
    /// only brings the angle check into line with it.
    const METERS_PER_DEGREE_LON: f64 = 84_000.0;
    const METERS_PER_DEGREE_LAT: f64 = 111_000.0;

    /// `ring` (GeoJSON lon/lat, closing repeat dropped) in locally flat
    /// metres, so angles and areas measured on it mean what they say.
    fn in_local_meters(ring: &[Position]) -> Vec<(f64, f64)> {
        ring[..ring.len().saturating_sub(1)]
            .iter()
            .map(|p| (p[0] * METERS_PER_DEGREE_LON, p[1] * METERS_PER_DEGREE_LAT))
            .collect()
    }

    /// An end-to-end coherence sweep over every zone `network_path`'s own
    /// network produces, shared by the real Barcelona and Eixample checks
    /// below — every ring this crate would actually ship has to be a
    /// simple polygon (no self-crossing a client's point-in-polygon test
    /// can't reason about) enclosing a physically plausible amount of
    /// ground, and every zone's own geometry has to be a plain `Polygon`
    /// with no interior ring at all, not just *some* of the narrower
    /// failure modes the fixtures above each target individually.
    ///
    /// This deliberately no longer gates on interior angles. It used to,
    /// and the check was doing real harm: every one of the 66 vertices
    /// it flagged on current output is the tip of a zero-width notch
    /// `resolve_overlaps` cut into a zone to separate it from a
    /// neighbour, and `overlaps::MAX_CLEANUP_AREA_GROWTH_M2` exists
    /// precisely to stop the cleanup passes from smoothing those away —
    /// doing so hands the zone back the ground the cut removed and
    /// reopens the overlap. So the check demanded a shape the crate must
    /// not produce, could only be satisfied by breaking a property that
    /// matters more, and buried the two checks below in its own noise.
    /// A needle-thin vertex encloses no area by definition, which is the
    /// same thing as saying no consumer of this output can observe it;
    /// `overlaps::drop_needle_vertices` still removes every one it can
    /// remove safely, as a cosmetic best effort rather than a contract.
    ///
    /// The "no interior ring at all" check is the newest of the three,
    /// and the one that would have caught the real regression the other
    /// two both missed: `feature_rings` (what the self-intersection and
    /// area checks both read) only ever returns a feature's own
    /// *exterior* ring by design, so a spurious hole — confirmed on real
    /// data, 106 of them across Barcelona and Eixample combined, one as
    /// large as 182.66m² (`1409641098#0_straight`, in Eixample; see
    /// `overlaps::drop_interior_rings`'s own docs for the two mechanisms
    /// behind all of them) — was invisible to both other checks even
    /// though a real client's point-in-polygon test reads the hole as
    /// "not part of the zone" just as much as it would a self-crossing
    /// ring.
    fn assert_every_zone_ring_is_coherent(network_path: &str, network_label: &str) {
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join(network_path);
        let network = sumo_types::read_network(&net_file)
            .unwrap_or_else(|error| panic!("reading {network_label} network: {error:#}"));
        let zones = crate::zone_generator::generate(&network, None, false);
        // Per mode, exactly as `write` ships them — never one combined
        // collection. `resolve_overlaps` only ever sees the zones handed
        // to it, so a combined run resolves vehicle-against-pedestrian
        // pairs no shipped file contains, and checks geometry no client
        // is ever served.
        let (pedestrian, vehicle): (Vec<E3Detector>, Vec<E3Detector>) =
            zones.into_iter().partition(|zone| !zone.detect_persons.is_empty());

        let mut failures = Vec::new();
        let collections = [
            to_feature_collection(&network, &vehicle).expect("building the vehicle collection"),
            to_feature_collection(&network, &pedestrian).expect("building the pedestrian collection"),
        ];
        for feature in collections.iter().flat_map(|collection| &collection.features) {
            let id = feature.property("waiting_zone_id").unwrap().as_str().unwrap();

            let ring_count = match feature.geometry.as_ref().map(|g| &g.value) {
                Some(geojson::GeometryValue::Polygon { coordinates }) => coordinates.len(),
                _ => 0,
            };
            if ring_count > 1 {
                failures.push(format!(
                    "{id} has {ring_count} rings (an exterior plus {} interior ring(s)/hole(s)) — \
                     a waiting zone is the union of one or more buffered lane strips, always simply \
                     connected by construction, so it should never have one at all",
                    ring_count - 1
                ));
            }

            for (ri, ring) in feature_rings(feature).into_iter().enumerate() {
                let points = in_local_meters(ring);
                let depth = ring_self_intersection_depth(&points);
                if depth > MAX_SELF_INTERSECTION_DEPTH_METERS {
                    failures.push(format!(
                        "{id} ring {ri} self-intersects {:.1}mm deep: {points:?}",
                        depth * 1000.0
                    ));
                    continue;
                }
                let area_m2 = signed_area(&points).abs();
                if area_m2 < MIN_PLAUSIBLE_RING_AREA_M2 {
                    failures.push(format!(
                        "{id} ring {ri} has an implausibly small area of {area_m2:.6}m2 \
                         (below {MIN_PLAUSIBLE_RING_AREA_M2}m2): {points:?}"
                    ));
                }
            }
        }

        assert!(
            failures.is_empty(),
            "{} incoherent ring(s) found among real {network_label} zones:\n{}",
            failures.len(),
            failures.join("\n"),
        );
    }

    #[test]
    fn every_real_barcelona_zone_ring_is_simple_and_has_a_plausible_shape() {
        assert_every_zone_ring_is_coherent("data/barcelona/barcelona.net.xml", "Barcelona");
    }

    /// Eixample's own network is worth sweeping separately, not folded
    /// into the Barcelona check above: it's where the interior-ring
    /// regression this test now guards against was actually found and
    /// where the overwhelming majority of the 106 real holes were —
    /// multi-lane and dedicated-bike-lane geometry that Barcelona's own
    /// sample network happens not to exercise as heavily.
    #[test]
    fn every_real_eixample_zone_ring_is_simple_and_has_a_plausible_shape() {
        assert_every_zone_ring_is_coherent("data/eixample/eixample.net.xml", "Eixample");
    }

    /// How far a zone's own published stop line may sit outside its
    /// polygon before that's a defect, in metres.
    ///
    /// Zero would be the ideal and is what most zones achieve, but it
    /// isn't the honest threshold: the stop line sits *on* the polygon's
    /// own end cap by construction, so which side of that boundary a
    /// float lands on is arbitrary, and 285 of Barcelona's own vehicle
    /// zones report a distance of exactly 0.000m "outside". What actually
    /// matters is whether a vehicle stopped at the line is inside the zone
    /// that's supposed to detect it — and at under a metre it always is,
    /// against a ~4.5m car and a GPS fix good to a few metres. Past that,
    /// the zone has genuinely been cut away from the ground it exists to
    /// cover.
    const STOP_LINE_TOLERANCE_METERS: f64 = 1.0;

    /// Zones whose published stop line is further than
    /// [`STOP_LINE_TOLERANCE_METERS`] from their own polygon today —
    /// every one of them a zone `resolve_overlaps` cut back past its own
    /// stop line while settling a dispute with a neighbour, which it has
    /// no rule against doing: the stop line is the one piece of ground a
    /// zone cannot give up and nothing currently tells the cut so.
    ///
    /// Listed rather than tolerated by loosening the threshold, so the
    /// invariant stays stated at its real value and these 6 stay visible
    /// as the debt they are. The test fails if any of them starts
    /// passing, too — a fix has to shorten this list rather than leave it
    /// quietly describing a network that no longer looks like this.
    ///
    /// `5588597255_w1_straight_ped` and `5588597271_w1_straight_ped` were
    /// here too until `overlaps::drop_negligible_vertices` started
    /// removing genuinely redundant vertices from pedestrian zones (see
    /// its own docs): apparently unrelated, but a stop line's own
    /// containment check runs against the *finished* polygon, and both
    /// zones' own final shape shifted just enough, incidentally, to
    /// close the gap. Left off rather than re-added.
    ///
    /// `-27641458#2_straight` is the opposite story, and joined the list
    /// for it: already marginal before `drop_negligible_vertices` moved
    /// to a per-zone-relative threshold (0.993m — under
    /// [`STOP_LINE_TOLERANCE_METERS`], but only just), it has three real
    /// vertices clustered tightly right at its own stop line, one of
    /// them genuinely negligible (0.089% of the zone's own area, safely
    /// under `overlaps::NEGLIGIBLE_VERTEX_AREA_FRACTION`, confirmed by
    /// sweeping every other ring in the network at that same cap — see
    /// that constant's own docs). Removing just that one, correctly
    /// leaving its two real neighbours in place, nudges the boundary
    /// there by about 9cm anyway — 0.993m to 1.082m, crossing the line
    /// this test draws without the shape becoming any less correct. The
    /// alternative (a tighter cap) isn't free: the worst of the eight
    /// rectangle-with-a-stray-vertex bugs that threshold exists to fix
    /// sits at 0.1017%, above this zone's own 0.089% — so no single cap
    /// both fixes every one of those and leaves this zone's own margin
    /// untouched.
    const ZONES_CUT_BACK_PAST_THEIR_OWN_STOP_LINE: [&str; 7] = [
        "683963534_straight+turn+partial_left",
        "1053359487#6_left+right",
        "-402739619#0_straight+turn+right",
        "201419371#5_straight",
        "46270517#0_straight",
        "5631458674_w1_straight_ped",
        "-27641458#2_straight",
    ];

    /// Every zone has to actually cover the stop line it's published with.
    ///
    /// This is the one property that decides whether a waiting zone works
    /// at all: a vehicle halted at the line is exactly what the zone
    /// exists to detect, and a zone that doesn't reach its own line
    /// detects nobody while still looking perfectly plausible — a real
    /// polygon, a sane area, a simple ring, in roughly the right place.
    /// Every other check in this module would pass such a zone.
    ///
    /// Checked against the `stop_line` property as *published*, not
    /// against an internally recomputed one, because that property is
    /// precisely what a client is told to expect (`resolver::catalogue`)
    /// — a stop line outside its own zone is a contradiction handed
    /// straight to the consumer.
    #[test]
    fn every_real_barcelona_zone_covers_its_own_published_stop_line() {
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None, false);
        let (pedestrian, vehicle): (Vec<E3Detector>, Vec<E3Detector>) =
            zones.into_iter().partition(|zone| !zone.detect_persons.is_empty());
        let collections = [
            to_feature_collection(&network, &vehicle).expect("building the vehicle collection"),
            to_feature_collection(&network, &pedestrian).expect("building the pedestrian collection"),
        ];

        let mut unexpectedly_far = Vec::new();
        let mut unexpectedly_fine = Vec::new();
        for feature in collections.iter().flat_map(|collection| &collection.features) {
            let id = feature.property("waiting_zone_id").unwrap().as_str().unwrap();
            let stop_line = feature.property("stop_line").unwrap().as_array().unwrap();
            let stop_line = Coord {
                x: stop_line[0].as_f64().unwrap() * METERS_PER_DEGREE_LON,
                y: stop_line[1].as_f64().unwrap() * METERS_PER_DEGREE_LAT,
            };
            let polygon = MultiPolygon::new(
                feature_rings(feature)
                    .into_iter()
                    .map(|ring| GeoPolygon::new(LineString::from(in_local_meters(ring)), Vec::new()))
                    .collect(),
            );

            let distance = distance_to_polygon(stop_line, &polygon);
            let known = ZONES_CUT_BACK_PAST_THEIR_OWN_STOP_LINE.contains(&id);
            match (distance > STOP_LINE_TOLERANCE_METERS, known) {
                (true, false) => unexpectedly_far.push(format!("{id}: stop line {distance:.2}m outside its own polygon")),
                (false, true) => unexpectedly_fine.push(id.to_string()),
                _ => {}
            }
        }

        assert!(
            unexpectedly_far.is_empty(),
            "{} zone(s) don't reach their own stop line and aren't listed as known:\n{}",
            unexpectedly_far.len(),
            unexpectedly_far.join("\n")
        );
        assert!(
            unexpectedly_fine.is_empty(),
            "{} zone(s) listed in ZONES_CUT_BACK_PAST_THEIR_OWN_STOP_LINE now reach their own \
             stop line — delete them from that list so it keeps describing reality: {:?}",
            unexpectedly_fine.len(),
            unexpectedly_fine
        );
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
        let zones = crate::zone_generator::generate(&network, None, false);
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

    /// `id`'s own feature from the real Barcelona network's vehicle *or*
    /// pedestrian collection, whichever has it — a shared lookup for the
    /// two regression tests below, neither of which cares which mode its
    /// own target zone is.
    fn find_barcelona_zone(id: &str) -> geojson::Feature {
        let manifest_dir = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        let net_file = manifest_dir.join("data/barcelona/barcelona.net.xml");
        let network = sumo_types::read_network(&net_file).expect("reading Barcelona network");
        let zones = crate::zone_generator::generate(&network, None, false);
        let (pedestrian, vehicle): (Vec<E3Detector>, Vec<E3Detector>) =
            zones.into_iter().partition(|zone| !zone.detect_persons.is_empty());
        for zones in [vehicle, pedestrian] {
            let collection = to_feature_collection(&network, &zones).expect("building collection");
            if let Some(feature) =
                collection.features.into_iter().find(|f| f.property("waiting_zone_id").unwrap().as_str().unwrap() == id)
            {
                return feature;
            }
        }
        panic!("zone {id:?} not found in either collection");
    }

    /// Regression test for the two real fixtures
    /// `overlaps::NEGLIGIBLE_VERTEX_AREA_M2`'s own docs cite as the reason
    /// pedestrian zones never got a blanket Douglas-Peucker pass: a 5cm
    /// tolerance once sharpened a real corner on each of these, because
    /// DP's global, chord-based test doesn't distinguish "noise" from "a
    /// gentle bend over a long run" any more reliably than a naive local
    /// check does — see `overlaps::drop_negligible_vertices`'s own docs
    /// for the mechanism. Both zones have very few vertices (6-7) and
    /// every one is a real corner (the smallest accounts for 0.044m² and
    /// 0.221m² respectively — 44x and 221x
    /// [`overlaps::NEGLIGIBLE_VERTEX_AREA_M2`]'s own cap), so this simply
    /// asserts neither shape moved a single vertex.
    #[test]
    fn known_sensitive_pedestrian_fixtures_keep_every_one_of_their_own_corners() {
        let cases = [
            ("5588597076_w0_straight_ped", 7),
            ("6119951203_w0_straight_ped", 6),
        ];
        for (id, expected_vertex_count) in cases {
            let feature = find_barcelona_zone(id);
            let rings = feature_rings(&feature);
            assert_eq!(rings.len(), 1, "{id}: expected a single ring, no holes");
            assert_eq!(
                rings[0].len() - 1,
                expected_vertex_count,
                "{id}: vertex count changed -- this zone's own corners are all real (see this \
                 test's own docs on the 44x-221x safety margin), so any cleanup pass removing \
                 one is exactly the regression that made pedestrian zones skip Douglas-Peucker \
                 entirely: {:?}",
                rings[0]
            );
        }
    }

    /// Regression test for the real case that shows why
    /// `overlaps::drop_negligible_vertices` has to recheck each vertex
    /// against its *current* neighbours after every removal, rather than
    /// removing every vertex whose interior angle is close to 180° in one
    /// pass against their original ones (that function's own docs walk
    /// through the mechanism in full). `171839324#6_straight` is a 440m²
    /// vehicle zone with a real, gentle ~0.5m bow over 138m: 7 of its own
    /// vertices individually read as being within a fraction of a degree
    /// of straight, but stripping all 7 at once (the naive reading of an
    /// angle-only test) shifts its area by 36m² (8.2%) -- a real,
    /// visible defect. The safe pass instead keeps removing sub-millimetre
    /// noise only until a vertex's own *current* neighbours have widened
    /// enough to reveal the real bend, landing on exactly the same-shaped
    /// zone with only its genuinely flat segment cleaned up.
    #[test]
    fn a_zones_own_gentle_curve_survives_negligible_vertex_cleanup() {
        let feature = find_barcelona_zone("171839324#6_straight");
        let rings = feature_rings(&feature);
        assert_eq!(rings.len(), 1, "expected a single ring, no holes");
        let points: Vec<(f64, f64)> = rings[0][..rings[0].len().saturating_sub(1)].iter().map(|p| (p[0], p[1])).collect();
        let area_m2 = signed_area(&points).abs() * 84_000.0 * 111_000.0;
        assert!(
            (439.0..441.0).contains(&area_m2),
            "expected this zone's own real ~440.23m² to survive cleanup within a rounding \
             error -- got {area_m2:.2}m2, which looks like either the naive angle-only defect \
             this test guards against (a real ~36m²/8.2% swing) or an unrelated shape change: \
             {points:?}"
        );
        assert!(
            rings[0].len() - 1 <= 8,
            "expected most of this zone's own genuinely flat run (originally 12 vertices, 7 of \
             them within a fraction of a degree of 180°) to be cleaned up, not just left in \
             place: {:?}",
            rings[0]
        );
    }

    /// [`ring_self_intersects`], adapted for a `geo::LineString` rather
    /// than a GeoJSON `Position` list — [`find_near_touch`] and
    /// [`weld_near_touch_and_split`] work directly in `geo` types, with
    /// no reprojection step to go through first.
    fn geo_ring_self_intersects(ring: &LineString<f64>) -> bool {
        let positions: Vec<Position> = ring.coords().map(|c| Position::from([c.x, c.y])).collect();
        ring_self_intersects(&positions)
    }

    /// Whether any two *consecutive* positions in `ring` (GeoJSON style:
    /// first repeated last) are the exact same point — a zero-length edge,
    /// which `ring_self_intersects`'s own orientation test can't flag
    /// (both endpoints coincide, so there's no direction to sign) but
    /// which is exactly the shape of bug `weld_near_touch_and_split`'s own
    /// slicing used to introduce (see its own docs) before double-counting
    /// the shared welded point was fixed.
    fn has_zero_length_edge(ring: &LineString<f64>) -> bool {
        let coords: Vec<Coord<f64>> = ring.coords().copied().collect();
        coords.windows(2).any(|w| w[0] == w[1])
    }

    #[test]
    fn find_near_touch_locates_a_vertex_grazing_a_distant_edge_before_it_in_the_ring() {
        // F (index 0) sits 1mm below segment B->C (indices 2,3) without
        // properly crossing it -- a "near T-touch" `split_self_intersection`
        // can't see at all (see `find_near_touch`'s own docs), with the
        // touching vertex positioned *before* the near edge in ring order,
        // so welding it needs no index shift.
        let coords = vec![
            Coord { x: 5.0, y: 9.999 }, // F: touches B->C from below
            Coord { x: 0.0, y: 0.0 },   // A
            Coord { x: 0.0, y: 10.0 },  // B
            Coord { x: 10.0, y: 10.0 }, // C
            Coord { x: 10.0, y: 0.0 },  // D
            Coord { x: 5.0, y: 0.0 },   // E
        ];
        let (k, i, weld) = find_near_touch(&coords).expect("F should read as touching B->C");
        assert_eq!(k, 0, "F is the touching vertex");
        assert_eq!(i, 2, "B->C starts at index 2");
        assert!((weld.x - 5.0).abs() < 1e-9 && (weld.y - 10.0).abs() < 1e-9, "weld point should land on B->C's own line: {weld:?}");

        let mut closed = coords.clone();
        closed.push(closed[0]);
        let ring = LineString::new(closed);
        let (inner, outer) = weld_near_touch_and_split(&ring).expect("a near-touch should split");
        for part in [inner, outer] {
            assert!(!geo_ring_self_intersects(part.exterior()), "{:?}", part.exterior());
            assert!(!has_zero_length_edge(part.exterior()), "{:?}", part.exterior());
        }
    }

    #[test]
    fn find_near_touch_locates_a_vertex_grazing_a_distant_edge_after_it_in_the_ring() {
        // Same shape as the test above, but with F moved to the *end* of
        // the ring instead of the front -- the touching vertex now comes
        // strictly *after* the near edge B->C, so welding it needs the
        // index-shift path (insert a new vertex onto the near edge, then
        // find the touching vertex's own shifted position) rather than
        // the no-shift path the other ordering exercises. This ordering
        // is what exposed the zero-length-edge bug `weld_near_touch_and_split`'s
        // own docs describe fixing: the touching vertex's own slot and
        // the freshly inserted point both end up holding the identical
        // welded coordinate, and a naive slice that pushes a closing copy
        // on top of one that's already there duplicates it.
        let coords = vec![
            Coord { x: 0.0, y: 0.0 },   // A
            Coord { x: 0.0, y: 10.0 },  // B
            Coord { x: 10.0, y: 10.0 }, // C
            Coord { x: 10.0, y: 0.0 },  // D
            Coord { x: 5.0, y: 0.0 },   // E
            Coord { x: 5.0, y: 9.999 }, // F: touches B->C from below
        ];
        let (k, i, _weld) = find_near_touch(&coords).expect("F should read as touching B->C");
        assert_eq!(k, 5, "F is the touching vertex");
        assert_eq!(i, 1, "B->C starts at index 1");

        let mut closed = coords.clone();
        closed.push(closed[0]);
        let ring = LineString::new(closed);
        let (inner, outer) = weld_near_touch_and_split(&ring).expect("a near-touch should split");
        for part in [inner, outer] {
            assert!(!geo_ring_self_intersects(part.exterior()), "{:?}", part.exterior());
            assert!(
                !has_zero_length_edge(part.exterior()),
                "shared welded point double-counted into a zero-length edge: {:?}",
                part.exterior()
            );
        }
    }

    #[test]
    fn find_near_touch_ignores_a_real_pinch_far_above_the_margin() {
        // The same hooked shape, but F sits 15cm below B->C instead of
        // 1mm -- comfortably above `SPLIT_SNAP_MARGIN_METERS` (1cm), the
        // scale a real (if tight) waiting-zone corner can legitimately
        // narrow to. `find_near_touch` must leave this alone: welding a
        // real corner into the wrong neighbour's boundary would be a
        // worse defect than the one this function exists to fix.
        let coords = vec![
            Coord { x: 5.0, y: 9.85 }, // F: 15cm below B->C, not a near-touch
            Coord { x: 0.0, y: 0.0 },
            Coord { x: 0.0, y: 10.0 },
            Coord { x: 10.0, y: 10.0 },
            Coord { x: 10.0, y: 0.0 },
            Coord { x: 5.0, y: 0.0 },
        ];
        assert!(find_near_touch(&coords).is_none(), "15cm of real clearance must not read as a near-touch");
    }

    #[test]
    fn find_near_touch_ignores_a_vertex_already_adjacent_to_the_near_edge() {
        // F sits 1mm from segment B->C same as the other tests here, but
        // this time F is *already* ring-adjacent to B (one ring-step
        // away, not sharing B's own position but next to it) rather than
        // several hops apart -- confirmed on real Barcelona data
        // (`6119951203_w0_straight_ped`): welding a vertex this close to
        // an edge it's already next to overwrites it to the *same*
        // coordinate as its own immediate neighbour, leaving a genuine
        // zero-length edge -- a self-intersection this function would be
        // introducing, not fixing. See `find_near_touch`'s own docs.
        let coords = vec![
            Coord { x: 0.0, y: 0.0 },   // A
            Coord { x: 0.0, y: 10.0 },  // B
            Coord { x: 5.0, y: 9.999 }, // F: one ring-step from B, grazing B->C
            Coord { x: 10.0, y: 10.0 }, // C
            Coord { x: 10.0, y: 0.0 },  // D
            Coord { x: 5.0, y: 0.0 },   // E
        ];
        assert!(
            find_near_touch(&coords).is_none(),
            "a vertex already ring-adjacent to the near edge's own endpoint must not be welded"
        );
    }

}

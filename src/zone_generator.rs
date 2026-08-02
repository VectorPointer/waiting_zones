//! Generates vehicle [`WaitingZone`]s at traffic-light-controlled junctions.
//!
//! Zones are grouped by **signal head**, not just by junction: two lanes
//! only belong to the same zone if they always share the exact same
//! character, in every phase of the junction's `tlLogic` program (i.e.
//! they're always red/green/yellow together). A junction with independent
//! signals for "straight" and "right turn" therefore gets two zones, not
//! one.
//!
//! Each zone spans the full length of its underlying lane by default: the
//! entry boundary sits at the lane's start and the exit boundary at its
//! end, so a vehicle counts as "waiting" for as long as it's on that lane.
//! `max_zone_length` caps that: the exit stays anchored at the stop line,
//! but the entry moves to `length - max_zone_length` (clamped to the
//! lane's start) instead of the lane's start outright.
//!
//! A junction is only grouped this way if its `tlLogic` program can be
//! found (by matching `TrafficLightProgram::id` to the junction's id, the
//! SUMO convention for junction-level traffic lights); if it can't
//! (`JunctionKind::TrafficLightUnregulated`, or an incomplete `.net.xml`),
//! we can't group correctly, so a warning is printed and that junction is
//! skipped entirely rather than guessing.
//!
//! Pedestrian waiting zones (at signalized crossings) aren't generated —
//! see the README's "Status" section for why.

use crate::domain::{
    Connection, EdgeId, Junction, JunctionKind, LaneId, LaneIndex, Network, TrafficLightId,
    TrafficLightProgram, WaitingZone, WaitingZoneId, ZoneBoundary,
};
use std::collections::HashMap;
use uom::si::f64::Length;
use uom::si::length::meter;

/// Junction kinds that control right-of-way with a traffic light, i.e. the
/// ones where vehicles queue up waiting for a green phase. Rail-specific
/// kinds ([`JunctionKind::RailSignal`]) are deliberately excluded: they
/// don't carry road traffic.
fn is_traffic_light(kind: JunctionKind) -> bool {
    matches!(
        kind,
        JunctionKind::TrafficLight
            | JunctionKind::TrafficLightUnregulated
            | JunctionKind::TrafficLightRightOnRed
    )
}

/// A lane's length together with whether it's restricted to pedestrians
/// only (`allow="pedestrian"` in `.net.xml`, e.g. crossings and
/// walkingareas) — used to exclude sidewalk/walkingarea lanes that SUMO
/// lists among a junction's `incLanes` alongside the real vehicle lanes,
/// straight from SUMO's own vClass permissions instead of the edge's
/// `function` label.
struct LaneInfo {
    length: Length,
    pedestrian_only: bool,
}

fn is_pedestrian_only(allow: &[String]) -> bool {
    !allow.is_empty() && allow.iter().all(|vclass| vclass == "pedestrian")
}

/// The sequence of `state` characters, one per phase (in program order), at
/// a single `linkIndex`. Two links with an equal `SignalKey` are always
/// red/green/yellow together — i.e. controlled by the same signal head.
type SignalKey = Vec<char>;

/// `None` if `link_index` is out of range for any phase (malformed input):
/// safer to fail the grouping for that link than to silently compare a
/// truncated key.
fn signal_key_for_link(program: &TrafficLightProgram, link_index: crate::domain::LinkIndex) -> Option<SignalKey> {
    let index = usize::try_from(link_index.0).ok()?;
    program.phases.iter().map(|state| state.chars().nth(index)).collect()
}

/// Generates vehicle [`WaitingZone`]s for every traffic-light junction in
/// `network` whose `tlLogic` program can be found. Junctions without one
/// are skipped, with a warning printed to stderr (see the module docs).
///
/// `max_zone_length` caps how far each zone's entry extends from the stop
/// line; `None` means the full lane, as before.
pub fn generate(network: &Network, max_zone_length: Option<Length>) -> Vec<WaitingZone> {
    let lanes: HashMap<&LaneId, LaneInfo> = network
        .edges
        .iter()
        .flat_map(|edge| &edge.lanes)
        .map(|lane| {
            (
                &lane.id,
                LaneInfo {
                    length: lane.length,
                    pedestrian_only: is_pedestrian_only(&lane.allow),
                },
            )
        })
        .collect();

    // Needed to resolve a `Connection`'s `(edge, lane index)` pair (how
    // lanes reference each other) into the actual `LaneId` the rest of the
    // domain model — and `lanes` above — keys off.
    let lane_ids_by_edge_and_index: HashMap<(&EdgeId, LaneIndex), &LaneId> = network
        .edges
        .iter()
        .flat_map(|edge| edge.lanes.iter().map(move |lane| ((&edge.id, lane.index), &lane.id)))
        .collect();

    // Every connection originating from a given lane, indexed once up
    // front. Without this, grouping lanes by signal (see
    // `group_key_for_lane`) would re-scan every connection in the network
    // for every lane — quadratic in network size, and prohibitively slow
    // on a real city-scale network (multiple minutes on ~10k junctions).
    let mut connections_by_from_lane: HashMap<&LaneId, Vec<&Connection>> = HashMap::new();
    for connection in &network.connections {
        if let Some(&lane_id) = lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane)) {
            connections_by_from_lane.entry(lane_id).or_default().push(connection);
        }
    }

    // A `tlLogic` id can repeat (multiple programs); keep the first one
    // encountered, in file order.
    let mut programs_by_id: HashMap<&TrafficLightId, &TrafficLightProgram> = HashMap::new();
    for program in &network.traffic_light_programs {
        programs_by_id.entry(&program.id).or_insert(program);
    }

    network
        .junctions
        .iter()
        .filter(|junction| is_traffic_light(junction.kind))
        .flat_map(|junction| {
            let tl_id = TrafficLightId(junction.id.0.clone());
            let Some(program) = programs_by_id.get(&tl_id) else {
                eprintln!(
                    "warning: no tlLogic program found for traffic light \"{tl_id}\" — skipping waiting zones for junction \"{}\"",
                    junction.id
                );
                return Vec::new();
            };

            vehicle_zones(junction, &lanes, &connections_by_from_lane, program, max_zone_length)
        })
        .collect()
}

/// Builds entry/exit boundary pairs for each lane in `lane_ids` that's
/// known to `lanes`. The exit always sits at the lane's end (the stop
/// line); the entry sits at the lane's start, unless `max_zone_length`
/// caps it closer to the exit (clamped so it never goes past it).
fn full_lane_boundaries(
    lane_ids: &[LaneId],
    lanes: &HashMap<&LaneId, LaneInfo>,
    max_zone_length: Option<Length>,
) -> (Vec<ZoneBoundary>, Vec<ZoneBoundary>) {
    lane_ids
        .iter()
        .filter_map(|lane_id| {
            let length = lanes.get(lane_id)?.length;
            let entry_position = match max_zone_length {
                Some(max) if max < length => length - max,
                _ => Length::new::<meter>(0.0),
            };
            let entry = ZoneBoundary {
                lane: lane_id.clone(),
                position: entry_position,
            };
            let exit = ZoneBoundary {
                lane: lane_id.clone(),
                position: length,
            };
            Some((entry, exit))
        })
        .unzip()
}

/// The combined [`SignalKey`]s (sorted, deduplicated) of every
/// signal-controlled connection originating from `lane_id`. A lane can have
/// more than one (e.g. a shared straight+right lane), in which case it
/// forms its own group distinct from lanes with a single movement — it
/// never changes state independently of either.
///
/// `None` if `lane_id` has no signal-controlled connection at all — the
/// caller should skip the lane (and warn) rather than lump it in.
fn group_key_for_lane(
    lane_id: &LaneId,
    connections_by_from_lane: &HashMap<&LaneId, Vec<&Connection>>,
    program: &TrafficLightProgram,
) -> Option<Vec<SignalKey>> {
    let mut keys: Vec<SignalKey> = connections_by_from_lane
        .get(lane_id)?
        .iter()
        .filter_map(|connection| signal_key_for_link(program, connection.link_index?))
        .collect();

    if keys.is_empty() {
        return None;
    }

    keys.sort();
    keys.dedup();
    Some(keys)
}

/// The vehicle waiting zones queued on `junction`'s incoming lanes, one per
/// distinct signal group.
fn vehicle_zones(
    junction: &Junction,
    lanes: &HashMap<&LaneId, LaneInfo>,
    connections_by_from_lane: &HashMap<&LaneId, Vec<&Connection>>,
    program: &TrafficLightProgram,
    max_zone_length: Option<Length>,
) -> Vec<WaitingZone> {
    let mut groups: HashMap<Vec<SignalKey>, Vec<LaneId>> = HashMap::new();

    for lane_id in &junction.incoming_lanes {
        if lanes.get(lane_id).is_some_and(|info| info.pedestrian_only) {
            continue; // sidewalk/walkingarea lane feeding into the junction, not a vehicle lane
        }

        match group_key_for_lane(lane_id, connections_by_from_lane, program) {
            Some(key) => groups.entry(key).or_default().push(lane_id.clone()),
            None => eprintln!(
                "warning: lane \"{lane_id}\" at traffic light \"{}\" has no signal-controlled connection — skipping it",
                junction.id
            ),
        }
    }

    let mut groups: Vec<_> = groups.into_iter().collect();
    groups.sort_by(|(a, _), (b, _)| a.cmp(b));

    groups
        .into_iter()
        .enumerate()
        .filter_map(|(index, (_, lane_ids))| {
            let (entries, exits) = full_lane_boundaries(&lane_ids, lanes, max_zone_length);
            if entries.is_empty() {
                return None;
            }

            Some(WaitingZone {
                id: WaitingZoneId(format!("{}_{index}", junction.id.0)),
                entries,
                exits,
                icon_position: junction.position,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::domain::{
        ConnectionDirection, Edge, EdgeFunction, EdgeId, Junction, JunctionId, JunctionKind, Lane,
        LaneId, LaneIndex, LinkIndex, LinkState, Point, TrafficLightId,
    };
    use uom::si::f64::Velocity;
    use uom::si::velocity::meter_per_second;

    fn lane(id: &str, length_m: f64) -> Lane {
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: Velocity::new::<meter_per_second>(10.0),
            length: Length::new::<meter>(length_m),
            width: Length::new::<meter>(3.2),
            end_offset: Length::new::<meter>(0.0),
            shape: Default::default(),
            allow: vec![],
            disallow: vec![],
        }
    }

    fn indexed_lane(id: &str, index: usize, length_m: f64) -> Lane {
        Lane {
            index: LaneIndex(index),
            ..lane(id, length_m)
        }
    }

    fn pedestrian_lane(id: &str, length_m: f64) -> Lane {
        Lane {
            allow: vec!["pedestrian".into()],
            ..lane(id, length_m)
        }
    }

    fn edge(id: &str, lanes: Vec<Lane>) -> Edge {
        edge_with_function(id, EdgeFunction::Normal, lanes)
    }

    fn edge_with_function(id: &str, function: EdgeFunction, lanes: Vec<Lane>) -> Edge {
        Edge {
            id: EdgeId(id.into()),
            function,
            from: None,
            to: None,
            name: None,
            priority: None,
            length: None,
            shape: None,
            spread_type: None,
            lanes,
        }
    }

    fn junction(id: &str, kind: JunctionKind, incoming_lanes: Vec<&str>) -> Junction {
        Junction {
            id: JunctionId(id.into()),
            position: Point { x: 0.0, y: 0.0, z: 0.0 },
            kind,
            incoming_lanes: incoming_lanes.into_iter().map(|l| LaneId(l.into())).collect(),
            internal_lanes: vec![],
            shape: None,
            name: None,
        }
    }

    fn program(id: &str, phases: Vec<&str>) -> TrafficLightProgram {
        TrafficLightProgram {
            id: TrafficLightId(id.into()),
            phases: phases.into_iter().map(String::from).collect(),
        }
    }

    fn vehicle_connection(
        from_edge: &str,
        from_lane: usize,
        tl: &str,
        link_index: i32,
    ) -> Connection {
        Connection {
            from_edge: EdgeId(from_edge.into()),
            to_edge: EdgeId("out".into()),
            from_lane: LaneIndex(from_lane),
            to_lane: LaneIndex(0),
            direction: ConnectionDirection::Straight,
            state: LinkState::Major,
            via: None,
            traffic_light: Some(TrafficLightId(tl.into())),
            link_index: Some(LinkIndex(link_index)),
            pass: false,
            keep_clear: true,
        }
    }

    #[test]
    fn groups_lanes_with_identical_state_across_all_phases_into_one_zone() {
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 40.0)]),
            ],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0", "e1_0"])],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 0, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "rr"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(zones.len(), 1, "link 0 and link 1 always share state -> one zone");
        assert_eq!(zones[0].entries.len(), 2);
    }

    #[test]
    fn splits_lanes_with_different_state_in_any_phase_into_separate_zones() {
        // link 0 (straight) and link 1 (protected right turn) diverge in the
        // second phase: 'G' vs 'r'.
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 40.0)]),
            ],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0", "e1_0"])],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 0, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "Gr"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(zones.len(), 2, "link 0 and link 1 diverge in phase 2 -> two zones");
        assert_eq!(zones[0].id, WaitingZoneId("j0_0".into()));
        assert_eq!(zones[1].id, WaitingZoneId("j0_1".into()));
    }

    #[test]
    fn skips_junction_and_warns_when_no_matching_tl_logic_program() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLightUnregulated, vec!["e0_0"])],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![], // no program at all
            ..Default::default()
        };

        assert!(generate(&network, None).is_empty());
    }

    #[test]
    fn skips_lane_and_warns_when_it_has_no_signal_controlled_connection() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0"])],
            connections: vec![], // no connection for e0_0 at all
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        assert!(generate(&network, None).is_empty());
    }

    #[test]
    fn excludes_pedestrian_only_lanes_from_vehicle_zones() {
        // Mirrors a real SUMO quirk: a traffic-light junction's `incLanes`
        // can include a walkingarea lane alongside the vehicle lanes.
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 2.0)],
                ),
            ],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", ":j0_w0_0"],
            )],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(zones.len(), 1, "the walkingarea lane isn't a vehicle lane");
        assert_eq!(zones[0].entries.len(), 1);
        assert_eq!(zones[0].entries[0].lane, LaneId("e0_0".into()));
    }

    #[test]
    fn max_zone_length_caps_the_entry_but_keeps_the_exit_at_the_stop_line() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 100.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0"])],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, Some(Length::new::<meter>(20.0)));

        assert_eq!(zones.len(), 1);
        assert_eq!(zones[0].entries[0].position, Length::new::<meter>(80.0));
        assert_eq!(zones[0].exits[0].position, Length::new::<meter>(100.0));
    }

    #[test]
    fn max_zone_length_longer_than_the_lane_has_no_effect() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0"])],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, Some(Length::new::<meter>(1000.0)));

        assert_eq!(zones[0].entries[0].position, Length::new::<meter>(0.0));
    }

    #[test]
    fn zones_at_the_same_junction_share_the_junction_position_as_icon_position() {
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 1, 40.0)]),
            ],
            junctions: vec![Junction {
                id: JunctionId("j0".into()),
                position: Point { x: 42.0, y: 7.0, z: 0.0 },
                kind: JunctionKind::TrafficLight,
                incoming_lanes: vec![LaneId("e0_0".into()), LaneId("e1_0".into())],
                internal_lanes: vec![],
                shape: None,
                name: None,
            }],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 1, "j0", 1),
            ],
            // link 0 and link 1 diverge in phase 2 -> two separate zones
            traffic_light_programs: vec![program("j0", vec!["GG", "Gr"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(zones.len(), 2, "two signal groups at the same junction");
        for zone in &zones {
            assert_eq!(
                zone.icon_position,
                Point { x: 42.0, y: 7.0, z: 0.0 },
                "zone {:?} should be anchored at the junction's own position",
                zone.id
            );
        }
    }
}

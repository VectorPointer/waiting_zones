//! Generates [`WaitingZone`]s at traffic-light-controlled junctions, for
//! both vehicles (queued on incoming lanes) and pedestrians (queued on the
//! sidewalk before a signalized crossing).
//!
//! Zones are grouped by **signal head**, not just by junction: two lanes
//! only belong to the same zone if they always share the exact same
//! character, in every phase of the junction's `tlLogic` program (i.e.
//! they're always red/green/yellow together). A junction with independent
//! signals for "straight" and "right turn" therefore gets two vehicle
//! zones, not one; two crossings with independent pedestrian signals get
//! two pedestrian zones even if they're right next to each other.
//!
//! Both kinds of zone span the full length of their underlying lane by
//! default: the entry boundary sits at the lane's start and the exit
//! boundary at its end, so the corresponding road user counts as "waiting"
//! for as long as they're on that lane. `max_zone_length` caps that: the
//! exit stays anchored at the stop line / crossing, but the entry moves to
//! `length - max_zone_length` (clamped to the lane's start) instead of the
//! lane's start outright. Crucially, the pedestrian zone's lane is the
//! *walkingarea before* the crossing, not the crossing itself — a
//! pedestrian on the crossing is actively walking across, not waiting; the
//! waiting happens on the sidewalk, before the signal lets them step onto
//! it.
//!
//! A junction is only grouped this way if its `tlLogic` program can be
//! found (by matching `TrafficLightProgram::id` to the junction's id, the
//! SUMO convention for junction-level traffic lights); if it can't
//! (`JunctionKind::TrafficLightUnregulated`, or an incomplete `.net.xml`),
//! we can't group correctly, so a warning is printed and that junction is
//! skipped entirely rather than guessing.

use crate::domain::{
    Connection, EdgeId, Junction, JunctionKind, LaneId, LaneIndex, Network, RoadUser,
    TrafficLightId, TrafficLightProgram, WaitingZone, WaitingZoneId, ZoneBoundary,
};
use std::collections::{HashMap, HashSet};
use uom::si::f64::Length;
use uom::si::length::meter;

/// Junction kinds that control right-of-way with a traffic light, i.e. the
/// ones where vehicles and pedestrians queue up waiting for a green phase.
/// Rail-specific kinds ([`JunctionKind::RailSignal`]) are deliberately
/// excluded: they don't carry road traffic.
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
/// walkingareas) — the signal used to tell vehicle lanes and pedestrian
/// lanes apart, straight from SUMO's own vClass permissions instead of the
/// edge's `function` label.
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

/// Generates vehicle and pedestrian [`WaitingZone`]s for every traffic-light
/// junction in `network` whose `tlLogic` program can be found. Junctions
/// without one are skipped, with a warning printed to stderr (see the
/// module docs).
///
/// `max_zone_length` caps how far each zone's entry extends from the stop
/// line / crossing; `None` means the full lane, as before.
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

            let mut zones = vehicle_zones(
                junction,
                &lanes,
                &network.connections,
                &lane_ids_by_edge_and_index,
                program,
                max_zone_length,
            );
            zones.extend(pedestrian_zones(
                junction,
                &lanes,
                &network.connections,
                &lane_ids_by_edge_and_index,
                program,
                max_zone_length,
            ));
            zones
        })
        .collect()
}

/// Builds entry/exit boundary pairs for each lane in `lane_ids` that's
/// known to `lanes`. The exit always sits at the lane's end (the stop line
/// / crossing); the entry sits at the lane's start, unless `max_zone_length`
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
    connections: &[Connection],
    lane_ids_by_edge_and_index: &HashMap<(&EdgeId, LaneIndex), &LaneId>,
    program: &TrafficLightProgram,
) -> Option<Vec<SignalKey>> {
    let mut keys: Vec<SignalKey> = connections
        .iter()
        .filter(|connection| {
            lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane)) == Some(&lane_id)
        })
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
    connections: &[Connection],
    lane_ids_by_edge_and_index: &HashMap<(&EdgeId, LaneIndex), &LaneId>,
    program: &TrafficLightProgram,
    max_zone_length: Option<Length>,
) -> Vec<WaitingZone> {
    let mut groups: HashMap<Vec<SignalKey>, Vec<LaneId>> = HashMap::new();

    for lane_id in &junction.incoming_lanes {
        if lanes.get(lane_id).is_some_and(|info| info.pedestrian_only) {
            continue; // sidewalk/walkingarea lane feeding into the junction, not a vehicle lane
        }

        match group_key_for_lane(lane_id, connections, lane_ids_by_edge_and_index, program) {
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
                road_user: RoadUser::Vehicle,
                icon_position: junction.position,
            })
        })
        .collect()
}

/// The pedestrian waiting zones approaching `junction`'s signalized
/// crossings ("pasos de peatones"), one per distinct signal group — on the
/// walkingarea lane(s) that lead into each crossing.
///
/// A crossing lane itself only appears as the `to` of a connection with
/// `tl`/`link_index` set (the pedestrian signal actually gates *entering*
/// the crossing; leaving it is unconstrained) — that connection's `from` is
/// the approaching walkingarea, which is what we're after, and its
/// `link_index` (resolved through `program`) is what determines the group.
fn pedestrian_zones(
    junction: &Junction,
    lanes: &HashMap<&LaneId, LaneInfo>,
    connections: &[Connection],
    lane_ids_by_edge_and_index: &HashMap<(&EdgeId, LaneIndex), &LaneId>,
    program: &TrafficLightProgram,
    max_zone_length: Option<Length>,
) -> Vec<WaitingZone> {
    let crossing_lanes: HashSet<&LaneId> = junction
        .internal_lanes
        .iter()
        .filter(|lane_id| lanes.get(lane_id).is_some_and(|info| info.pedestrian_only))
        .collect();

    let mut groups: HashMap<SignalKey, Vec<LaneId>> = HashMap::new();

    for connection in connections {
        let Some(to_lane) = lane_ids_by_edge_and_index.get(&(&connection.to_edge, connection.to_lane)) else {
            continue;
        };
        if !crossing_lanes.contains(*to_lane) {
            continue;
        }
        let Some(from_lane) = lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane))
        else {
            continue;
        };
        let Some(link_index) = connection.link_index else {
            continue; // not the tl-controlled "enter the crossing" connection
        };
        let Some(key) = signal_key_for_link(program, link_index) else {
            eprintln!(
                "warning: link index {link_index} out of range for traffic light \"{}\" — skipping crossing lane \"{from_lane}\"",
                junction.id
            );
            continue;
        };

        groups.entry(key).or_default().push((*from_lane).clone());
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
                id: WaitingZoneId(format!("{}_ped_{index}", junction.id.0)),
                entries,
                exits,
                road_user: RoadUser::Pedestrian,
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
        junction_with_internal_lanes(id, kind, incoming_lanes, vec![])
    }

    fn junction_with_internal_lanes(
        id: &str,
        kind: JunctionKind,
        incoming_lanes: Vec<&str>,
        internal_lanes: Vec<&str>,
    ) -> Junction {
        Junction {
            id: JunctionId(id.into()),
            position: Point { x: 0.0, y: 0.0, z: 0.0 },
            kind,
            incoming_lanes: incoming_lanes.into_iter().map(|l| LaneId(l.into())).collect(),
            internal_lanes: internal_lanes.into_iter().map(|l| LaneId(l.into())).collect(),
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

    fn pedestrian_connection(from_edge: &str, to_edge: &str, tl: &str, link_index: i32) -> Connection {
        Connection {
            from_edge: EdgeId(from_edge.into()),
            to_edge: EdgeId(to_edge.into()),
            from_lane: LaneIndex(0),
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

        assert_eq!(zones.len(), 1, "no pedestrian zone: no crossing in internal_lanes");
        assert_eq!(zones[0].entries.len(), 1);
        assert_eq!(zones[0].entries[0].lane, LaneId("e0_0".into()));
    }

    #[test]
    fn generates_a_pedestrian_zone_on_the_walkingarea_before_the_crossing() {
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge_with_function(
                    ":j0_c0",
                    EdgeFunction::Crossing,
                    vec![pedestrian_lane(":j0_c0_0", 6.4)],
                ),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 3.0)],
                ),
            ],
            junctions: vec![junction_with_internal_lanes(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0"],
                vec![":j0_c0_0"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                pedestrian_connection(":j0_w0", ":j0_c0", "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["Gg", "rG"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(zones.len(), 2, "one vehicle zone and one pedestrian zone");

        let ped_zone = zones
            .iter()
            .find(|z| z.id == WaitingZoneId("j0_ped_0".into()))
            .expect("pedestrian zone should be generated");
        assert_eq!(ped_zone.road_user, RoadUser::Pedestrian);
        assert_eq!(
            ped_zone.entries,
            vec![ZoneBoundary {
                lane: LaneId(":j0_w0_0".into()),
                position: Length::new::<meter>(0.0),
            }],
            "entry is on the walkingarea approaching the crossing, not the crossing itself"
        );
        assert_eq!(
            ped_zone.exits,
            vec![ZoneBoundary {
                lane: LaneId(":j0_w0_0".into()),
                position: Length::new::<meter>(3.0),
            }]
        );

        let vehicle_zone = zones
            .iter()
            .find(|z| z.id == WaitingZoneId("j0_0".into()))
            .expect("vehicle zone should still be generated");
        assert_eq!(vehicle_zone.road_user, RoadUser::Vehicle);
    }

    #[test]
    fn splits_two_close_crossings_with_independent_signals_into_separate_zones() {
        let network = Network {
            edges: vec![
                edge_with_function(
                    ":j0_c0",
                    EdgeFunction::Crossing,
                    vec![pedestrian_lane(":j0_c0_0", 6.4)],
                ),
                edge_with_function(
                    ":j0_c1",
                    EdgeFunction::Crossing,
                    vec![pedestrian_lane(":j0_c1_0", 6.4)],
                ),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 3.0)],
                ),
                edge_with_function(
                    ":j0_w1",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w1_0", 3.0)],
                ),
            ],
            junctions: vec![junction_with_internal_lanes(
                "j0",
                JunctionKind::TrafficLight,
                vec![],
                vec![":j0_c0_0", ":j0_c1_0"],
            )],
            connections: vec![
                pedestrian_connection(":j0_w0", ":j0_c0", "j0", 0),
                pedestrian_connection(":j0_w1", ":j0_c1", "j0", 1),
            ],
            // link 0 and link 1 never share the same state -> independent signals
            traffic_light_programs: vec![program("j0", vec!["Gr", "rG"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(zones.len(), 2);
        assert_eq!(zones[0].id, WaitingZoneId("j0_ped_0".into()));
        assert_eq!(zones[1].id, WaitingZoneId("j0_ped_1".into()));
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
                edge_with_function(
                    ":j0_c0",
                    EdgeFunction::Crossing,
                    vec![pedestrian_lane(":j0_c0_0", 6.4)],
                ),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 3.0)],
                ),
            ],
            junctions: vec![Junction {
                id: JunctionId("j0".into()),
                position: Point { x: 42.0, y: 7.0, z: 0.0 },
                kind: JunctionKind::TrafficLight,
                incoming_lanes: vec![LaneId("e0_0".into())],
                internal_lanes: vec![LaneId(":j0_c0_0".into())],
                shape: None,
                name: None,
            }],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                pedestrian_connection(":j0_w0", ":j0_c0", "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["Gg", "rG"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(zones.len(), 2, "one vehicle zone and one pedestrian zone");
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

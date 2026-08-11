//! Generates vehicle waiting zones — one
//! [`E3Detector`](sumo_types::additional::domain::E3Detector) per zone — at
//! traffic-light-controlled junctions.
//!
//! There's no project-specific "waiting zone" type: `sumo_types` already
//! models exactly what one is (an area delimited by entry/exit gates), so
//! this module builds its `E3Detector` values directly rather than
//! maintaining a parallel type that would only ever get converted into one.
//! `id`/`entries`/`exits`/`icon_position` are set here; `file` (SUMO
//! requires the attribute, but the destination path is
//! [`crate::zone_output`]'s concern, not this module's) is left empty and
//! patched in before writing — see that module's own docs.
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

use std::collections::HashMap;
use sumo_types::additional::domain::{DetectorGate, DetectorId, E3Detector, LanePosition, LaneRef};
use sumo_types::domain::{
    Connection, EdgeId, Junction, JunctionKind, LaneId, LaneIndex, LinkIndex, Network,
    TrafficLightId, TrafficLightProgram,
};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

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
fn signal_key_for_link(program: &TrafficLightProgram, link_index: LinkIndex) -> Option<SignalKey> {
    let index = usize::try_from(link_index.0).ok()?;
    program
        .phases
        .iter()
        .map(|phase| phase.state.chars().nth(index))
        .collect()
}

/// Generates vehicle waiting zones for every traffic-light junction in
/// `network` whose `tlLogic` program can be found. Junctions without one
/// are skipped, with a warning printed to stderr (see the module docs).
///
/// `max_zone_length` caps how far each zone's entry extends from the stop
/// line; `None` means the full lane, as before.
pub fn generate(network: &Network, max_zone_length: Option<Length>) -> Vec<E3Detector> {
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
        .flat_map(|edge| {
            edge.lanes
                .iter()
                .map(move |lane| ((&edge.id, lane.index), &lane.id))
        })
        .collect();

    // Every connection originating from a given lane, indexed once up
    // front. Without this, grouping lanes by signal (see
    // `group_key_for_lane`) would re-scan every connection in the network
    // for every lane — quadratic in network size, and prohibitively slow
    // on a real city-scale network (multiple minutes on ~10k junctions).
    let mut connections_by_from_lane: HashMap<&LaneId, Vec<&Connection>> = HashMap::new();
    for connection in &network.connections {
        if let Some(&lane_id) =
            lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane))
        {
            connections_by_from_lane
                .entry(lane_id)
                .or_default()
                .push(connection);
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

/// Builds entry/exit detector gates for each lane in `lane_ids` that's known
/// to `lanes`. The exit always sits at the lane's end (the stop line); the
/// entry sits at the lane's start, unless `max_zone_length` caps it closer
/// to the exit (clamped so it never goes past it).
///
/// Both are always [`LanePosition::FromStart`]: a waiting zone's boundaries
/// are computed from the lane's own length, so there's never a reason to
/// express one as `FromEnd` instead.
///
/// Both also set `friendlyPos`: the exit sits at exactly `length`, the
/// `.net.xml`'s own reported lane length, but netedit computes a lane's
/// *geometric* length from its shape, which can differ from that attribute
/// by the last handful of floating-point digits. Without `friendlyPos`,
/// netedit rejects a `pos` fractionally beyond what it computes as "Invalid
/// position over lane" — real Barcelona data hits this. `friendlyPos`
/// clamps a mismatch like that into range instead of erroring, which is
/// exactly what it exists for (see `DetectorGate::friendly_position`'s own
/// docs) and costs nothing when the two lengths already agree.
fn full_lane_boundaries(
    lane_ids: &[LaneId],
    lanes: &HashMap<&LaneId, LaneInfo>,
    max_zone_length: Option<Length>,
) -> (Vec<DetectorGate>, Vec<DetectorGate>) {
    lane_ids
        .iter()
        .filter_map(|lane_id| {
            let length = lanes.get(lane_id)?.length;
            let entry_position = match max_zone_length {
                Some(max) if max < length => length - max,
                _ => Length::new::<meter>(0.0),
            };
            let lane = LaneRef(lane_id.0.clone());
            let entry = DetectorGate {
                lane: lane.clone(),
                position: LanePosition::FromStart(entry_position),
                friendly_position: Some(true),
            };
            let exit = DetectorGate {
                lane,
                position: LanePosition::FromStart(length),
                friendly_position: Some(true),
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
) -> Vec<E3Detector> {
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

            Some(E3Detector {
                id: DetectorId(format!("{}_{index}", junction.id.0)),
                entries,
                exits,
                // Not this module's concern — see the module docs.
                // `zone_output::write` fills this in before serializing.
                file: String::new(),
                icon_position: Some(junction.position),
                period: None,
                name: None,
                speed_threshold: None,
                time_threshold: None,
                open_entry: None,
            })
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use sumo_types::domain::{
        ConnectionDirection, Edge, EdgeFunction, EdgeId, Junction, JunctionId, JunctionKind, Lane,
        LaneId, LaneIndex, LinkIndex, LinkState, Phase, Point, TrafficLightId, TrafficLightKind,
    };
    use sumo_types::uom::si::f64::{Time, Velocity};
    use sumo_types::uom::si::time::second;
    use sumo_types::uom::si::velocity::meter_per_second;

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
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            kind,
            incoming_lanes: incoming_lanes
                .into_iter()
                .map(|l| LaneId(l.into()))
                .collect(),
            internal_lanes: vec![],
            shape: None,
            name: None,
        }
    }

    /// Only `state` matters to the grouping under test, so every phase gets
    /// the same placeholder duration; `program_id` is the id SUMO gives a
    /// junction's first program.
    fn program(id: &str, phases: Vec<&str>) -> TrafficLightProgram {
        TrafficLightProgram {
            id: TrafficLightId(id.into()),
            program_id: "0".into(),
            kind: Some(TrafficLightKind::Static),
            offset: None,
            phases: phases
                .into_iter()
                .map(|state| Phase {
                    duration: Time::new::<second>(30.0),
                    state: state.into(),
                })
                .collect(),
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
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", "e1_0"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 0, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "rr"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(
            zones.len(),
            1,
            "link 0 and link 1 always share state -> one zone"
        );
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
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", "e1_0"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 0, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "Gr"])],
            ..Default::default()
        };

        let zones = generate(&network, None);

        assert_eq!(
            zones.len(),
            2,
            "link 0 and link 1 diverge in phase 2 -> two zones"
        );
        assert_eq!(zones[0].id, DetectorId("j0_0".into()));
        assert_eq!(zones[1].id, DetectorId("j0_1".into()));
    }

    #[test]
    fn skips_junction_and_warns_when_no_matching_tl_logic_program() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLightUnregulated,
                vec!["e0_0"],
            )],
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
        assert_eq!(zones[0].entries[0].lane, LaneRef("e0_0".into()));
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
        assert_eq!(
            zones[0].entries[0].position,
            LanePosition::FromStart(Length::new::<meter>(80.0))
        );
        assert_eq!(
            zones[0].exits[0].position,
            LanePosition::FromStart(Length::new::<meter>(100.0))
        );
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

        assert_eq!(
            zones[0].entries[0].position,
            LanePosition::FromStart(Length::new::<meter>(0.0))
        );
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
                position: Point {
                    x: 42.0,
                    y: 7.0,
                    z: 0.0,
                },
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
                Some(Point {
                    x: 42.0,
                    y: 7.0,
                    z: 0.0
                }),
                "zone {:?} should be anchored at the junction's own position",
                zone.id
            );
        }
    }
}

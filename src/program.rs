//! A junction's signal program ("the cycle"): which movements and
//! pedestrian crossings get green together, in which order, and for how
//! long — plus which waiting zones each phase serves.
//!
//! The layout is our own rule, informed by how SUMO builds a default
//! program:
//! 1. Approaches facing each other are paired; each pair gets a phase with
//!    its straight and right movements green (`G`). A left turn joins as
//!    permissive (`g`, yielding) when the only traffic it crosses is the
//!    opposite approach's.
//! 2. An approach with nobody opposite gets a phase of its own.
//! 3. A pair whose left turn has its own lane gets a protected-left phase.
//! 4. Any movement still never green goes into the first phase it doesn't
//!    conflict with, or a new one.
//! 5. A crosswalk is green in a phase when nothing crossing it goes
//!    straight (turns crossing it yield: `g`); one that's never green gets
//!    a pedestrian phase.
//! 6. Each green phase is followed by an amber one.

use std::collections::{BTreeSet, HashMap};

use crate::clusters::Cluster;
use crate::geometry::{self, Pt};
use crate::graph::Graph;
use crate::movements::{Junction, heading_at_end, heading_at_start, turn_degrees};
use crate::network::{Direction, Mode};
use crate::osm::NodeId;

/// Two approaches whose headings differ by more than this face each other.
const OPPOSITE_DEGREES: f64 = 145.0;
/// The heading window (degrees) within which two straight movements count as
/// crossing each other, whatever the approximate paths say.
const MIN_PERPENDICULAR_DEGREES: f64 = 60.0;
const MAX_PERPENDICULAR_DEGREES: f64 = 120.0;
/// How far past the junction a movement's path is followed, so a crosswalk
/// on the exit (set back from the junction) is seen as crossed.
const EXIT_TAIL_METERS: f64 = 15.0;
const MAIN_GREEN_SECS: f64 = 30.0;
const MINOR_GREEN_SECS: f64 = 20.0;
const PROTECTED_LEFT_SECS: f64 = 10.0;
const EXTRA_GREEN_SECS: f64 = 15.0;
const PEDESTRIAN_GREEN_SECS: f64 = 15.0;
pub const MIN_GREEN_SECS: f64 = 5.0;
pub const MAX_GREEN_SECS: f64 = 60.0;
/// An approach faster than this gets a longer amber.
const FAST_SPEED_KMH: f64 = 50.0;
const FAST_AMBER_SECS: f64 = 4.0;
const AMBER_SECS: f64 = 3.0;
/// The heading a movement is taken to have when its edge gives none.
const DEFAULT_HEADING: Pt = [1.0, 0.0];
/// How far back along an edge its bearing at the junction is measured.
const BEARING_BASELINE_METERS: f64 = 10.0;
/// Furthest the control point bridging two joined pieces may reach.
const BEND_CONTROL_MAX_METERS: f64 = 60.0;
/// Segments a bridged bend's bezier is drawn with.
const BEZIER_SEGMENTS: usize = 8;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Signal {
    Green,
    Permissive,
    Red,
}

impl Signal {
    fn is_green(self) -> bool {
        self != Signal::Red
    }
}

/// A signalized pedestrian crossing this junction controls.
pub struct Crossing {
    pub zone_id: String,
    /// The node where it crosses the road.
    pub anchor: NodeId,
    /// The crosswalk polyline, local metres.
    pub line: Vec<Pt>,
}

/// Which waiting zone a movement is counted in, per approach lane group.
pub struct ZoneRef {
    pub approach: usize,
    pub directions: BTreeSet<Direction>,
    pub lanes: Vec<u32>,
    pub id: String,
}

pub struct Link {
    pub zone: String,
    pub from_edge: Option<String>,
    pub from_lane: Option<u32>,
    pub to_edge: Option<String>,
    pub direction: String,
}

pub struct GreenPhase {
    pub kind: &'static str,
    pub duration: f64,
    pub movements: Vec<Signal>,
    pub crossings: Vec<Signal>,
}

pub struct Plan {
    pub links: Vec<Link>,
    /// `(duration, state)`, green and amber phases alternating.
    pub phases: Vec<(f64, String)>,
    pub is_green_phase: Vec<bool>,
    pub zone_phases: Vec<(String, Vec<i32>)>,
    /// `(green phase, amber phase, amber seconds)`.
    pub transitions: Vec<(i32, i32, f64)>,
    pub conflicts: Vec<(String, String)>,
    /// Movement-level result, for invariant checks.
    pub green_phases: Vec<GreenPhase>,
    pub movement_conflicts: Vec<Vec<bool>>,
    pub crossing_conflicts: Vec<Vec<bool>>,
    pub opposite: Vec<Option<usize>>,
    pub movement_approach: Vec<usize>,
    pub movement_direction: Vec<Direction>,
    /// Each movement's heading as it reaches the junction.
    pub movement_heading: Vec<Pt>,
    /// How many points each movement's conflict path has; one with fewer
    /// than two can't conflict with anything, which is never right.
    pub path_points: Vec<usize>,
}

#[allow(clippy::too_many_arguments)]
pub fn build(
    graph: &Graph,
    positions: &HashMap<NodeId, Pt>,
    cluster: &Cluster,
    junction: &Junction,
    setbacks: &HashMap<usize, f64>,
    zones: &[ZoneRef],
    crossings: &[Crossing],
) -> Plan {
    let movements = &junction.movements;
    let n = movements.len();
    let approach_of: Vec<usize> = movements.iter().map(|m| m.approach).collect();

    let paths: Vec<Vec<Pt>> = movements
        .iter()
        .map(|m| {
            path(
                graph,
                positions,
                m.approach,
                m.exit,
                &m.via,
                lanes_for(junction, m.approach, m.direction),
                setbacks,
            )
        })
        .collect();
    let mut movement_conflicts: Vec<Vec<bool>> = (0..n)
        .map(|i| {
            (0..n)
                .map(|j| {
                    i != j
                        && approach_of[i] != approach_of[j]
                        && (movements[i].exit == movements[j].exit
                            || geometry::polylines_meet(&paths[i], &paths[j]))
                })
                .collect()
        })
        .collect();
    // Two straight movements from perpendicular approaches cross, whatever
    // the approximate paths say: in a controller spanning several nodes
    // their paths can miss each other on the map while the streets still
    // cross (the same rule `violations` checks).
    let approach_heading = |m: usize| {
        heading_at_end(&graph.edges[movements[m].approach].points).unwrap_or(DEFAULT_HEADING)
    };
    for i in 0..n {
        for j in 0..n {
            let angle = turn_degrees(approach_heading(i), approach_heading(j)).abs();
            if i != j
                && movements[i].direction == Direction::Straight
                && movements[j].direction == Direction::Straight
                && (MIN_PERPENDICULAR_DEGREES..=MAX_PERPENDICULAR_DEGREES).contains(&angle)
            {
                movement_conflicts[i][j] = true;
            }
        }
    }
    // A crosswalk conflicts with a movement whose path it meets — or,
    // whatever the approximate lines say, whose own nodes it sits on.
    let crossing_conflicts: Vec<Vec<bool>> = crossings
        .iter()
        .map(|c| {
            movements
                .iter()
                .zip(&paths)
                .map(|(m, p)| m.via.contains(&c.anchor) || geometry::polylines_meet(&c.line, p))
                .collect()
        })
        .collect();

    // Pair approaches that face each other (cars only).
    let car_approaches: Vec<usize> = junction
        .approaches
        .iter()
        .map(|a| a.edge)
        .filter(|&e| graph.edges[e].mode == Mode::Car)
        .collect();
    let order = clockwise(graph, cluster, &car_approaches);
    let heading = |e: usize| heading_at_end(&graph.edges[e].points).unwrap_or(DEFAULT_HEADING);
    let mut candidates: Vec<(f64, usize, usize)> = Vec::new();
    for (i, &a) in order.iter().enumerate() {
        for &b in &order[i + 1..] {
            let angle = turn_degrees(heading(a), heading(b)).abs();
            if angle >= OPPOSITE_DEGREES {
                candidates.push((180.0 - angle, a, b));
            }
        }
    }
    candidates.sort_by(|x, y| x.0.total_cmp(&y.0));
    let mut opposite_edge: HashMap<usize, usize> = HashMap::new();
    for (_, a, b) in candidates {
        if !opposite_edge.contains_key(&a) && !opposite_edge.contains_key(&b) {
            opposite_edge.insert(a, b);
            opposite_edge.insert(b, a);
        }
    }
    let opposite: Vec<Option<usize>> = approach_of
        .iter()
        .map(|a| opposite_edge.get(a).copied())
        .collect();
    // Opposite left turns pass each other rather than cross — which is why
    // they can share a protected phase — whatever the approximate paths
    // above say about the middle of the junction.
    for i in 0..n {
        for j in 0..n {
            if movements[i].direction == Direction::Left
                && movements[j].direction == Direction::Left
                && opposite[i] == Some(approach_of[j])
                && movements[i].exit != movements[j].exit
            {
                movement_conflicts[i][j] = false;
            }
        }
    }
    let mut units: Vec<Vec<usize>> = Vec::new();
    for &a in &order {
        if units.iter().any(|u| u.contains(&a)) {
            continue;
        }
        units.push(match opposite_edge.get(&a) {
            Some(&b) => vec![a, b],
            None => vec![a],
        });
    }

    let conflicts = |i: usize, j: usize| movement_conflicts[i][j];
    let lanes_of_unit = |unit: &[usize]| unit.iter().map(|&e| graph.edges[e].lanes).sum::<u32>();
    let busiest = units.iter().map(|u| lanes_of_unit(u)).max().unwrap_or(0);
    let dedicated_left = |m: usize| {
        zones.iter().any(|z| {
            z.approach == approach_of[m]
                && z.directions.contains(&movements[m].direction)
                && z.directions
                    .iter()
                    .all(|d| matches!(d, Direction::Left | Direction::UTurn))
        })
    };

    let mut phases: Vec<GreenPhase> = Vec::new();
    let new_phase = |kind: &'static str, duration: f64| GreenPhase {
        kind,
        duration,
        movements: vec![Signal::Red; n],
        crossings: vec![Signal::Red; crossings.len()],
    };
    for unit in &units {
        let mut phase = new_phase(
            "main",
            if lanes_of_unit(unit) == busiest {
                MAIN_GREEN_SECS
            } else {
                MINOR_GREEN_SECS
            },
        );
        let in_unit = |m: usize| unit.contains(&approach_of[m]);
        let is_turn_left =
            |m: usize| matches!(movements[m].direction, Direction::Left | Direction::UTurn);
        for m in (0..n).filter(|&m| in_unit(m) && (!is_turn_left(m) || unit.len() == 1)) {
            if (0..n).all(|o| !phase.movements[o].is_green() || !conflicts(m, o)) {
                phase.movements[m] = Signal::Green;
            }
        }
        let mut waiting_lefts = Vec::new();
        for m in (0..n).filter(|&m| in_unit(m) && is_turn_left(m) && unit.len() == 2) {
            let against: Vec<usize> = (0..n)
                .filter(|&o| phase.movements[o].is_green() && conflicts(m, o))
                .collect();
            if against.is_empty() {
                phase.movements[m] = Signal::Green;
            } else if against.iter().all(|&o| Some(approach_of[o]) == opposite[m]) {
                phase.movements[m] = Signal::Permissive;
                waiting_lefts.push(m);
            } else {
                waiting_lefts.push(m);
            }
        }
        phases.push(phase);

        let protected: Vec<usize> = waiting_lefts
            .into_iter()
            .filter(|&m| dedicated_left(m))
            .collect();
        if !protected.is_empty() {
            let mut phase = new_phase("protected_left", PROTECTED_LEFT_SECS);
            for m in protected
                .into_iter()
                .chain((0..n).filter(|&m| in_unit(m) && movements[m].direction == Direction::Right))
            {
                if (0..n).all(|o| !phase.movements[o].is_green() || !conflicts(m, o)) {
                    phase.movements[m] = Signal::Green;
                }
            }
            phases.push(phase);
        }
    }

    // Every movement green somewhere: bigger approaches first.
    let mut missing: Vec<usize> = (0..n)
        .filter(|&m| phases.iter().all(|p| !p.movements[m].is_green()))
        .collect();
    missing.sort_by_key(|&m| (std::cmp::Reverse(graph.edges[approach_of[m]].lanes), m));
    for m in missing {
        if phases.iter().any(|p| p.movements[m].is_green()) {
            continue;
        }
        let fits = |p: &GreenPhase| (0..n).all(|o| !p.movements[o].is_green() || !conflicts(m, o));
        if let Some(phase) = phases.iter_mut().find(|p| fits(p)) {
            phase.movements[m] = Signal::Green;
        } else {
            let mut phase = new_phase("extra", EXTRA_GREEN_SECS);
            phase.movements[m] = Signal::Green;
            phases.push(phase);
        }
    }

    // Crosswalks: green unless something crossing them goes straight; the
    // turns that cross a green crosswalk yield to it.
    let settle_crossings = |phase: &mut GreenPhase| {
        for (c, conflicts_with) in crossing_conflicts.iter().enumerate() {
            let active: Vec<usize> = (0..n)
                .filter(|&m| conflicts_with[m] && phase.movements[m].is_green())
                .collect();
            if active
                .iter()
                .all(|&m| movements[m].direction != Direction::Straight)
            {
                phase.crossings[c] = Signal::Green;
                for m in active {
                    phase.movements[m] = Signal::Permissive;
                }
            }
        }
    };
    for phase in phases.iter_mut() {
        settle_crossings(phase);
    }
    if (0..crossings.len()).any(|c| phases.iter().all(|p| !p.crossings[c].is_green())) {
        let mut phase = new_phase("pedestrian", PEDESTRIAN_GREEN_SECS);
        settle_crossings(&mut phase);
        phases.push(phase);
    }

    // Links: approaches clockwise, lanes right to left, then crosswalks.
    let all_approaches: Vec<usize> = junction.approaches.iter().map(|a| a.edge).collect();
    let mut links: Vec<Link> = Vec::new();
    let mut link_source: Vec<Result<usize, usize>> = Vec::new();
    for edge in clockwise(graph, cluster, &all_approaches) {
        let approach = junction
            .approaches
            .iter()
            .find(|a| a.edge == edge)
            .expect("listed above");
        for (lane, directions) in approach.lane_directions.iter().enumerate() {
            let Some(zone) = zones
                .iter()
                .find(|z| z.approach == edge && z.lanes.contains(&(lane as u32)))
            else {
                continue;
            };
            let mut served: Vec<usize> = approach
                .movements
                .iter()
                .copied()
                .filter(|&m| directions.contains(&movements[m].direction))
                .collect();
            served.sort_by_key(|&m| (movements[m].direction, movements[m].exit));
            for m in served {
                links.push(Link {
                    zone: zone.id.clone(),
                    from_edge: Some(graph.edges[edge].id.clone()),
                    from_lane: Some(lane as u32),
                    to_edge: Some(graph.edges[movements[m].exit].id.clone()),
                    direction: movements[m].direction.label().to_string(),
                });
                link_source.push(Ok(m));
            }
        }
    }
    for (c, crossing) in crossings.iter().enumerate() {
        links.push(Link {
            zone: crossing.zone_id.clone(),
            from_edge: None,
            from_lane: None,
            to_edge: None,
            direction: "crossing".into(),
        });
        link_source.push(Err(c));
    }

    let signal = |phase: &GreenPhase, source: &Result<usize, usize>| match *source {
        Ok(m) => phase.movements[m],
        Err(c) => phase.crossings[c],
    };
    let always_green: Vec<bool> = link_source
        .iter()
        .map(|s| phases.iter().all(|p| signal(p, s).is_green()))
        .collect();
    let fast = movements
        .iter()
        .any(|m| graph.edges[m.approach].speed_kmh > FAST_SPEED_KMH);
    let amber_secs = if fast { FAST_AMBER_SECS } else { AMBER_SECS };
    let mut program: Vec<(f64, String)> = Vec::new();
    let mut is_green_phase = Vec::new();
    let mut transitions = Vec::new();
    for phase in &phases {
        let green: String = link_source
            .iter()
            .map(|s| match signal(phase, s) {
                Signal::Green => 'G',
                Signal::Permissive => 'g',
                Signal::Red => 'r',
            })
            .collect();
        let amber: String = link_source
            .iter()
            .zip(&always_green)
            .zip(green.chars())
            .map(|((_, &always), state)| match state {
                'G' | 'g' if always => state,
                'G' | 'g' => 'y',
                _ => 'r',
            })
            .collect();
        let index = program.len() as i32;
        transitions.push((index, index + 1, amber_secs));
        program.push((phase.duration, green));
        program.push((amber_secs, amber));
        is_green_phase.extend([true, false]);
    }

    // Which phases each zone is green in: any of its links G/g.
    let mut zone_phases: Vec<(String, Vec<i32>)> = Vec::new();
    let mut zone_ids: Vec<String> = links.iter().map(|l| l.zone.clone()).collect();
    zone_ids.sort();
    zone_ids.dedup();
    for zone in zone_ids {
        let indices: Vec<i32> = program
            .iter()
            .enumerate()
            .filter(|(i, (_, state))| {
                is_green_phase[*i]
                    && links
                        .iter()
                        .zip(state.chars())
                        .any(|(l, s)| l.zone == zone && matches!(s, 'G' | 'g'))
            })
            .map(|(i, _)| i as i32)
            .collect();
        zone_phases.push((zone, indices));
    }

    // Zone-level conflicts: any movement (or crosswalk) of one crossing any
    // of the other's.
    let zones_of_movement = |m: usize| -> Vec<&str> {
        zones
            .iter()
            .filter(|z| {
                z.approach == approach_of[m] && z.directions.contains(&movements[m].direction)
            })
            .map(|z| z.id.as_str())
            .collect()
    };
    let mut conflict_set: BTreeSet<(String, String)> = BTreeSet::new();
    let mut add = |a: &str, b: &str| {
        if a != b {
            let (x, y) = if a < b { (a, b) } else { (b, a) };
            conflict_set.insert((x.to_string(), y.to_string()));
        }
    };
    for (i, row) in movement_conflicts.iter().enumerate() {
        for (j, &conflict) in row.iter().enumerate().skip(i + 1) {
            if conflict {
                for a in zones_of_movement(i) {
                    for b in zones_of_movement(j) {
                        add(a, b);
                    }
                }
            }
        }
    }
    for (c, crossing) in crossings.iter().enumerate() {
        for m in (0..n).filter(|&m| crossing_conflicts[c][m]) {
            for z in zones_of_movement(m) {
                add(&crossing.zone_id, z);
            }
        }
    }

    Plan {
        links,
        phases: program,
        is_green_phase,
        zone_phases,
        transitions,
        conflicts: conflict_set.into_iter().collect(),
        green_phases: phases,
        movement_conflicts,
        crossing_conflicts,
        opposite,
        movement_approach: approach_of,
        movement_direction: movements.iter().map(|m| m.direction).collect(),
        movement_heading: movements
            .iter()
            .map(|m| heading_at_end(&graph.edges[m.approach].points).unwrap_or(DEFAULT_HEADING))
            .collect(),
        path_points: paths.iter().map(Vec::len).collect(),
    }
}

/// Every way `plan` breaks the rules a signal program must keep; empty
/// when it's sound.
pub fn violations(plan: &Plan) -> Vec<String> {
    let mut problems = Vec::new();
    let n = plan.movement_direction.len();
    for (m, &points) in plan.path_points.iter().enumerate() {
        if points < 2 {
            problems.push(format!(
                "movement {m} has a degenerate path ({points} point(s))"
            ));
        }
    }
    // Independent of the path geometry: two straight movements from
    // perpendicular approaches cross, whatever the paths say.
    let perpendicular = |i: usize, j: usize| {
        let angle = turn_degrees(plan.movement_heading[i], plan.movement_heading[j]).abs();
        (MIN_PERPENDICULAR_DEGREES..=MAX_PERPENDICULAR_DEGREES).contains(&angle)
    };
    let turn = |m: usize| plan.movement_direction[m] != Direction::Straight;
    for (p, phase) in plan.green_phases.iter().enumerate() {
        for i in 0..n {
            for j in i + 1..n {
                if plan.movement_conflicts[i][j]
                    && phase.movements[i] == Signal::Green
                    && phase.movements[j] == Signal::Green
                {
                    problems.push(format!(
                        "phase {p}: conflicting movements {i} and {j} both G"
                    ));
                }
            }
            for j in 0..n {
                if plan.movement_direction[i] == Direction::Straight
                    && plan.movement_direction[j] == Direction::Straight
                    && i < j
                    && perpendicular(i, j)
                    && phase.movements[i].is_green()
                    && phase.movements[j].is_green()
                {
                    problems.push(format!(
                        "phase {p}: perpendicular straight movements {i} and {j} both green"
                    ));
                }
            }
            if phase.movements[i] == Signal::Permissive {
                for o in (0..n)
                    .filter(|&o| plan.movement_conflicts[i][o] && phase.movements[o].is_green())
                {
                    if Some(plan.movement_approach[o]) != plan.opposite[i] {
                        problems.push(format!(
                            "phase {p}: movement {i} yields to {o}, which isn't opposite it"
                        ));
                    }
                }
            }
        }
        for (c, crossing) in phase.crossings.iter().enumerate() {
            if !crossing.is_green() {
                continue;
            }
            for m in (0..n).filter(|&m| plan.crossing_conflicts[c][m]) {
                match phase.movements[m] {
                    Signal::Green => problems.push(format!(
                        "phase {p}: crosswalk {c} green with movement {m} G across it"
                    )),
                    Signal::Permissive if !turn(m) => problems.push(format!(
                        "phase {p}: crosswalk {c} green with straight movement {m} across it"
                    )),
                    _ => {}
                }
            }
        }
    }
    let green_somewhere = |state_index: usize| {
        plan.phases
            .iter()
            .zip(&plan.is_green_phase)
            .any(|((_, s), &g)| g && matches!(s.as_bytes()[state_index], b'G' | b'g'))
    };
    for (index, link) in plan.links.iter().enumerate() {
        if !green_somewhere(index) {
            problems.push(format!("link {index} ({}) is never green", link.zone));
        }
    }
    for (i, ((_, state), &green)) in plan.phases.iter().zip(&plan.is_green_phase).enumerate() {
        if state.len() != plan.links.len() {
            problems.push(format!(
                "phase {i}: {} states for {} links",
                state.len(),
                plan.links.len()
            ));
        }
        if green {
            let (_, amber) = &plan.phases[i + 1];
            let always = |k: usize| {
                plan.phases
                    .iter()
                    .zip(&plan.is_green_phase)
                    .all(|((_, s), &g)| !g || matches!(s.as_bytes()[k], b'G' | b'g'))
            };
            for (k, (now, next)) in state.bytes().zip(amber.bytes()).enumerate() {
                if matches!(now, b'G' | b'g') && !always(k) && next != b'y' {
                    problems.push(format!(
                        "phase {}: link {k} goes from green to {} instead of amber",
                        i + 1,
                        next as char
                    ));
                }
            }
        }
    }
    problems
}

fn lanes_for(junction: &Junction, approach: usize, direction: Direction) -> Vec<u32> {
    junction
        .approaches
        .iter()
        .find(|a| a.edge == approach)
        .map(|a| {
            a.lane_directions
                .iter()
                .enumerate()
                .filter(|(_, d)| d.contains(&direction))
                .map(|(i, _)| i as u32)
                .collect()
        })
        .unwrap_or_default()
}

/// The ground a movement covers: from its stop line, along its lanes to the
/// junction, curving into its exit, and a stretch along the exit.
fn path(
    graph: &Graph,
    positions: &HashMap<NodeId, Pt>,
    approach: usize,
    exit: usize,
    via: &[NodeId],
    lanes: Vec<u32>,
    setbacks: &HashMap<usize, f64>,
) -> Vec<Pt> {
    let a = &graph.edges[approach];
    let x = &graph.edges[exit];
    let lane_offset = if lanes.is_empty() {
        a.lane_offset(0)
    } else {
        lanes.iter().map(|&l| a.lane_offset(l)).sum::<f64>() / lanes.len() as f64
    };
    let approach_line = geometry::offset(&a.points, lane_offset);
    let stop = (a.length - setbacks.get(&approach).copied().unwrap_or(0.0)).max(0.0);
    let mut points = geometry::sub_polyline(&approach_line, stop, a.length);
    // A stop line right at the approach's end (a light on the node itself)
    // leaves nothing to cut: the path still starts there.
    if points.is_empty()
        && let Some(&end) = approach_line.last()
    {
        points.push(end);
    }
    let exit_line = geometry::offset(&x.points, x.lane_offset(x.lanes / 2));
    let tail = geometry::sub_polyline(&exit_line, 0.0, EXIT_TAIL_METERS);
    // A multi-node junction: follow each internal edge on its own lanes, so
    // two opposite movements through the same nodes stay on their own side
    // of the road instead of meeting on its centreline.
    let internal: Vec<Vec<Pt>> = via
        .windows(2)
        .map(|hop| {
            graph
                .outgoing(hop[0])
                .iter()
                .map(|&e| &graph.edges[e])
                .find(|e| e.to == hop[1])
                .map(|e| e.lane_line(e.lanes / 2))
                .unwrap_or_else(|| {
                    hop.iter()
                        .filter_map(|n| positions.get(n).copied())
                        .collect()
                })
        })
        .collect();
    for piece in internal.iter().chain(std::iter::once(&tail)) {
        join(&mut points, piece);
    }
    points.dedup();
    points
}

/// Appends `next` to `points`, bridging the gap between them with a curve
/// from one's heading into the other's when they meet at an angle.
fn join(points: &mut Vec<Pt>, next: &[Pt]) {
    let (Some(&from), Some(&to)) = (points.last(), next.first()) else {
        points.extend_from_slice(next);
        return;
    };
    let bend = match (heading_at_end(points), heading_at_start(next)) {
        (Some(d), Some(e)) => geometry::line_intersection(from, d, to, e)
            .filter(|&(t, u)| {
                t > 0.0 && u < 0.0 && t < BEND_CONTROL_MAX_METERS && -u < BEND_CONTROL_MAX_METERS
            })
            .map(|(t, _)| [from[0] + d[0] * t, from[1] + d[1] * t]),
        _ => None,
    };
    match bend {
        Some(control) => points.extend(
            geometry::bezier(from, control, to, BEZIER_SEGMENTS)
                .into_iter()
                .skip(1),
        ),
        None => points.push(to),
    }
    points.extend(next.iter().skip(1).copied());
}

/// `edges` ordered clockwise (from north) by where each ends around the
/// junction's centre.
fn clockwise(graph: &Graph, cluster: &Cluster, edges: &[usize]) -> Vec<usize> {
    let bearing = |e: usize| {
        let edge = &graph.edges[e];
        let p = *geometry::sub_polyline(
            &edge.points,
            (edge.length - BEARING_BASELINE_METERS).max(0.0),
            edge.length,
        )
        .first()
        .unwrap_or(&edge.points[0]);
        let (dx, dy) = (p[0] - cluster.centre[0], p[1] - cluster.centre[1]);
        dx.atan2(dy).rem_euclid(std::f64::consts::TAU)
    };
    let mut sorted = edges.to_vec();
    sorted.sort_by(|&a, &b| bearing(a).total_cmp(&bearing(b)).then(a.cmp(&b)));
    sorted
}

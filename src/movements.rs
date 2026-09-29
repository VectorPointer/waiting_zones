//! What each approach of a signalized junction can do: which exits it
//! reaches through the junction, how it turns to get there, and which of
//! its lanes serve which turn.

use std::collections::{BTreeMap, BTreeSet, BinaryHeap, HashMap};

use crate::clusters::Cluster;
use crate::geometry::{self, Pt};
use crate::graph::Graph;
use crate::network::{Direction, LaneArrows, Mode, Network};
use crate::osm::{NodeId, Osm};

/// How much of an edge's end (or start) its heading is measured over, so a
/// last short kink in the OSM line doesn't decide the turn.
const HEADING_BASELINE_METERS: f64 = 10.0;
const STRAIGHT_DEGREES: f64 = 30.0;
const UTURN_DEGREES: f64 = 150.0;

pub struct Movement {
    pub approach: usize,
    pub exit: usize,
    pub direction: Direction,
    /// Nodes the movement passes through inside the junction, including
    /// the approach's last node and the exit's first.
    pub via: Vec<NodeId>,
}

/// One approach of a junction and how its lanes split between movements.
pub struct Approach {
    pub edge: usize,
    /// Indices into the junction's movements.
    pub movements: Vec<usize>,
    /// Per lane (0 = rightmost), the turns it serves.
    pub lane_directions: Vec<BTreeSet<Direction>>,
}

impl Approach {
    /// Lanes grouped by the exact set of turns they serve: each group is
    /// one waiting zone.
    pub fn groups(&self) -> BTreeMap<BTreeSet<Direction>, Vec<u32>> {
        let mut groups: BTreeMap<BTreeSet<Direction>, Vec<u32>> = BTreeMap::new();
        for (lane, directions) in self.lane_directions.iter().enumerate() {
            if !directions.is_empty() {
                groups
                    .entry(directions.clone())
                    .or_default()
                    .push(lane as u32);
            }
        }
        groups
    }
}

pub struct Junction {
    pub approaches: Vec<Approach>,
    pub movements: Vec<Movement>,
}

pub fn heading_at_end(points: &[Pt]) -> Option<Pt> {
    let length = geometry::length(points);
    let tail = geometry::sub_polyline(points, length - HEADING_BASELINE_METERS, length);
    let (a, b) = (tail.first()?, tail.last()?);
    geometry::unit([b[0] - a[0], b[1] - a[1]])
}

pub fn heading_at_start(points: &[Pt]) -> Option<Pt> {
    let head = geometry::sub_polyline(points, 0.0, HEADING_BASELINE_METERS);
    let (a, b) = (head.first()?, head.last()?);
    geometry::unit([b[0] - a[0], b[1] - a[1]])
}

/// Signed turn from heading `a` to heading `b`, degrees, left positive.
pub fn turn_degrees(a: Pt, b: Pt) -> f64 {
    (a[0] * b[1] - a[1] * b[0])
        .atan2(a[0] * b[0] + a[1] * b[1])
        .to_degrees()
}

pub fn classify(degrees: f64) -> Direction {
    match degrees {
        d if d.abs() < STRAIGHT_DEGREES => Direction::Straight,
        d if d.abs() > UTURN_DEGREES => Direction::UTurn,
        d if d > 0.0 => Direction::Left,
        _ => Direction::Right,
    }
}

pub fn build(osm: &Osm, net: &Network, graph: &Graph, cluster: &Cluster) -> Junction {
    let mut movements = Vec::new();
    let mut approaches = Vec::new();
    for &approach in &cluster.approaches {
        let a = &graph.edges[approach];
        // Cyclists on a cycleway cross with the pedestrians, from the
        // sidewalk: they wait at the crosswalk's own zone, not in a lane.
        if a.mode != Mode::Car {
            continue;
        }
        let Some(heading_in) = heading_at_end(&a.points) else {
            continue;
        };
        let mut found: Vec<Movement> = reachable(graph, cluster, approach)
            .into_iter()
            .filter_map(|(exit, via)| {
                let heading_out = heading_at_start(&graph.edges[exit].points)?;
                let direction = classify(turn_degrees(heading_in, heading_out));
                Some(Movement {
                    approach,
                    exit,
                    direction,
                    via,
                })
            })
            .collect();
        apply_restrictions(osm, net, graph, &mut found);
        // A U-turn only when it's the only way out.
        if found.iter().any(|m| m.direction != Direction::UTurn) {
            found.retain(|m| m.direction != Direction::UTurn);
        }
        if found.is_empty() {
            continue;
        }
        let directions: BTreeSet<Direction> = found.iter().map(|m| m.direction).collect();
        let lane_directions = assign_lanes(a.lanes, a.arrows.as_deref(), &directions);
        let first = movements.len();
        movements.extend(found);
        approaches.push(Approach {
            edge: approach,
            movements: (first..movements.len()).collect(),
            lane_directions,
        });
    }
    Junction {
        approaches,
        movements,
    }
}

/// Every exit reachable from `approach` through the cluster's own internal
/// edges, with the nodes it passes on the shortest such path.
fn reachable(graph: &Graph, cluster: &Cluster, approach: usize) -> Vec<(usize, Vec<NodeId>)> {
    let a = &graph.edges[approach];
    let usable = |edge: usize| graph.edges[edge].mode == Mode::Car;
    let start = a.to;
    let mut best: HashMap<NodeId, (f64, Vec<NodeId>)> =
        HashMap::from([(start, (0.0, vec![start]))]);
    let mut queue = BinaryHeap::from([(std::cmp::Reverse(0u64), start)]);
    while let Some((std::cmp::Reverse(cost), node)) = queue.pop() {
        let (distance, path) = best[&node].clone();
        if (distance * 1000.0) as u64 != cost {
            continue;
        }
        for &e in graph.outgoing(node) {
            let edge = &graph.edges[e];
            if !cluster.contains(edge.to) || !usable(e) || Some(e) == a.reverse {
                continue;
            }
            let next = distance + edge.length;
            if best.get(&edge.to).is_none_or(|(d, _)| next < *d) {
                let mut next_path = path.clone();
                next_path.push(edge.to);
                best.insert(edge.to, (next, next_path));
                queue.push((std::cmp::Reverse((next * 1000.0) as u64), edge.to));
            }
        }
    }
    let mut exits: Vec<(usize, Vec<NodeId>)> = cluster
        .exits
        .iter()
        .filter(|&&x| usable(x))
        .filter_map(|&x| {
            best.get(&graph.edges[x].from)
                .map(|(_, path)| (x, path.clone()))
        })
        .collect();
    exits.sort_by_key(|(x, _)| *x);
    exits
}

/// Drops movements an OSM `type=restriction` relation forbids. A
/// restriction names ways, and a way can leave the junction both ways, so
/// its turn kind (`left_turn`, …) decides which movement onto `to` it means.
fn apply_restrictions(osm: &Osm, net: &Network, graph: &Graph, movements: &mut Vec<Movement>) {
    let way = |edge: usize| net.roads[graph.edges[edge].road].way;
    let turn = |kind: &str| match kind {
        "left_turn" => Some(Direction::Left),
        "right_turn" => Some(Direction::Right),
        "straight_on" => Some(Direction::Straight),
        "u_turn" => Some(Direction::UTurn),
        _ => None,
    };
    movements.retain(|m| {
        osm.restrictions.iter().all(|r| {
            if r.from != way(m.approach) || !m.via.contains(&r.via) {
                return true;
            }
            let named = r.to == way(m.exit) && turn(&r.turn).is_none_or(|d| d == m.direction);
            if r.only { named } else { !named }
        })
    });
}

/// Which turns each of `lanes` lanes serves (index 0 = rightmost), given
/// the turns the approach allows. Painted arrows rule when mapped;
/// otherwise the right lane takes the right turn, the left lane the left
/// turn, and every other lane goes straight.
pub fn assign_lanes(
    lanes: u32,
    arrows: Option<&[LaneArrows]>,
    allowed: &BTreeSet<Direction>,
) -> Vec<BTreeSet<Direction>> {
    let n = lanes as usize;
    if let Some(arrows) = arrows {
        let assigned: Vec<BTreeSet<Direction>> = arrows
            .iter()
            .map(|lane| {
                // An unmarked lane, or one painted for a turn this junction
                // doesn't offer (a slip road that leaves before it, say),
                // goes straight on when it can.
                let marked: BTreeSet<Direction> = lane.intersection(allowed).copied().collect();
                match (marked.is_empty(), allowed.contains(&Direction::Straight)) {
                    (false, _) => marked,
                    (true, true) => [Direction::Straight].into(),
                    (true, false) => allowed.clone(),
                }
            })
            .collect();
        let served: BTreeSet<Direction> = assigned.iter().flatten().copied().collect();
        if served == *allowed {
            return assigned;
        }
    }
    if n <= 1 {
        return vec![allowed.clone(); n];
    }
    let has = |d: Direction| allowed.contains(&d);
    let left_turns: BTreeSet<Direction> = [Direction::Left, Direction::UTurn]
        .into_iter()
        .filter(|&d| has(d))
        .collect();
    let mut lanes_out = vec![BTreeSet::new(); n];
    if has(Direction::Straight) {
        for lane in lanes_out.iter_mut() {
            lane.insert(Direction::Straight);
        }
        if has(Direction::Right) {
            lanes_out[0].insert(Direction::Right);
        }
        if !left_turns.is_empty() {
            if n >= 3 {
                lanes_out[n - 1] = left_turns;
            } else {
                lanes_out[n - 1].extend(left_turns);
            }
        }
    } else {
        // No straight on: the right half turns right, the left half left.
        for (i, lane) in lanes_out.iter_mut().enumerate() {
            let rightward = 2 * i < n;
            let leftward = 2 * i + 1 >= n;
            if has(Direction::Right) && (rightward || left_turns.is_empty()) {
                lane.insert(Direction::Right);
            }
            if leftward || !has(Direction::Right) {
                lane.extend(left_turns.iter().copied());
            }
        }
    }
    lanes_out
}

#[cfg(test)]
mod tests {
    use super::*;
    use Direction::*;

    fn set(ds: &[Direction]) -> BTreeSet<Direction> {
        ds.iter().copied().collect()
    }

    #[test]
    fn turns_are_classified_by_angle() {
        let east = [1.0, 0.0];
        assert_eq!(classify(turn_degrees(east, [1.0, 0.1])), Straight);
        assert_eq!(classify(turn_degrees(east, [0.0, 1.0])), Left);
        assert_eq!(classify(turn_degrees(east, [0.0, -1.0])), Right);
        assert_eq!(classify(turn_degrees(east, [-1.0, 0.01])), UTurn);
    }

    #[test]
    fn unmarked_lanes_split_right_straight_left() {
        let all = set(&[Straight, Left, Right]);
        assert_eq!(assign_lanes(1, None, &all), vec![all.clone()]);
        assert_eq!(
            assign_lanes(2, None, &all),
            vec![set(&[Straight, Right]), set(&[Straight, Left])]
        );
        assert_eq!(
            assign_lanes(3, None, &all),
            vec![set(&[Straight, Right]), set(&[Straight]), set(&[Left])]
        );
        // A T-junction's stem: no straight on.
        assert_eq!(
            assign_lanes(2, None, &set(&[Left, Right])),
            vec![set(&[Right]), set(&[Left])]
        );
    }

    #[test]
    fn painted_arrows_win_over_the_default_split() {
        let all = set(&[Straight, Left, Right]);
        let arrows = vec![set(&[Straight, Right]), set(&[Straight]), set(&[Left])];
        assert_eq!(assign_lanes(3, Some(&arrows), &all), arrows);
        // Arrows for a turn the junction doesn't allow fall back to the split.
        let wrong = vec![set(&[Right]), set(&[Right]), set(&[Right])];
        assert_eq!(assign_lanes(3, Some(&wrong), &all)[2], set(&[Left]));
        // A lane painted for a turn that isn't there goes straight on.
        let no_right = set(&[Straight, Left]);
        let arrows = vec![set(&[Right]), set(&[Straight]), set(&[Straight, Left])];
        assert_eq!(
            assign_lanes(3, Some(&arrows), &no_right),
            vec![set(&[Straight]), set(&[Straight]), set(&[Straight, Left])]
        );
    }
}

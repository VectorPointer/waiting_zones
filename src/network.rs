//! The road network as OSM describes it: every way a car can drive along,
//! with its allowed directions and lane counts, plus which node is where
//! and which roads meet at it. Cycleways aren't part of it: cyclists are
//! left out of waiting zones altogether.

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::geometry::Pt;
use crate::osm::{NodeId, Osm, WayId};
use crate::projection::Projection;

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Mode {
    Car,
}

impl Mode {
    pub fn lane_width(self) -> f64 {
        match self {
            Mode::Car => CAR_LANE_WIDTH_METERS,
        }
    }
}

/// Barcelona's urban lanes are ~3m; OSM rarely carries `width`.
pub const CAR_LANE_WIDTH_METERS: f64 = 3.0;

/// Direction of travel relative to the way's own node order.
#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
pub enum Dir {
    Forward,
    Backward,
}

impl Dir {
    pub fn tag(self) -> &'static str {
        match self {
            Dir::Forward => "fwd",
            Dir::Backward => "bwd",
        }
    }
}

/// Where a vehicle goes through a junction, relative to where it came from.
/// Ordered the way zone ids list them.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug)]
pub enum Direction {
    Straight,
    UTurn,
    Left,
    Right,
}

impl Direction {
    pub fn label(self) -> &'static str {
        match self {
            Direction::Straight => "straight",
            Direction::UTurn => "turn",
            Direction::Left => "left",
            Direction::Right => "right",
        }
    }
}

/// One lane's painted arrows (`turn:lanes`); empty when unmarked.
pub type LaneArrows = BTreeSet<Direction>;

pub struct Road {
    pub way: WayId,
    pub mode: Mode,
    /// An OSM slip road / ramp (`highway=*_link`): a branch off a street,
    /// never the street a queue carries on along.
    pub is_link: bool,
    pub nodes: Vec<NodeId>,
    pub points: Vec<Pt>,
    pub forward_lanes: u32,
    pub backward_lanes: u32,
    /// Per-lane arrows for each direction, index 0 = rightmost lane; `None`
    /// when OSM doesn't map them (or maps a different lane count).
    pub forward_arrows: Option<Vec<LaneArrows>>,
    pub backward_arrows: Option<Vec<LaneArrows>>,
    pub speed_kmh: f64,
}

impl Road {
    pub fn lanes(&self, dir: Dir) -> u32 {
        match dir {
            Dir::Forward => self.forward_lanes,
            Dir::Backward => self.backward_lanes,
        }
    }

    pub fn allows(&self, dir: Dir) -> bool {
        self.lanes(dir) > 0
    }

    pub fn arrows(&self, dir: Dir) -> Option<&Vec<LaneArrows>> {
        match dir {
            Dir::Forward => self.forward_arrows.as_ref(),
            Dir::Backward => self.backward_arrows.as_ref(),
        }
    }

    pub fn is_two_way(&self) -> bool {
        self.forward_lanes > 0 && self.backward_lanes > 0
    }

    /// Total carriageway width, both directions.
    pub fn width(&self) -> f64 {
        f64::from(self.forward_lanes + self.backward_lanes) * self.mode.lane_width()
    }
}

/// Where a road touches a node: which road, and at which index of its own
/// node list.
#[derive(Clone, Copy)]
pub struct Incidence {
    pub road: usize,
    pub index: usize,
}

pub struct Network {
    pub roads: Vec<Road>,
    pub positions: HashMap<NodeId, Pt>,
    pub incidences: HashMap<NodeId, Vec<Incidence>>,
}

impl Network {
    pub fn build(osm: &Osm, projection: &Projection) -> Self {
        let positions: HashMap<NodeId, Pt> = osm
            .nodes
            .values()
            .map(|node| (node.id, projection.to_local(node.lon_lat)))
            .collect();

        let mut roads = Vec::new();
        for way in &osm.ways {
            let Some(highway) = way.tag("highway") else {
                continue;
            };
            let mode = match highway {
                h if is_drivable(h) => Mode::Car,
                _ => continue,
            };
            // A street cars may only enter for access (a pedestrian street
            // with a `destination` sign, say) isn't through traffic: it
            // neither makes a junction nor feeds a queue.
            let closed = |key: &str| {
                matches!(
                    way.tag(key),
                    Some("no" | "private" | "destination" | "delivery" | "customers")
                )
            };
            if way.tag("area") == Some("yes") || closed("access") || closed("motor_vehicle") {
                continue;
            }
            let nodes: Vec<NodeId> = way
                .refs
                .iter()
                .copied()
                .filter(|id| positions.contains_key(id))
                .collect();
            if nodes.len() < 2 {
                continue;
            }
            let points = nodes.iter().map(|id| positions[id]).collect();
            let (forward_lanes, backward_lanes) = lanes(highway, mode, |k| way.tag(k));
            let arrows = |lanes: u32, keys: &[&str]| {
                keys.iter()
                    .find_map(|k| way.tag(k))
                    .and_then(|value| lane_arrows(value, lanes))
            };
            let one_way = forward_lanes == 0 || backward_lanes == 0;
            let (forward_keys, backward_keys): (&[&str], &[&str]) = if one_way {
                (
                    &["turn:lanes", "turn:lanes:forward"],
                    &["turn:lanes", "turn:lanes:backward"],
                )
            } else {
                (&["turn:lanes:forward"], &["turn:lanes:backward"])
            };
            roads.push(Road {
                way: way.id,
                mode,
                is_link: highway.ends_with("_link"),
                nodes,
                points,
                forward_lanes,
                backward_lanes,
                forward_arrows: arrows(forward_lanes, forward_keys),
                backward_arrows: arrows(backward_lanes, backward_keys),
                speed_kmh: speed_kmh(way.tag("maxspeed")),
            });
        }

        let mut incidences: HashMap<NodeId, Vec<Incidence>> = HashMap::new();
        for (road, r) in roads.iter().enumerate() {
            for (index, node) in r.nodes.iter().enumerate() {
                incidences
                    .entry(*node)
                    .or_default()
                    .push(Incidence { road, index });
            }
        }
        Self {
            roads,
            positions,
            incidences,
        }
    }

    /// How many road *arms* leave `node`: 1 for a road ending there, 2 for
    /// one passing through. 3 or more is a junction.
    pub fn arms(&self, node: NodeId) -> usize {
        self.incidences.get(&node).map_or(0, |list| {
            list.iter()
                .map(|inc| {
                    let last = self.roads[inc.road].nodes.len() - 1;
                    if inc.index == 0 || inc.index == last {
                        1
                    } else {
                        2
                    }
                })
                .sum()
        })
    }

    pub fn is_junction(&self, node: NodeId) -> bool {
        self.arms(node) >= 3
    }

    /// The node index a vehicle travelling `dir` along `road` reaches after
    /// `index`, if any.
    pub fn next_index(&self, road: usize, dir: Dir, index: usize) -> Option<usize> {
        match dir {
            Dir::Forward => (index + 1 < self.roads[road].nodes.len()).then_some(index + 1),
            Dir::Backward => index.checked_sub(1),
        }
    }

    /// The node index a vehicle travelling `dir` along `road` came from
    /// before reaching `index`, if any.
    pub fn previous_index(&self, road: usize, dir: Dir, index: usize) -> Option<usize> {
        let opposite = match dir {
            Dir::Forward => Dir::Backward,
            Dir::Backward => Dir::Forward,
        };
        self.next_index(road, opposite, index)
    }

    /// Unit direction of `road` at node `index`, along its node order.
    pub fn tangent(&self, road: usize, index: usize) -> Option<Pt> {
        let points = &self.roads[road].points;
        let a = points[index.saturating_sub(1)];
        let b = points[(index + 1).min(points.len() - 1)];
        crate::geometry::unit([b[0] - a[0], b[1] - a[1]])
    }
}

/// Crossing nodes only a cycleway runs through (no footway, path or
/// sidewalk): a crossing for bicycles, which waiting zones leave out
/// altogether — it's no light for cars, no crosswalk to stop before, and
/// no place people wait.
pub fn bike_only_crossings(osm: &Osm) -> HashSet<NodeId> {
    let mut cycle: HashSet<NodeId> = HashSet::new();
    let mut foot: HashSet<NodeId> = HashSet::new();
    for way in &osm.ways {
        match way.tag("highway") {
            Some("cycleway") => cycle.extend(&way.refs),
            Some("footway" | "path" | "pedestrian" | "steps") => foot.extend(&way.refs),
            _ => {}
        }
    }
    cycle
        .into_iter()
        .filter(|n| !foot.contains(n))
        .filter(|n| {
            osm.nodes.get(n).is_some_and(|node| {
                node.tags.contains_key("crossing") || node.tag("highway") == Some("crossing")
            })
        })
        .collect()
}

/// Whether a node or way's tags mark a signalized pedestrian crossing.
pub fn is_signalized_crossing(crossing: Option<&str>, signals: Option<&str>) -> bool {
    crossing == Some("traffic_signals") || signals == Some("yes")
}

/// `turn:lanes` (`left|through;right|right`, listed left to right as the
/// driver sees them) as per-lane arrows indexed from the rightmost lane.
/// `None` when it doesn't list exactly `lanes` lanes.
fn lane_arrows(value: &str, lanes: u32) -> Option<Vec<LaneArrows>> {
    let mut per_lane: Vec<LaneArrows> = value
        .split('|')
        .map(|lane| {
            lane.split(';')
                .filter_map(|arrow| match arrow.trim() {
                    "through" => Some(Direction::Straight),
                    "left" | "slight_left" | "sharp_left" => Some(Direction::Left),
                    "right" | "slight_right" | "sharp_right" => Some(Direction::Right),
                    "reverse" => Some(Direction::UTurn),
                    _ => None,
                })
                .collect()
        })
        .collect();
    if per_lane.len() != lanes as usize {
        return None;
    }
    per_lane.reverse();
    Some(per_lane)
}

/// `maxspeed` in km/h; 50 (the Spanish urban limit) when untagged or not a
/// number.
fn speed_kmh(maxspeed: Option<&str>) -> f64 {
    let Some(value) = maxspeed else { return 50.0 };
    let number: String = value
        .chars()
        .take_while(|c| c.is_ascii_digit() || *c == '.')
        .collect();
    match number.parse::<f64>() {
        Ok(n) if value.contains("mph") => n * 1.609,
        Ok(n) => n,
        Err(_) => 50.0,
    }
}

fn is_drivable(highway: &str) -> bool {
    matches!(
        highway,
        "motorway"
            | "trunk"
            | "primary"
            | "secondary"
            | "tertiary"
            | "unclassified"
            | "residential"
            | "living_street"
            | "service"
            | "road"
            | "motorway_link"
            | "trunk_link"
            | "primary_link"
            | "secondary_link"
            | "tertiary_link"
    )
}

/// `(forward, backward)` lane counts from `oneway`/`lanes`/`lanes:*`, with
/// per-class defaults for the (common) untagged case.
fn lanes<'a>(highway: &str, mode: Mode, tag: impl Fn(&str) -> Option<&'a str>) -> (u32, u32) {
    let number = |key: &str| tag(key).and_then(|v| v.split(';').next()?.trim().parse::<u32>().ok());
    let oneway = match tag("oneway") {
        Some("yes" | "true" | "1") => Some(Dir::Forward),
        Some("-1" | "reverse") => Some(Dir::Backward),
        Some("no" | "false" | "0") => None,
        _ if matches!(tag("junction"), Some("roundabout" | "circular")) => Some(Dir::Forward),
        _ if highway == "motorway" || highway == "motorway_link" => Some(Dir::Forward),
        _ => None,
    };
    let default_one_way = match (mode, highway) {
        (Mode::Car, "motorway" | "trunk" | "primary" | "secondary") => 2,
        _ => 1,
    };
    match oneway {
        Some(dir) => {
            let n = number("lanes").unwrap_or(default_one_way).max(1);
            if dir == Dir::Forward { (n, 0) } else { (0, n) }
        }
        None => {
            let total = number("lanes");
            let forward = number("lanes:forward")
                .or(total.map(|t| t.div_ceil(2)))
                .unwrap_or(1)
                .max(1);
            let backward = number("lanes:backward")
                .or(total.map(|t| (t / 2).max(1)))
                .unwrap_or(1)
                .max(1);
            (forward, backward)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn lanes_of(highway: &str, tags: &[(&str, &str)]) -> (u32, u32) {
        let map: HashMap<&str, &str> = tags.iter().copied().collect();
        lanes(highway, Mode::Car, |k| map.get(k).copied())
    }

    #[test]
    fn turn_lanes_are_indexed_from_the_rightmost_lane() {
        let arrows = lane_arrows("left|through|through;right", 3).unwrap();
        assert_eq!(arrows[0], [Direction::Straight, Direction::Right].into());
        assert_eq!(arrows[1], [Direction::Straight].into());
        assert_eq!(arrows[2], [Direction::Left].into());
        assert!(lane_arrows("left|through", 3).is_none());
        assert!(
            lane_arrows("none|", 2)
                .unwrap()
                .iter()
                .all(BTreeSet::is_empty)
        );
    }

    #[test]
    fn maxspeed_parses_numbers_and_defaults_to_urban() {
        assert_eq!(speed_kmh(Some("30")), 30.0);
        assert_eq!(speed_kmh(Some("ES:urban")), 50.0);
        assert_eq!(speed_kmh(None), 50.0);
    }

    #[test]
    fn lane_counts_follow_oneway_and_lanes_tags() {
        assert_eq!(lanes_of("residential", &[]), (1, 1));
        assert_eq!(
            lanes_of("primary", &[("oneway", "yes"), ("lanes", "4")]),
            (4, 0)
        );
        assert_eq!(lanes_of("primary", &[("oneway", "-1")]), (0, 2));
        assert_eq!(lanes_of("tertiary", &[("lanes", "3")]), (2, 1));
        assert_eq!(
            lanes_of(
                "tertiary",
                &[
                    ("lanes", "3"),
                    ("lanes:forward", "1"),
                    ("lanes:backward", "2")
                ]
            ),
            (1, 2)
        );
        assert_eq!(
            lanes_of("residential", &[("junction", "roundabout")]),
            (1, 0)
        );
    }
}

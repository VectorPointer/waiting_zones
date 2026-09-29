//! The road network as a directed graph: every road split at its junctions
//! and signal nodes into edges, one per direction of travel.
//!
//! An edge is `{way}#{k}` in the way's own node order and `-{way}#{k}`
//! against it, `k` counting the way's segments. Neither contains `_`, which
//! zone ids use as the edge/movement separator.

use std::collections::{HashMap, HashSet};

use crate::geometry::{self, Pt};
use crate::network::{Dir, LaneArrows, Mode, Network};
use crate::osm::NodeId;

pub struct Edge {
    pub id: String,
    pub road: usize,
    pub mode: Mode,
    pub from: NodeId,
    pub to: NodeId,
    /// Node ids and positions in travel order.
    pub nodes: Vec<NodeId>,
    pub points: Vec<Pt>,
    pub length: f64,
    pub lanes: u32,
    /// Index 0 = rightmost lane.
    pub arrows: Option<Vec<LaneArrows>>,
    pub lane_width: f64,
    /// Where the rightmost lane's right edge sits, in metres to the right
    /// of the OSM centreline: a two-way road's own direction is its right
    /// half, a one-way road is centred on its line.
    right_edge: f64,
    pub speed_kmh: f64,
    /// The same segment travelled the other way, if the road allows it.
    pub reverse: Option<usize>,
}

impl Edge {
    /// How far right of the centreline lane `lane` runs (negative: left).
    pub fn lane_offset(&self, lane: u32) -> f64 {
        self.right_edge - (f64::from(lane) + 0.5) * self.lane_width
    }

    /// `lane`'s own centreline, in travel order.
    pub fn lane_line(&self, lane: u32) -> Vec<Pt> {
        geometry::offset(&self.points, self.lane_offset(lane.min(self.lanes - 1)))
    }
}

pub struct Graph {
    pub edges: Vec<Edge>,
    pub incoming: HashMap<NodeId, Vec<usize>>,
    pub outgoing: HashMap<NodeId, Vec<usize>>,
}

impl Graph {
    /// Splits every road of `net` at its junctions and at every node in
    /// `split_at`.
    pub fn build(net: &Network, split_at: &HashSet<NodeId>) -> Self {
        let mut edges = Vec::new();
        for (road_index, road) in net.roads.iter().enumerate() {
            let last = road.nodes.len() - 1;
            let cuts: Vec<usize> = (0..=last)
                .filter(|&i| {
                    i == 0
                        || i == last
                        || net.is_junction(road.nodes[i])
                        || split_at.contains(&road.nodes[i])
                })
                .collect();
            for (k, pair) in cuts.windows(2).enumerate() {
                let (a, b) = (pair[0], pair[1]);
                let forward = (road.lanes(Dir::Forward) > 0).then_some(edges.len());
                if forward.is_some() {
                    edges.push(edge(
                        net,
                        road_index,
                        Dir::Forward,
                        a,
                        b,
                        format!("{}#{k}", road.way),
                    ));
                }
                let backward = (road.lanes(Dir::Backward) > 0).then_some(edges.len());
                if backward.is_some() {
                    edges.push(edge(
                        net,
                        road_index,
                        Dir::Backward,
                        a,
                        b,
                        format!("-{}#{k}", road.way),
                    ));
                }
                if let (Some(f), Some(b)) = (forward, backward) {
                    edges[f].reverse = Some(b);
                    edges[b].reverse = Some(f);
                }
            }
        }
        let mut incoming: HashMap<NodeId, Vec<usize>> = HashMap::new();
        let mut outgoing: HashMap<NodeId, Vec<usize>> = HashMap::new();
        for (i, e) in edges.iter().enumerate() {
            outgoing.entry(e.from).or_default().push(i);
            incoming.entry(e.to).or_default().push(i);
        }
        Self {
            edges,
            incoming,
            outgoing,
        }
    }

    pub fn incoming(&self, node: NodeId) -> &[usize] {
        self.incoming.get(&node).map_or(&[], Vec::as_slice)
    }

    pub fn outgoing(&self, node: NodeId) -> &[usize] {
        self.outgoing.get(&node).map_or(&[], Vec::as_slice)
    }
}

fn edge(net: &Network, road_index: usize, dir: Dir, a: usize, b: usize, id: String) -> Edge {
    let road = &net.roads[road_index];
    let (mut nodes, mut points) = (road.nodes[a..=b].to_vec(), road.points[a..=b].to_vec());
    if dir == Dir::Backward {
        nodes.reverse();
        points.reverse();
    }
    let lanes = road.lanes(dir);
    let lane_width = road.mode.lane_width();
    let right_edge = if road.is_two_way() {
        f64::from(lanes) * lane_width
    } else {
        f64::from(lanes) * lane_width / 2.0
    };
    Edge {
        id,
        road: road_index,
        mode: road.mode,
        from: nodes[0],
        to: *nodes.last().expect("a segment has two nodes"),
        length: geometry::length(&points),
        nodes,
        points,
        lanes,
        arrows: road.arrows(dir).cloned(),
        lane_width,
        right_edge,
        speed_kmh: road.speed_kmh,
        reverse: None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::osm::read_from;
    use crate::projection::Projection;

    #[test]
    fn roads_split_at_junctions_into_directed_edges() {
        let osm = read_from(
            r#"<osm>
            <node id="1" lat="41.4000" lon="2.1990"/>
            <node id="2" lat="41.4000" lon="2.2000"/>
            <node id="3" lat="41.4000" lon="2.2010"/>
            <node id="4" lat="41.4010" lon="2.2000"/>
            <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="primary"/></way>
            <way id="20"><nd ref="4"/><nd ref="2"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>
            </osm>"#
                .as_bytes(),
        )
        .unwrap();
        let projection = Projection::centred_on(osm.nodes.values().map(|n| n.lon_lat));
        let net = Network::build(&osm, &projection);
        let graph = Graph::build(&net, &HashSet::new());
        let mut ids: Vec<&str> = graph.edges.iter().map(|e| e.id.as_str()).collect();
        ids.sort_unstable();
        assert_eq!(ids, vec!["-10#0", "-10#1", "10#0", "10#1", "20#0"]);
        let into_2: Vec<&str> = graph
            .incoming(2)
            .iter()
            .map(|&e| graph.edges[e].id.as_str())
            .collect();
        assert_eq!(into_2.len(), 3);
        // A two-way road's own lane sits right of the centreline.
        let eastbound = graph.edges.iter().find(|e| e.id == "10#0").unwrap();
        assert!(eastbound.lane_offset(0) > 0.0);
    }
}

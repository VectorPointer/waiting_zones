//! Signalized junctions: which OSM nodes one traffic-light controller
//! covers, which edges enter it (approaches) and which leave it (exits).
//!
//! OSM rarely puts the signal on the junction node itself: it's usually a
//! `highway=traffic_signals` node on each approach, or a signalized
//! crosswalk just before the junction, and a dual carriageway meets a road
//! at two junction nodes. All of those belong to one controller, so they
//! are joined into one cluster when a short edge links them.

use std::collections::{BTreeSet, HashMap, HashSet};

use crate::geometry::{self, Pt};
use crate::graph::Graph;
use crate::network::{Mode, Network, is_signalized_crossing};
use crate::osm::{NodeId, Osm};

/// Two control nodes linked by a stretch of road shorter than this are one
/// junction.
const JOIN_METERS: f64 = 30.0;
/// A cluster wider than this is two junctions a chain of short edges has
/// wrongly glued together.
const MAX_CLUSTER_DIAMETER_METERS: f64 = 80.0;
/// Half a zebra: the stop line sits at the crosswalk's own edge.
const STOP_BEFORE_CROSSWALK_METERS: f64 = 2.0;
/// How far back from a junction a crosswalk may sit and still be the one
/// its approach stops in front of.
const CROSSWALK_SEARCH_METERS: f64 = 25.0;

/// Every node a vehicle stops at a signal for: a traffic light, or a
/// signalized pedestrian crossing on a road.
pub fn signal_nodes(osm: &Osm, net: &Network) -> HashSet<NodeId> {
    let bike_only = crate::network::bike_only_crossings(osm);
    osm.nodes
        .values()
        .filter(|n| net.incidences.contains_key(&n.id))
        .filter(|n| {
            n.tag("highway") == Some("traffic_signals")
                || (is_signalized_crossing(n.tag("crossing"), n.tag("crossing:signals"))
                    && !bike_only.contains(&n.id))
        })
        .map(|n| n.id)
        .collect()
}

pub struct Cluster {
    /// The controller's id: the cluster's smallest node id.
    pub tls_id: String,
    pub nodes: BTreeSet<NodeId>,
    pub centre: Pt,
    /// Edges entering the cluster from outside.
    pub approaches: Vec<usize>,
    /// Edges leaving the cluster.
    pub exits: Vec<usize>,
}

impl Cluster {
    pub fn contains(&self, node: NodeId) -> bool {
        self.nodes.contains(&node)
    }
}

pub fn build(osm: &Osm, net: &Network, graph: &Graph, signals: &HashSet<NodeId>) -> Vec<Cluster> {
    let is_light = |n: NodeId| osm.nodes[&n].tag("highway") == Some("traffic_signals");
    // A node where three or more car arms meet: a junction of its own,
    // never walked through to link two others.
    let car_junction = |n: NodeId| {
        graph
            .outgoing(n)
            .iter()
            .map(|&e| (graph.edges[e].to, graph.edges[e].mode))
            .chain(
                graph
                    .incoming(n)
                    .iter()
                    .map(|&e| (graph.edges[e].from, graph.edges[e].mode)),
            )
            .filter(|&(_, mode)| mode == Mode::Car)
            .map(|(other, _)| other)
            .collect::<HashSet<_>>()
            .len()
            >= 3
    };
    // The nodes `stop` accepts within [`JOIN_METERS`] of `n` along the
    // road — following traffic (`forward`) or against it — each with the
    // nodes passed on the way. The way may pass nodes that are only where a
    // cycleway or a path meets the road (Barcelona's Carrer de Tortosa, a
    // bike lane touching it between its crosswalk and the junction), never
    // another car junction.
    let reach = |n: NodeId, forward: bool, stop: &dyn Fn(NodeId) -> bool| {
        let mut found: Vec<(NodeId, Vec<NodeId>)> = Vec::new();
        let mut seen = HashSet::from([n]);
        let mut stack = vec![(n, 0.0, Vec::new())];
        while let Some((at, travelled, via)) = stack.pop() {
            let edges = if forward {
                graph.outgoing(at)
            } else {
                graph.incoming(at)
            };
            for &e in edges {
                let edge = &graph.edges[e];
                let next = if forward { edge.to } else { edge.from };
                let total = travelled + edge.length;
                if total > JOIN_METERS || !seen.insert(next) {
                    continue;
                }
                if stop(next) {
                    found.push((next, via.clone()));
                } else if !car_junction(next) {
                    let mut via = via.clone();
                    via.push(next);
                    stack.push((next, total, via));
                }
            }
        }
        found
    };
    let neighbours = |n: NodeId, stop: &dyn Fn(NodeId) -> bool| {
        let mut both = reach(n, true, stop);
        both.extend(reach(n, false, stop));
        both
    };

    // Whether traffic meets `light` on its way into `node`: the road runs
    // from the light to the node. A light standing on a one-way road just
    // *after* a junction (Barcelona's Rotonda de l'Alguer, the light on its
    // Avinguda de Pomar exit) stops cars that have already left the
    // junction — it guards the crosswalk beyond it, and says nothing about
    // the junction itself.
    let feeds = |light: NodeId, node: NodeId| {
        reach(light, true, &|n| n == node || signals.contains(&n))
            .iter()
            .any(|(n, _)| *n == node)
    };

    // A junction is controlled when it carries the light itself, when a
    // light stands on one of its arms before it, or when two of its arms
    // have a signalized crosswalk — one alone is a mid-block crossing that
    // happens to sit near an ordinary junction.
    let mut control: BTreeSet<NodeId> = signals.iter().copied().collect();
    for &node in net.incidences.keys() {
        if !car_junction(node) || control.contains(&node) {
            continue;
        }
        let near: Vec<NodeId> = neighbours(node, &|n| signals.contains(&n))
            .into_iter()
            .map(|(n, _)| n)
            .collect();
        if near.iter().any(|&n| is_light(n) && feeds(n, node))
            || near.iter().collect::<HashSet<_>>().len() >= 2
        {
            control.insert(node);
        }
    }

    // Union-find over control nodes linked by a short stretch, refusing any
    // merge that would make a cluster implausibly wide.
    let nodes: Vec<NodeId> = control.iter().copied().collect();
    let index: HashMap<NodeId, usize> = nodes.iter().enumerate().map(|(i, &n)| (n, i)).collect();
    let mut members: Vec<Vec<NodeId>> = nodes.iter().map(|&n| vec![n]).collect();
    // Nodes passed on the way between two joined control nodes: inside the
    // controller too, or the stretch between them would be an approach.
    let mut passed: Vec<Vec<NodeId>> = vec![Vec::new(); nodes.len()];
    let mut owner: Vec<usize> = (0..nodes.len()).collect();
    for (i, &node) in nodes.iter().enumerate() {
        for (other, via) in neighbours(node, &|n| control.contains(&n)) {
            let Some(&j) = index.get(&other) else {
                continue;
            };
            // A light joins a junction only if traffic meets it first.
            let junction = |n: NodeId| car_junction(n) && !is_light(n);
            if (is_light(node) && junction(other) && !feeds(node, other))
                || (is_light(other) && junction(node) && !feeds(other, node))
            {
                continue;
            }
            let (a, b) = (owner[i], owner[j]);
            if a == b {
                continue;
            }
            let diameter = members[a]
                .iter()
                .flat_map(|x| members[b].iter().map(move |y| (x, y)))
                .map(|(x, y)| geometry::dist(net.positions[x], net.positions[y]))
                .fold(0.0, f64::max);
            if diameter > MAX_CLUSTER_DIAMETER_METERS {
                continue;
            }
            let moved = std::mem::take(&mut members[b]);
            for &n in &moved {
                owner[index[&n]] = a;
            }
            members[a].extend(moved);
            let moved = std::mem::take(&mut passed[b]);
            passed[a].extend(moved);
            passed[a].extend(via);
        }
    }
    for (m, p) in members.iter_mut().zip(passed) {
        if !m.is_empty() {
            m.extend(p);
        }
    }

    let mut clusters: Vec<Cluster> = members
        .into_iter()
        .filter(|m| !m.is_empty())
        .map(|m| {
            let nodes: BTreeSet<NodeId> = m.into_iter().collect();
            let centre = {
                let n = nodes.len() as f64;
                let sum = nodes.iter().fold([0.0, 0.0], |acc, id| {
                    let p = net.positions[id];
                    [acc[0] + p[0], acc[1] + p[1]]
                });
                [sum[0] / n, sum[1] / n]
            };
            let approaches = (0..graph.edges.len())
                .filter(|&e| {
                    nodes.contains(&graph.edges[e].to) && !nodes.contains(&graph.edges[e].from)
                })
                .collect();
            let exits = (0..graph.edges.len())
                .filter(|&e| {
                    nodes.contains(&graph.edges[e].from) && !nodes.contains(&graph.edges[e].to)
                })
                .collect();
            Cluster {
                tls_id: nodes.first().expect("non-empty").to_string(),
                nodes,
                centre,
                approaches,
                exits,
            }
        })
        .filter(|c: &Cluster| {
            c.approaches
                .iter()
                .any(|&e| graph.edges[e].mode == Mode::Car)
        })
        .collect();
    clusters.sort_by(|a, b| a.tls_id.cmp(&b.tls_id));
    clusters
}

/// How far before the end of approach `edge` its vehicles stop.
pub fn stop_setback(
    osm: &Osm,
    net: &Network,
    graph: &Graph,
    crossing_nodes: &HashSet<NodeId>,
    edge: usize,
) -> f64 {
    let e = &graph.edges[edge];
    let end = osm.nodes.get(&e.to);
    let tag = |k: &str| end.and_then(|n| n.tag(k));
    if is_signalized_crossing(tag("crossing"), tag("crossing:signals"))
        || tag("highway") == Some("crossing")
    {
        return STOP_BEFORE_CROSSWALK_METERS;
    }
    if tag("highway") == Some("traffic_signals") && !net.is_junction(e.to) {
        return 0.0;
    }
    // A junction: stop in front of the crosswalk guarding it, when mapped.
    let mut travelled = 0.0;
    for w in e.nodes.windows(2).rev() {
        travelled += geometry::dist(net.positions[&w[0]], net.positions[&w[1]]);
        if travelled > CROSSWALK_SEARCH_METERS {
            break;
        }
        if crossing_nodes.contains(&w[0]) {
            return travelled + STOP_BEFORE_CROSSWALK_METERS;
        }
    }
    let widest_other = graph
        .outgoing(e.to)
        .iter()
        .filter(|&&x| Some(x) != e.reverse)
        .map(|&x| {
            let road = &net.roads[graph.edges[x].road];
            road.width()
        })
        .fold(0.0, f64::max);
    (widest_other / 2.0 + STOP_BEFORE_CROSSWALK_METERS).clamp(3.0, 15.0)
}

/// Nodes where a pedestrian crosses a road: tagged as a crossing, or on a
/// `footway=crossing` way — not a crossing only a cycleway runs through.
pub fn crossing_nodes(osm: &Osm) -> HashSet<NodeId> {
    let mut nodes: HashSet<NodeId> = osm
        .nodes
        .values()
        .filter(|n| n.tag("highway") == Some("crossing") || n.tags.contains_key("crossing"))
        .map(|n| n.id)
        .collect();
    for way in &osm.ways {
        if way.tag("footway") == Some("crossing") {
            nodes.extend(&way.refs);
        }
    }
    for bike in crate::network::bike_only_crossings(osm) {
        nodes.remove(&bike);
    }
    nodes
}

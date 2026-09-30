//! Serialise the generator's own road graph into the split files a
//! simulator reads, with no SUMO vocabulary.
//!
//! Three files, each self-describing:
//!
//! - `network.edges.json`: every directed edge — its lanes, geometry, speed
//!   and the edges that follow it.
//! - `network.junctions.json`: every junction node — its kind, its signal
//!   (if any) and its incident edges. References `network.edges.json`.
//! - `network.connections.json`: every movement through a signalised
//!   junction, with the approach lanes that serve it. References both.
//!
//! Every file carries a `source_digest` (the run's own inputs) and, in
//! `references`, the SHA-256 of the bytes of each file it names. A reader
//! that loads one file and then a referenced one checks that hash first: a
//! mixed set (one file left over from an older run) is detected instead of
//! silently used, the same way a lockfile pins its tree.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::{Context, Result};
use serde::Serialize;
use sha2::{Digest, Sha256};

use crate::clusters::Cluster;
use crate::graph::Graph;
use crate::movements::Junction as Movements;
use crate::network::{Direction, Network};
use crate::osm::NodeId;

/// Bump when a change here would make an older file wrong.
pub const FORMAT_VERSION: u32 = 1;

pub const EDGES_FILE: &str = "network.edges.json";
pub const JUNCTIONS_FILE: &str = "network.junctions.json";
pub const CONNECTIONS_FILE: &str = "network.connections.json";

/// Which file names a reader must co-load, and their content hashes.
type References = BTreeMap<String, String>;

/// SHA-256 of a byte slice, hex.
pub fn digest(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    format!("{:x}", hasher.finalize())
}

/// The run's own digest: the `.osm` bytes plus the reach options, so two
/// runs of the same extract with different options are told apart.
pub fn run_digest(osm: &Path, reach: crate::zones::Reach) -> Result<String> {
    let bytes = std::fs::read(osm).with_context(|| format!("reading {}", osm.display()))?;
    let mut hasher = Sha256::new();
    hasher.update(&bytes);
    hasher.update(format!("max_length={:?};", reach.max_length).as_bytes());
    Ok(format!("{:x}", hasher.finalize()))
}

/// Serialize `value` and write it to `out_dir/name`, returning the file's
/// own content hash.
fn write_json<T: Serialize>(out_dir: &Path, name: &str, value: &T) -> Result<String> {
    let mut bytes = serde_json::to_vec_pretty(value).context("serializing")?;
    bytes.push(b'\n');
    let path = out_dir.join(name);
    std::fs::write(&path, &bytes).with_context(|| format!("writing {}", path.display()))?;
    Ok(digest(&bytes))
}

#[derive(Serialize)]
struct EdgeFile {
    format_version: u32,
    territory: String,
    source_digest: String,
    references: References,
    edges: Vec<EdgeRecord>,
}

#[derive(Serialize)]
struct EdgeRecord {
    id: String,
    way: i64,
    from: String,
    to: String,
    /// Centreline, local metres.
    points: Vec<[f64; 2]>,
    length_meters: f64,
    speed_kmh: f64,
    reverse: Option<String>,
    lanes: Vec<LaneRecord>,
    /// The edges that follow this one at `to`: every outgoing edge there,
    /// except this one's own reverse.
    successors: Vec<String>,
}

#[derive(Serialize)]
struct LaneRecord {
    /// 0 = rightmost.
    index: u32,
    width: f64,
    /// Offset from the centreline, metres, positive to the right.
    offset: f64,
    /// Painted turn arrows, empty when OSM maps none.
    arrows: Vec<String>,
}

#[derive(Serialize)]
struct JunctionFile {
    format_version: u32,
    territory: String,
    source_digest: String,
    references: References,
    junctions: Vec<JunctionRecord>,
}

#[derive(Serialize)]
struct JunctionRecord {
    id: String,
    /// `traffic_light`, `junction`, or `node` (a two-arm pass-through).
    kind: String,
    /// The controller's id when `kind` is `traffic_light`.
    traffic_light: Option<String>,
    position: [f64; 2],
    incoming: Vec<String>,
    outgoing: Vec<String>,
}

#[derive(Serialize)]
struct ConnectionFile {
    format_version: u32,
    territory: String,
    source_digest: String,
    references: References,
    connections: Vec<ConnectionRecord>,
}

/// One movement through a signalised junction. The matching signal state is
/// in `programs.json`'s own `links` (same `from_edge`/`from_lane`/`to_edge`/
/// `direction`), so it isn't duplicated here.
#[derive(Serialize)]
struct ConnectionRecord {
    junction: String,
    from_edge: String,
    from_lane: u32,
    to_edge: String,
    direction: String,
    /// Nodes the movement passes through inside the junction.
    via: Vec<String>,
}

/// Write the three network files into `out_dir`.
pub fn write(
    net: &Network,
    graph: &Graph,
    clusters: &[Cluster],
    movements: &[Movements],
    out_dir: &Path,
    territory: &str,
    source_digest: &str,
) -> Result<()> {
    std::fs::create_dir_all(out_dir).with_context(|| format!("creating {}", out_dir.display()))?;

    let edge_id = |index: usize| graph.edges[index].id.clone();

    let edges: Vec<EdgeRecord> = (0..graph.edges.len())
        .map(|index| {
            let edge = &graph.edges[index];
            let lanes = (0..edge.lanes)
                .map(|lane| LaneRecord {
                    index: lane,
                    width: edge.lane_width,
                    offset: edge.lane_offset(lane),
                    arrows: edge
                        .arrows
                        .as_ref()
                        .and_then(|arrows| arrows.get(lane as usize))
                        .map(|arrows| arrows.iter().map(|d| d.label().to_string()).collect())
                        .unwrap_or_default(),
                })
                .collect();
            let successors = graph
                .outgoing(edge.to)
                .iter()
                .filter(|&&successor| Some(successor) != edge.reverse)
                .map(|&successor| edge_id(successor))
                .collect();
            EdgeRecord {
                id: edge.id.clone(),
                way: net.roads[edge.road].way,
                from: edge.from.to_string(),
                to: edge.to.to_string(),
                points: edge.points.clone(),
                length_meters: edge.length,
                speed_kmh: edge.speed_kmh,
                reverse: edge.reverse.map(edge_id),
                lanes,
                successors,
            }
        })
        .collect();

    let tls_by_node: BTreeMap<NodeId, &str> = clusters
        .iter()
        .flat_map(|cluster| {
            cluster
                .nodes
                .iter()
                .map(move |node| (*node, cluster.tls_id.as_str()))
        })
        .collect();
    let mut nodes: BTreeSet<NodeId> = BTreeSet::new();
    for edge in &graph.edges {
        nodes.insert(edge.from);
        nodes.insert(edge.to);
    }
    let junctions: Vec<JunctionRecord> = nodes
        .iter()
        .map(|&node| {
            let kind = match tls_by_node.get(&node) {
                Some(_) => "traffic_light",
                None if net.is_junction(node) => "junction",
                None => "node",
            };
            JunctionRecord {
                id: node.to_string(),
                kind: kind.to_string(),
                traffic_light: tls_by_node.get(&node).map(|tls| tls.to_string()),
                position: net.positions[&node],
                incoming: graph.incoming(node).iter().map(|&e| edge_id(e)).collect(),
                outgoing: graph.outgoing(node).iter().map(|&e| edge_id(e)).collect(),
            }
        })
        .collect();

    // Movements are derived per signalised junction (the only place the
    // generator needs them today); the sim uses each edge's own
    // `successors` everywhere else.
    let mut connections: Vec<ConnectionRecord> = Vec::new();
    for junction in movements {
        for (index, movement) in junction.movements.iter().enumerate() {
            let approach = junction
                .approaches
                .iter()
                .find(|approach| approach.movements.contains(&index));
            let Some(approach) = approach else {
                continue;
            };
            let lanes = approach
                .lane_directions
                .iter()
                .enumerate()
                .filter(|(_, turns)| turns.contains(&movement.direction))
                .map(|(lane, _)| lane as u32);
            for lane in lanes {
                connections.push(ConnectionRecord {
                    junction: graph.edges[movement.approach].from.to_string(),
                    from_edge: edge_id(movement.approach),
                    from_lane: lane,
                    to_edge: edge_id(movement.exit),
                    direction: direction_label(movement.direction).to_string(),
                    via: movement.via.iter().map(|n| n.to_string()).collect(),
                });
            }
        }
    }

    // Edges first (the root), then the files that reference it, so each
    // hash is available before it is embedded.
    let edges_hash = write_json(
        out_dir,
        EDGES_FILE,
        &EdgeFile {
            format_version: FORMAT_VERSION,
            territory: territory.to_string(),
            source_digest: source_digest.to_string(),
            references: References::new(),
            edges,
        },
    )?;
    let mut junction_refs = References::new();
    junction_refs.insert(EDGES_FILE.to_string(), edges_hash.clone());
    let junctions_hash = write_json(
        out_dir,
        JUNCTIONS_FILE,
        &JunctionFile {
            format_version: FORMAT_VERSION,
            territory: territory.to_string(),
            source_digest: source_digest.to_string(),
            references: junction_refs,
            junctions,
        },
    )?;
    let mut connection_refs = References::new();
    connection_refs.insert(EDGES_FILE.to_string(), edges_hash);
    connection_refs.insert(JUNCTIONS_FILE.to_string(), junctions_hash);
    write_json(
        out_dir,
        CONNECTIONS_FILE,
        &ConnectionFile {
            format_version: FORMAT_VERSION,
            territory: territory.to_string(),
            source_digest: source_digest.to_string(),
            references: connection_refs,
            connections,
        },
    )?;
    Ok(())
}

/// The generator's own direction label (not the neutral API's), kept as-is
/// so `programs.json`'s links and this file name the same movement.
fn direction_label(direction: Direction) -> &'static str {
    direction.label()
}

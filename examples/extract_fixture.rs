//! Extracts a small, self-contained subset of a real `.net.xml` around one
//! junction — for building test fixtures out of *real* `netconvert` output,
//! not a hand-built synthetic `Network`. The messiest defects this crate
//! has actually hit only show up in real output's own quirks — a `joinTLS`
//! program spanning 15 junctions, a 0.2m connector edge, a via lane's own
//! sharply curved shape — none of which anyone would think to reproduce by
//! hand; a real fixture this small can carry all of that faithfully and
//! still be readable.
//!
//! Usage:
//! ```sh
//! cargo run --release --example extract_fixture -- \
//!   <input.net.xml> <junction-id> <output-dir> [radius-meters]
//! ```
//! Writes `<output-dir>/network.net.xml`. `radius-meters` defaults to 150 —
//! comfortably past `zone_generator::DEFAULT_EXTENSION_METERS`'s own 120m
//! extension budget, so a zone at `junction-id` whose own chain never
//! reaches past this radius gets byte-identical output from the extract as
//! from the full network (verify this for a new fixture the same way
//! `tests/fixtures/README.md` describes: diff the target zone's own feature
//! between a run against the full network and a run against the extract).
//!
//! Keeps every edge within that real (not hop-count) walking distance of
//! `junction-id` along the connection graph, in *both* directions —
//! `zone_generator::extended_entry_lanes` only ever walks backward, but a
//! forward neighbour can still be the `to_edge` a connection needs resolved,
//! or belong to a junction this extract also needs complete. And for every
//! junction any collected edge touches, this keeps *every* edge that
//! junction has — not only the ones this specific walk happened to reach —
//! because `zone_generator` needs a junction's real incoming/outgoing lanes
//! in full to compute correct successor counts and movement groups; a
//! partially-present junction can silently produce different output than
//! the real one, not just less of it.

use anyhow::{Context, Result, bail};
use std::collections::{HashMap, HashSet, VecDeque};
use std::path::Path;
use sumo_types::domain::{Edge, EdgeId, Junction, JunctionId, LaneId, TrafficLightId};

const DEFAULT_RADIUS_METERS: f64 = 150.0;

fn edge_length_meters(edge: &Edge) -> f64 {
    edge.length
        .map(|l| l.get::<sumo_types::uom::si::length::meter>())
        .or_else(|| edge.lanes.first().map(|lane| lane.length.get::<sumo_types::uom::si::length::meter>()))
        .unwrap_or(0.0)
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().collect();
    let (input, junction_id, output_dir, radius) = match args.as_slice() {
        [_, input, junction_id, output_dir] => (input, junction_id, output_dir, DEFAULT_RADIUS_METERS),
        [_, input, junction_id, output_dir, radius] => {
            (input, junction_id, output_dir, radius.parse().context("radius-meters must be a number")?)
        }
        _ => bail!("usage: extract_fixture <input.net.xml> <junction-id> <output-dir> [radius-meters]"),
    };
    let target = JunctionId(junction_id.clone());

    let network = sumo_types::read_network(Path::new(input))?;
    if !network.junctions.iter().any(|j| j.id == target) {
        bail!("junction {junction_id:?} not found in {input:?}");
    }

    let edge_by_id: HashMap<&EdgeId, &Edge> = network.edges.iter().map(|edge| (&edge.id, edge)).collect();

    // Every edge whose own `from` or `to` is `target` — the walk's own seed.
    let seeds: Vec<&EdgeId> = network
        .edges
        .iter()
        .filter(|edge| edge.from.as_ref() == Some(&target) || edge.to.as_ref() == Some(&target))
        .map(|edge| &edge.id)
        .collect();
    if seeds.is_empty() {
        bail!("junction {junction_id:?} has no edges touching it in {input:?}");
    }

    // Real-distance-bounded flood fill over the connection graph (both
    // directions — see this file's own module docs on why): not an exact
    // shortest-path search, just "how far from `target`, walking edge by
    // edge, can you get without exceeding `radius`" — enough to decide
    // what's "close enough" to include.
    let mut distance: HashMap<&EdgeId, f64> = seeds.iter().map(|&id| (id, 0.0)).collect();
    let mut frontier: VecDeque<&EdgeId> = seeds.into_iter().collect();
    while let Some(edge_id) = frontier.pop_front() {
        let here = distance[edge_id];
        let Some(&edge) = edge_by_id.get(edge_id) else { continue };
        let step = here + edge_length_meters(edge);
        if step > radius {
            continue;
        }
        for connection in &network.connections {
            let neighbor = if &connection.from_edge == edge_id {
                Some(&connection.to_edge)
            } else if &connection.to_edge == edge_id {
                Some(&connection.from_edge)
            } else {
                None
            };
            let Some(neighbor) = neighbor else { continue };
            if distance.get(neighbor).is_none_or(|&known| step < known) {
                distance.insert(neighbor, step);
                frontier.push_back(neighbor);
            }
        }
    }
    let mut visited_edges: HashSet<&EdgeId> = distance.keys().copied().collect();

    // A real-to-real connection that survives into the fixture (both its
    // `from_edge`/`to_edge` already in `visited_edges`) can name an
    // internal lane as its own `via` — the actual curved path SUMO's own
    // connection takes through the junction's interior — and that lane's
    // own internal edge can in turn be the `from_edge` of a *further*
    // connection whose own `via` chains into yet another internal edge (a
    // multi-stage turn, e.g. a u-turn split across two internal edges:
    // confirmed on real Barcelona data, junction `303587355`, where
    // `:303587355_8`'s own successor connection immediately vias into
    // `:303587355_11`). Deliberately not sourced from the junction's own
    // `intLanes` attribute — confirmed empirically that it's *not* a
    // reliable list of every internal lane a real connection actually
    // uses as `via` (`:303587355_8_0` is one, and it's simply absent from
    // junction `303587355`'s own `intLanes` in real Barcelona data) — so
    // this walks the connection graph's own `via` edges directly instead,
    // to a fixed point, unbounded by `radius`: a via edge is junction-
    // interior geometry with a real but tiny length, not a further hop
    // away from `target` the way a real successor edge is.
    let lane_to_edge: HashMap<&LaneId, &EdgeId> =
        network.edges.iter().flat_map(|edge| edge.lanes.iter().map(move |lane| (&lane.id, &edge.id))).collect();
    loop {
        let mut grew = false;
        for connection in &network.connections {
            if !visited_edges.contains(&connection.from_edge) || !visited_edges.contains(&connection.to_edge) {
                continue;
            }
            let Some(via_lane) = &connection.via else { continue };
            let Some(&via_edge) = lane_to_edge.get(via_lane) else { continue };
            if visited_edges.insert(via_edge) {
                grew = true;
            }
        }
        if !grew {
            break;
        }
    }

    let edges: Vec<Edge> = network.edges.iter().filter(|edge| visited_edges.contains(&edge.id)).cloned().collect();

    // Every junction any collected edge actually touches — not just
    // `target` — so each one's own connectivity is complete (see this
    // file's own module docs on why a partial junction is a correctness
    // risk, not just an incompleteness one).
    let junction_ids: HashSet<&JunctionId> =
        edges.iter().flat_map(|edge| [edge.from.as_ref(), edge.to.as_ref()]).flatten().collect();
    let junctions: Vec<Junction> =
        network.junctions.iter().filter(|junction| junction_ids.contains(&junction.id)).cloned().collect();

    let connections: Vec<_> = network
        .connections
        .iter()
        .filter(|connection| visited_edges.contains(&connection.from_edge) && visited_edges.contains(&connection.to_edge))
        .cloned()
        .collect();

    let traffic_light_ids: HashSet<&TrafficLightId> =
        connections.iter().filter_map(|connection| connection.traffic_light.as_ref()).collect();
    let traffic_light_programs: Vec<_> = network
        .traffic_light_programs
        .iter()
        .filter(|program| traffic_light_ids.contains(&program.id))
        .cloned()
        .collect();

    let extracted = sumo_types::domain::Network {
        location: network.location.clone(),
        edges,
        junctions,
        connections,
        // Never consulted by `zone_generator` — dropped rather than
        // cross-referenced against `visited_edges`, which would need to
        // partially-include or exclude a roundabout this radius only
        // catches part of.
        roundabouts: Vec::new(),
        traffic_light_programs,
    };

    std::fs::create_dir_all(output_dir)
        .with_context(|| format!("could not create output directory: {output_dir:?}"))?;
    let output_path = Path::new(output_dir).join("network.net.xml");
    sumo_types::write_network(&output_path, &extracted)
        .with_context(|| format!("could not write extracted network: {output_path:?}"))?;
    println!(
        "Extracted {} edges / {} junctions / {} connections around {junction_id:?} to {output_path:?}",
        extracted.edges.len(),
        extracted.junctions.len(),
        extracted.connections.len()
    );
    Ok(())
}

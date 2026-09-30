//! Experimental waiting-zone generator that works from an OpenStreetMap
//! extract alone — no SUMO network. It rebuilds, in its own terms, the
//! slice of what `netconvert` would derive that waiting zones need:
//!
//! - [`graph`]: roads split into directed edges with lanes and arrows;
//! - [`clusters`]: signalized junctions (one controller each);
//! - [`movements`]: where each approach goes and which lanes serve it;
//! - [`zones`]: one vehicle zone per group of lanes with the same turns;
//! - [`program`]: the signal cycle, and which zones each phase serves;
//! - [`pedestrians`]: one zone per signalized crossing;
//! - [`output`]: overlap splitting, GeoJSON and the programs file.

pub mod clusters;
pub mod geometry;
pub mod graph;
pub mod movements;
pub mod network;
pub mod network_output;
pub mod osm;
pub mod output;
pub mod pedestrians;
pub mod program;
pub mod projection;
pub mod zones;

use std::collections::{HashMap, HashSet};
use std::path::Path;

use anyhow::Result;

use crate::network::Network;
use crate::osm::NodeId;
use crate::output::{JunctionProgram, ProgramLink, ProgramZone, Transition, Zone};
use crate::program::{Crossing, ZoneRef};
use crate::projection::Projection;
use crate::zones::Reach;

pub struct Summary {
    pub vehicle_zones: usize,
    pub pedestrian_zones: usize,
    pub junctions: usize,
    /// Pedestrian zones no controller claimed (their crossing isn't at a
    /// signalized junction this generator found).
    pub unassigned_pedestrian_zones: usize,
}

pub struct Generated {
    pub zones: Vec<Zone>,
    pub programs: Vec<JunctionProgram>,
    pub projection: Projection,
    pub unassigned_pedestrian_zones: usize,
    /// Movement-level plans, for invariant checks.
    pub plans: Vec<program::Plan>,
}

/// One junction's inputs, held until every zone is final: `assemble` can
/// drop a zone, and a junction's cycle must be built only from the zones
/// that survive.
struct Pending<'a> {
    cluster: &'a clusters::Cluster,
    junction: movements::Junction,
    setbacks: HashMap<usize, f64>,
    refs: Vec<ZoneRef>,
    crossings: Vec<Crossing>,
    program_zones: Vec<ProgramZone>,
}

pub fn generate(osm: &osm::Osm, reach: Reach) -> Generated {
    let projection = Projection::centred_on(osm.nodes.values().map(|n| n.lon_lat));
    let net = Network::build(osm, &projection);
    let signals = clusters::signal_nodes(osm, &net);
    let graph = graph::Graph::build(&net, &signals);
    let clusters = clusters::build(osm, &net, &graph, &signals);
    let controlled: HashSet<NodeId> = clusters
        .iter()
        .flat_map(|c| c.nodes.iter().copied())
        .collect();
    let crossing_nodes = clusters::crossing_nodes(osm);

    let pedestrian_zones = pedestrians::generate(osm, &net);
    let mut pedestrian_owner: HashMap<String, String> = HashMap::new();

    let mut zones = Vec::new();
    let mut pending: Vec<Pending> = Vec::new();
    for cluster in &clusters {
        let junction = movements::build(osm, &net, &graph, cluster);
        if junction.movements.is_empty() {
            continue;
        }
        let setbacks: HashMap<usize, f64> = junction
            .approaches
            .iter()
            .map(|a| {
                (
                    a.edge,
                    clusters::stop_setback(osm, &net, &graph, &crossing_nodes, a.edge),
                )
            })
            .collect();

        let mut refs = Vec::new();
        let mut program_zones = Vec::new();
        for approach in &junction.approaches {
            for (directions, lanes) in approach.groups() {
                let Some(zone) = zones::build(
                    &net,
                    &graph,
                    &controlled,
                    approach.edge,
                    directions.clone(),
                    lanes.clone(),
                    setbacks[&approach.edge],
                    reach,
                ) else {
                    continue;
                };
                refs.push(ZoneRef {
                    approach: approach.edge,
                    directions,
                    lanes: lanes.clone(),
                    id: zone.id.clone(),
                });
                program_zones.push(ProgramZone {
                    detector_id: zone.id.clone(),
                    phases: Vec::new(),
                    lanes: lanes.len() as u32,
                    length_meters: zone.length_meters,
                    edge: graph.edges[approach.edge].id.clone(),
                    is_pedestrian: false,
                    situational: false,
                });
                zones.push(Zone::vehicle(zone, cluster.tls_id.clone()));
            }
        }

        let crossings: Vec<Crossing> = pedestrian_zones
            .iter()
            .filter(|p| p.signalized && cluster.contains(p.anchor))
            .map(|p| Crossing {
                zone_id: p.id.clone(),
                anchor: p.anchor,
                line: p.crosswalk.clone(),
            })
            .collect();
        for p in pedestrian_zones
            .iter()
            .filter(|p| p.signalized && cluster.contains(p.anchor))
        {
            pedestrian_owner.insert(p.id.clone(), cluster.tls_id.clone());
            program_zones.push(ProgramZone {
                detector_id: p.id.clone(),
                phases: Vec::new(),
                lanes: 1,
                length_meters: geometry::length(&p.crosswalk),
                edge: format!("crossing#{}", p.anchor),
                is_pedestrian: true,
                situational: false,
            });
        }

        pending.push(Pending {
            cluster,
            junction,
            setbacks,
            refs,
            crossings,
            program_zones,
        });
    }

    let mut unassigned_pedestrian_zones = 0;
    for p in pedestrian_zones {
        let intersection = match pedestrian_owner.get(&p.id) {
            Some(tls) => tls.clone(),
            None => {
                unassigned_pedestrian_zones += 1;
                p.anchor.to_string()
            }
        };
        zones.push(Zone::pedestrian(p, intersection));
    }

    // The zones are final now (`assemble` may drop a groundless stub). Build
    // each junction's cycle from only the surviving zones, so its links, its
    // conflicts and its state strings never name a zone that isn't emitted —
    // the zone↔cycle relation stays exact.
    let zones = output::assemble(zones);
    let surviving: HashSet<&str> = zones.iter().map(|z| z.id.as_str()).collect();
    let mut programs = Vec::new();
    let mut plans = Vec::new();
    for Pending {
        cluster,
        junction,
        setbacks,
        mut refs,
        mut crossings,
        mut program_zones,
    } in pending
    {
        refs.retain(|z| surviving.contains(z.id.as_str()));
        crossings.retain(|c| surviving.contains(c.zone_id.as_str()));
        program_zones.retain(|z| surviving.contains(z.detector_id.as_str()));

        let plan = program::build(
            &graph,
            &net.positions,
            cluster,
            &junction,
            &setbacks,
            &refs,
            &crossings,
        );
        for zone in &mut program_zones {
            if let Some((_, phases)) = plan
                .zone_phases
                .iter()
                .find(|(id, _)| *id == zone.detector_id)
            {
                zone.phases = phases.clone();
            }
        }
        programs.push(JunctionProgram {
            tls_id: cluster.tls_id.clone(),
            zones: program_zones,
            transitions: plan
                .transitions
                .iter()
                .map(
                    |&(green_phase, amber_phase, amber_duration_secs)| Transition {
                        green_phase,
                        amber_phase,
                        amber_duration_secs,
                    },
                )
                .collect(),
            program: output::Program {
                program_id: "0".into(),
                phases: plan.phases.clone(),
                is_static: true,
                min_green_secs: plan
                    .is_green_phase
                    .iter()
                    .map(|&g| g.then_some(program::MIN_GREEN_SECS))
                    .collect(),
                max_green_secs: plan
                    .is_green_phase
                    .iter()
                    .map(|&g| g.then_some(program::MAX_GREEN_SECS))
                    .collect(),
            },
            links: plan
                .links
                .iter()
                .enumerate()
                .map(|(index, l)| ProgramLink {
                    index,
                    zone: l.zone.clone(),
                    from_edge: l.from_edge.clone(),
                    from_lane: l.from_lane,
                    to_edge: l.to_edge.clone(),
                    direction: l.direction.clone(),
                })
                .collect(),
            conflicts: plan.conflicts.clone(),
        });
        plans.push(plan);
    }

    Generated {
        zones,
        programs,
        projection,
        unassigned_pedestrian_zones,
        plans,
    }
}

pub fn run(input: &Path, geojson: &Path, reach: Reach) -> Result<Summary> {
    let osm = osm::read(input)?;
    let generated = generate(&osm, reach);
    output::write(geojson, &generated.zones, &generated.projection)?;
    output::write_programs(geojson, &generated.programs)?;
    let pedestrian_zones = generated
        .zones
        .iter()
        .filter(|z| z.class == output::Class::Pedestrian)
        .count();
    Ok(Summary {
        vehicle_zones: generated.zones.len() - pedestrian_zones,
        pedestrian_zones,
        junctions: generated.programs.len(),
        unassigned_pedestrian_zones: generated.unassigned_pedestrian_zones,
    })
}

/// Write the split network files a simulator reads (see
/// [`network_output`]) into `out_dir`, beside the zone catalogue. Rebuilds
/// the graph this generator already derives internally; it is a separate
/// entry point so the zone/program run pays nothing for it.
pub fn export_network(input: &Path, out_dir: &Path, reach: Reach) -> Result<()> {
    let osm = osm::read(input)?;
    let projection = Projection::centred_on(osm.nodes.values().map(|n| n.lon_lat));
    let net = Network::build(&osm, &projection);
    let signals = clusters::signal_nodes(&osm, &net);
    let graph = graph::Graph::build(&net, &signals);
    let clusters = clusters::build(&osm, &net, &graph, &signals);
    let movements: Vec<movements::Junction> = clusters
        .iter()
        .map(|cluster| movements::build(&osm, &net, &graph, cluster))
        .collect();
    let territory = input
        .file_stem()
        .and_then(|stem| stem.to_str())
        .unwrap_or("territory");
    let source_digest = network_output::run_digest(input, reach)?;
    network_output::write(
        &net,
        &graph,
        &clusters,
        &movements,
        out_dir,
        territory,
        &source_digest,
    )
}

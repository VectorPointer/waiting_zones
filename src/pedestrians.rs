//! Pedestrian waiting zones: one per signalized crossing — the crosswalk
//! itself, carried a short way onto the sidewalk at both ends.
//!
//! The crosswalk is OSM's own `footway=crossing` way (its pieces chained
//! back together where OSM drew one crossing as several ways), and
//! otherwise a segment across the road at the signalized crossing node, as
//! wide as OSM's lane counts say the road is. An unsignalized crosswalk
//! touching a zone belongs to that zone too; signalized ones side by side
//! over the same stretch of road are one zone, otherwise each is its own.

use std::collections::{BTreeSet, HashMap, HashSet};

use geo::{Area, BooleanOps, Coord, LineString, MultiPolygon};

use crate::geometry::{self, Pt};
use crate::network::is_signalized_crossing;
use crate::network::{Mode, Network};
use crate::osm::{NodeId, Osm, WayId};

/// A zebra and the margin people stand in.
pub const CROSSWALK_WIDTH_METERS: f64 = 4.0;
/// How far past each end of the crosswalk a zone reaches onto the
/// sidewalk: where people actually stand waiting for green.
pub const SIDEWALK_OFFSET_METERS: f64 = 2.0;
/// How far past the road's edge a drawn crosswalk stripe reaches, so it
/// meets the curb rather than stopping short.
const CROSSWALK_EDGE_MARGIN_METERS: f64 = 0.5;
/// A crosswalk counts as overlapping a zone only over more than this share
/// of the shorter of the two.
const MAJORITY_OVERLAP: f64 = 0.5;
/// An unsignalized crosswalk whose stripe shares more than this with a
/// zone touches it.
const MIN_SHARED_M2: f64 = 0.5;
/// Absorbing a nearby crosswalk grows a zone, which can bring another
/// within reach; bounded so a chain of crosswalks can't grow one forever.
const MAX_ABSORB_ROUNDS: usize = 10;
/// How far inside the crosswalk, from its first end, the stop line sits.
const STOP_LINE_INSET_METERS: f64 = 0.5;
/// Two crossing ways meeting end to end at a sharper angle than this are
/// two crossings meeting at a corner, not one crossing drawn in pieces.
const CHAIN_MAX_TURN_DEGREES: f64 = 45.0;
/// A crosswalk's nodes this close to the straight line between their
/// neighbours are drawn on it.
const STRAIGHT_WITHIN_METERS: f64 = 0.5;
/// Two signalized crosswalks closer to parallel than this, and no further
/// apart than [`SIDE_BY_SIDE_METERS`] alongside each other, are one zone.
const SIDE_BY_SIDE_DEGREES: f64 = 20.0;
const SIDE_BY_SIDE_METERS: f64 = 5.0;
/// A crosswalk shorter than this is a stray node, not a crossing.
const MIN_CROSSWALK_METERS: f64 = 1.0;

pub struct PedestrianZone {
    pub id: String,
    pub anchor: NodeId,
    pub polygon: MultiPolygon<f64>,
    pub stop_line: Pt,
    /// Crosswalk midpoint.
    pub site: Pt,
    /// The crosswalk polyline this zone was built around.
    pub crosswalk: Vec<Pt>,
    /// Every crosswalk the zone contains: its own first, then any
    /// signalized ones merged into it and unsignalized ones it absorbed.
    pub lines: Vec<Vec<Pt>>,
    /// Built around a mapped `footway=crossing` way, not a stripe
    /// synthesized across the road.
    pub mapped: bool,
    /// Has a light: a crossing of its junction's program. An unsignalized
    /// one is a zone only where it would otherwise be swallowed by a
    /// signalized neighbour across another street.
    pub signalized: bool,
    /// The streets (OSM `name`, else the way) its crosswalks cross.
    pub streets: BTreeSet<String>,
}

/// A crosswalk not (yet) part of any zone: an unsignalized one, or one that
/// crosses no road at all.
struct Loose {
    line: Vec<Pt>,
    streets: BTreeSet<String>,
    /// Its first node on a road, if it crosses one.
    anchor: Option<NodeId>,
}

/// Every `footway=crossing` way, with the pieces of one physical crossing
/// joined back together: two ways are joined when they are the only
/// crossing ways ending at a shared node and carry on in roughly the same
/// direction — unless each has a light of its own inside it (not just at
/// the node they share): that's a crossing in two stages, each carriageway
/// with its own light and an island to wait on between them, so two
/// places people wait (Barcelona's Avinguda del Marquès de Mont-roig at
/// `5588596925`). Returns node-id chains.
fn crossing_chains(
    osm: &Osm,
    net: &Network,
    signal_node: &dyn Fn(NodeId) -> bool,
) -> Vec<(Vec<NodeId>, Vec<WayId>)> {
    let pieces: Vec<(WayId, Vec<NodeId>)> = osm
        .ways
        .iter()
        .filter(|w| w.tag("footway") == Some("crossing"))
        .map(|w| {
            (
                w.id,
                w.refs
                    .iter()
                    .copied()
                    .filter(|n| net.positions.contains_key(n))
                    .collect::<Vec<_>>(),
            )
        })
        .filter(|(_, refs)| refs.len() >= 2)
        .collect();
    let mut ends: HashMap<NodeId, Vec<usize>> = HashMap::new();
    for (i, (_, refs)) in pieces.iter().enumerate() {
        for end in [refs[0], refs[refs.len() - 1]] {
            ends.entry(end).or_default().push(i);
        }
    }
    let direction = |from: NodeId, to: NodeId| {
        let (a, b) = (net.positions[&from], net.positions[&to]);
        geometry::unit([b[0] - a[0], b[1] - a[1]]).unwrap_or([0.0, 0.0])
    };
    let own_light = |piece: &[NodeId]| {
        piece.len() > 2 && piece[1..piece.len() - 1].iter().any(|&n| signal_node(n))
    };
    let cos_limit = CHAIN_MAX_TURN_DEGREES.to_radians().cos();
    let mut used = vec![false; pieces.len()];
    let mut chains = Vec::new();
    for start in 0..pieces.len() {
        if used[start] {
            continue;
        }
        used[start] = true;
        let mut chain = pieces[start].1.clone();
        let mut ways = vec![pieces[start].0];
        let mut lit = own_light(&pieces[start].1);
        for _ in 0..2 {
            loop {
                let end = chain[chain.len() - 1];
                let free: Vec<usize> = ends[&end].iter().copied().filter(|&i| !used[i]).collect();
                if ends[&end].len() != 2 || free.len() != 1 {
                    break;
                }
                let mut next = pieces[free[0]].1.clone();
                if next[0] != end {
                    next.reverse();
                }
                let (d1, d2) = (
                    direction(chain[chain.len() - 2], end),
                    direction(next[0], next[1]),
                );
                if d1[0] * d2[0] + d1[1] * d2[1] < cos_limit {
                    break;
                }
                if lit && own_light(&next) {
                    break;
                }
                lit |= own_light(&next);
                used[free[0]] = true;
                ways.push(pieces[free[0]].0);
                chain.extend(next.into_iter().skip(1));
            }
            chain.reverse();
        }
        chains.push((chain, ways));
    }
    chains
}

pub fn generate(osm: &Osm, net: &Network) -> Vec<PedestrianZone> {
    let on_car_road = |node: NodeId| {
        net.incidences
            .get(&node)
            .is_some_and(|list| list.iter().any(|inc| net.roads[inc.road].mode == Mode::Car))
    };
    let bike_only = crate::network::bike_only_crossings(osm);
    let signal_node = |n: NodeId| {
        !bike_only.contains(&n)
            && osm.nodes.get(&n).is_some_and(|n| {
                is_signalized_crossing(n.tag("crossing"), n.tag("crossing:signals"))
            })
    };
    let signalized_nodes: BTreeSet<NodeId> = osm
        .nodes
        .keys()
        .copied()
        .filter(|&n| signal_node(n) && on_car_road(n))
        .collect();
    let ways_by_id: HashMap<WayId, &crate::osm::Way> = osm.ways.iter().map(|w| (w.id, w)).collect();

    let mut covered: HashSet<NodeId> = HashSet::new();
    let mut zones = Vec::new();
    let mut unsignalized: Vec<Loose> = Vec::new();
    // The street a road node lies on: its way's name, else the way itself.
    let streets = |nodes: &[NodeId]| -> BTreeSet<String> {
        nodes
            .iter()
            .flat_map(|n| net.incidences.get(n).into_iter().flatten())
            .filter(|inc| net.roads[inc.road].mode == Mode::Car)
            .map(|inc| {
                let way = net.roads[inc.road].way;
                ways_by_id
                    .get(&way)
                    .and_then(|w| w.tag("name"))
                    .map_or_else(|| way.to_string(), str::to_string)
            })
            .collect()
    };
    for (refs, ways) in crossing_chains(osm, net, &signal_node) {
        let points: Vec<Pt> = refs.iter().map(|n| net.positions[n]).collect();
        let signalized = refs.iter().any(|&n| signal_node(n))
            || ways.iter().any(|w| {
                let way = ways_by_id[w];
                is_signalized_crossing(way.tag("crossing"), way.tag("crossing:signals"))
            });
        let road_indices: Vec<usize> = refs
            .iter()
            .enumerate()
            .filter(|(_, n)| on_car_road(**n))
            .map(|(i, _)| i)
            .collect();
        let (Some(&first), Some(&last)) = (road_indices.first(), road_indices.last()) else {
            // Shares no node with a road (a path across a park, or the
            // stretch of a two-stage crossing over its median island): it
            // stops no traffic, whatever lights OSM puts on it, so it's
            // never a zone of its own — but still a zebra the zone it
            // touches has to contain.
            unsignalized.push(Loose {
                line: points,
                streets: BTreeSet::new(),
                anchor: None,
            });
            continue;
        };
        let crosswalk = points;
        let crossed = streets(&refs[first..=last]);
        if !signalized {
            unsignalized.push(Loose {
                line: crosswalk,
                streets: crossed,
                anchor: Some(refs[first]),
            });
            continue;
        }
        covered.extend(refs.iter().copied());
        let anchor = refs[first..=last]
            .iter()
            .copied()
            .find(|n| signalized_nodes.contains(n))
            .unwrap_or(refs[first]);
        if let Some(mut zone) = zone_around(format!("{anchor}_ped"), anchor, crosswalk, true) {
            zone.streets = crossed;
            zones.push(zone);
        }
    }

    // Signalized crossing nodes with no mapped crossing way: synthesize one
    // straight across the road.
    for &node in &signalized_nodes {
        if covered.contains(&node) {
            continue;
        }
        let Some(inc) = net.incidences[&node]
            .iter()
            .filter(|inc| net.roads[inc.road].mode == Mode::Car)
            .max_by(|a, b| {
                net.roads[a.road]
                    .width()
                    .total_cmp(&net.roads[b.road].width())
            })
        else {
            continue;
        };
        let Some(tangent) = net.tangent(inc.road, inc.index) else {
            continue;
        };
        let axis = geometry::right_of(tangent);
        let half = net.roads[inc.road].width() / 2.0 + CROSSWALK_EDGE_MARGIN_METERS;
        let centre = net.positions[&node];
        let crosswalk = vec![
            [centre[0] - axis[0] * half, centre[1] - axis[1] * half],
            [centre[0] + axis[0] * half, centre[1] + axis[1] * half],
        ];
        if let Some(mut zone) = zone_around(format!("{node}_ped"), node, crosswalk, false) {
            zone.streets = streets(&[node]);
            zones.push(zone);
        }
    }

    let mut zones = merge_side_by_side(zones);
    let left = absorb(&mut zones, unsignalized);
    // An unsignalized crosswalk over another street, touching a zone: a
    // place of its own (Barcelona's `590815626_ped`, a zebra across the
    // Plaça de Josep Irla link next to Lluís Companys' signalized one).
    for loose in left {
        let stripe = zone_polygon(std::slice::from_ref(&loose.line));
        let touches = zones
            .iter()
            .any(|z| z.polygon.intersection(&stripe).unsigned_area() > MIN_SHARED_M2);
        if let (true, Some(anchor)) = (touches, loose.anchor)
            && let Some(mut zone) = zone_around(format!("{anchor}_ped"), anchor, loose.line, true)
        {
            zone.signalized = false;
            zone.streets = loose.streets;
            zones.push(zone);
        }
    }
    zones
}

/// Two signalized crossings drawn side by side over the same stretch of
/// road, their stripes touching (a zebra and the bike crossing painted
/// right beside it, each mapped on its own node), are one place people
/// wait: one zone. The zone keeps
/// the mapped crossing's id (else the lowest node's) and takes in the
/// other's crosswalk.
fn merge_side_by_side(zones: Vec<PedestrianZone>) -> Vec<PedestrianZone> {
    let mut group: Vec<usize> = (0..zones.len()).collect();
    let root = |group: &Vec<usize>, mut i: usize| {
        while group[i] != i {
            i = group[i];
        }
        i
    };
    for i in 0..zones.len() {
        for j in i + 1..zones.len() {
            // Side by side and touching: two stripes with a gap between
            // are two places to wait, however parallel.
            let touching = zones[i]
                .polygon
                .intersection(&zones[j].polygon)
                .unsigned_area()
                > MIN_SHARED_M2;
            if touching && side_by_side(&zones[i].crosswalk, &zones[j].crosswalk) {
                let (a, b) = (root(&group, i), root(&group, j));
                group[a.max(b)] = a.min(b);
            }
        }
    }
    let mut members: HashMap<usize, Vec<PedestrianZone>> = HashMap::new();
    for (i, zone) in zones.into_iter().enumerate() {
        members.entry(root(&group, i)).or_default().push(zone);
    }
    let mut merged: Vec<PedestrianZone> = members
        .into_values()
        .map(|mut members| {
            members.sort_by_key(|z| (!z.mapped, z.anchor));
            let mut members = members.into_iter();
            let mut zone = members.next().expect("a group has a member");
            let others: Vec<PedestrianZone> = members.collect();
            if !others.is_empty() {
                for other in &others {
                    zone.streets.extend(other.streets.iter().cloned());
                }
                zone.lines.extend(others.into_iter().flat_map(|z| z.lines));
                zone.polygon = zone_polygon(&zone.lines);
            }
            zone
        })
        .collect();
    merged.sort_by_key(|z| z.anchor);
    merged
}

/// Whether crosswalks `a` and `b` run side by side: within
/// [`SIDE_BY_SIDE_DEGREES`] of parallel, no further apart than
/// [`SIDE_BY_SIDE_METERS`], and alongside each other for most of the
/// shorter one's length — not end to end, as the two halves of a crossing
/// over a median are.
fn side_by_side(a: &[Pt], b: &[Pt]) -> bool {
    let ends = |l: &[Pt]| (l[0], l[l.len() - 1]);
    let ((a0, a1), (b0, b1)) = (ends(a), ends(b));
    let (Some(ua), Some(ub)) = (
        geometry::unit([a1[0] - a0[0], a1[1] - a0[1]]),
        geometry::unit([b1[0] - b0[0], b1[1] - b0[1]]),
    ) else {
        return false;
    };
    if (ua[0] * ub[0] + ua[1] * ub[1]).abs() < SIDE_BY_SIDE_DEGREES.to_radians().cos() {
        return false;
    }
    let along = |p: Pt| (p[0] - a0[0]) * ua[0] + (p[1] - a0[1]) * ua[1];
    let across = |p: Pt| ((p[0] - a0[0]) * ua[1] - (p[1] - a0[1]) * ua[0]).abs();
    let mid_b = [(b0[0] + b1[0]) / 2.0, (b0[1] + b1[1]) / 2.0];
    if across(mid_b) > SIDE_BY_SIDE_METERS {
        return false;
    }
    let (len_a, (lo, hi)) = (geometry::dist(a0, a1), {
        let (x, y) = (along(b0), along(b1));
        (x.min(y), x.max(y))
    });
    let overlap = hi.min(len_a) - lo.max(0.0);
    overlap > MAJORITY_OVERLAP * len_a.min(hi - lo)
}

/// An unsignalized crosswalk whose own stripe overlaps a zone's ground
/// belongs to that zone (the one it overlaps most) — if it crosses the
/// same street, or none (a median island's stretch): the zone takes it in
/// and stays one piece. One that doesn't touch any zone stays out, as does
/// one across another street: merging either in would make one zone of two
/// places people wait. Taking one in grows the zone, so repeat until
/// nothing more touches. Returns what's left.
fn absorb(zones: &mut [PedestrianZone], mut unowned: Vec<Loose>) -> Vec<Loose> {
    for _ in 0..MAX_ABSORB_ROUNDS {
        let mut grown: HashSet<usize> = HashSet::new();
        unowned.retain(|loose| {
            let stripe = zone_polygon(std::slice::from_ref(&loose.line));
            let best = zones
                .iter()
                .enumerate()
                .filter(|(_, zone)| {
                    loose.streets.is_empty() || !loose.streets.is_disjoint(&zone.streets)
                })
                .map(|(z, zone)| (z, zone.polygon.intersection(&stripe).unsigned_area()))
                .filter(|&(_, shared)| shared > MIN_SHARED_M2)
                .max_by(|a, b| a.1.total_cmp(&b.1));
            match best {
                Some((z, _)) => {
                    zones[z].lines.push(loose.line.clone());
                    grown.insert(z);
                    false
                }
                None => true,
            }
        });
        if grown.is_empty() {
            break;
        }
        for z in grown {
            zones[z].polygon = zone_polygon(&zones[z].lines);
        }
    }
    unowned
}

/// Every line's stripe, carried [`SIDEWALK_OFFSET_METERS`] onto the
/// sidewalk at both ends.
fn zone_polygon(lines: &[Vec<Pt>]) -> MultiPolygon<f64> {
    lines
        .iter()
        .map(|line| {
            geometry::stroke(
                &extended(&straightened(line), SIDEWALK_OFFSET_METERS),
                CROSSWALK_WIDTH_METERS,
            )
        })
        .fold(MultiPolygon::new(Vec::new()), |acc, stripe| {
            acc.union(&stripe)
        })
}

/// `line` without the kinks OSM's nodes put in a crosswalk that's really
/// straight (the node where it meets the road a little off the line
/// between its ends): a bend of less than [`STRAIGHT_WITHIN_METERS`] is
/// drawn straight, so the zone's sides are too. A real turn (an L-shaped
/// crosswalk round a corner) stays.
fn straightened(line: &[Pt]) -> Vec<Pt> {
    use geo::Simplify;
    LineString::new(line.iter().map(|&[x, y]| Coord { x, y }).collect())
        .simplify(STRAIGHT_WITHIN_METERS)
        .coords()
        .map(|c| [c.x, c.y])
        .collect()
}

/// `line` carried `offset` metres further along its own direction at both
/// ends.
fn extended(line: &[Pt], offset: f64) -> Vec<Pt> {
    let mut out = line.to_vec();
    let reversed: Vec<Pt> = line.iter().rev().copied().collect();
    if let (Some(out_start), Some(out_end)) = (
        geometry::end_direction(&reversed),
        geometry::end_direction(line),
    ) {
        let (start, end) = (line[0], line[line.len() - 1]);
        out[0] = [
            start[0] + out_start[0] * offset,
            start[1] + out_start[1] * offset,
        ];
        let last = out.len() - 1;
        out[last] = [end[0] + out_end[0] * offset, end[1] + out_end[1] * offset];
    }
    out
}

fn zone_around(
    id: String,
    anchor: NodeId,
    crosswalk: Vec<Pt>,
    mapped: bool,
) -> Option<PedestrianZone> {
    let length = geometry::length(&crosswalk);
    if length < MIN_CROSSWALK_METERS {
        return None;
    }
    let lines = vec![crosswalk.clone()];
    let polygon = zone_polygon(&lines);
    let site = *geometry::sub_polyline(&crosswalk, 0.0, length / 2.0).last()?;
    // Just inside the crosswalk's first end: always on the zone's own
    // crosswalk, which no neighbour can take.
    let stop_line =
        *geometry::sub_polyline(&crosswalk, 0.0, STOP_LINE_INSET_METERS.min(length / 2.0))
            .last()?;
    Some(PedestrianZone {
        id,
        anchor,
        polygon,
        stop_line,
        site,
        crosswalk,
        lines,
        mapped,
        signalized: true,
        streets: BTreeSet::new(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::osm::read_from;
    use crate::projection::Projection;
    use geo::{Contains, Point};

    #[test]
    fn a_mapped_crossing_way_becomes_one_zone_containing_it() {
        let osm = read_from(
            r#"<osm>
            <node id="1" lat="41.4000" lon="2.1990"/>
            <node id="2" lat="41.4000" lon="2.2000"><tag k="highway" v="crossing"/><tag k="crossing" v="traffic_signals"/></node>
            <node id="3" lat="41.4000" lon="2.2010"/>
            <node id="4" lat="41.40007" lon="2.2000"/>
            <node id="5" lat="41.39993" lon="2.2000"/>
            <way id="100"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="primary"/></way>
            <way id="300"><nd ref="4"/><nd ref="2"/><nd ref="5"/><tag k="highway" v="footway"/><tag k="footway" v="crossing"/></way>
            </osm>"#
                .as_bytes(),
        )
        .unwrap();
        let projection = Projection::centred_on(osm.nodes.values().map(|n| n.lon_lat));
        let net = Network::build(&osm, &projection);
        let zones = generate(&osm, &net);
        assert_eq!(zones.len(), 1);
        let zone = &zones[0];
        assert_eq!(zone.id, "2_ped");
        for id in [4, 5, 2] {
            let [x, y] = net.positions[&id];
            assert!(
                zone.polygon.contains(&Point::new(x, y)),
                "crosswalk node {id} outside the zone"
            );
        }
    }

    #[test]
    fn an_unmapped_signalized_crossing_gets_a_synthesized_stripe_across_the_road() {
        let osm = read_from(
            r#"<osm>
            <node id="1" lat="41.4000" lon="2.1990"/>
            <node id="2" lat="41.4000" lon="2.2000"><tag k="crossing" v="traffic_signals"/></node>
            <node id="3" lat="41.4000" lon="2.2010"/>
            <way id="100"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="primary"/><tag k="lanes" v="4"/></way>
            </osm>"#
                .as_bytes(),
        )
        .unwrap();
        let projection = Projection::centred_on(osm.nodes.values().map(|n| n.lon_lat));
        let net = Network::build(&osm, &projection);
        let zones = generate(&osm, &net);
        assert_eq!(zones.len(), 1);
        // 4 lanes × 3m: the stripe must reach both kerbs, 6m either side.
        let [x, y] = net.positions[&2];
        for dy in [-6.0, 6.0] {
            assert!(zones[0].polygon.contains(&Point::new(x, y + dy)));
        }
    }
}

//! Vehicle waiting zones: one per group of an approach's lanes serving the
//! same turns, covering those lanes from the stop line back along the whole
//! approach and on upstream through every predecessor that only leads here.

use std::collections::{BTreeSet, HashSet};

use geo::{BooleanOps, MultiPolygon};

use crate::geometry::{self, Pt};
use crate::graph::Graph;
use crate::network::{Direction, Mode, Network};
use crate::osm::NodeId;

/// Two pieces of a route whose centrelines meet within this share a node.
const JOINT_METERS: f64 = 0.1;
/// How much of each other arm of a junction a zone ends at counts as that
/// junction's ground.
const ARM_METERS: f64 = 30.0;
/// A band turning back sharper than this (past what an offset line can
/// follow) is drawn as two runs meeting square.
const SHARP_TURN_DEGREES: f64 = 100.0;
/// Two arms leaving a node this close in heading (degrees) are the same road.
const SAME_ARM_DEGREES: f64 = 10.0;
/// The shortest an approach is taken to be, so a near-zero-length edge still
/// gets a stop line inside it.
const MIN_APPROACH_METERS: f64 = 1.0;
/// How far behind the stop line the published stop point sits.
const STOP_LINE_SETBACK_METERS: f64 = 0.5;
/// The nearest the stop point may sit to the approach's own start.
const MIN_STOP_LINE_OFFSET_METERS: f64 = 0.1;

pub struct VehicleZone {
    pub id: String,
    pub approach: usize,
    pub directions: BTreeSet<Direction>,
    /// Approach lane indices (0 = rightmost) this zone covers.
    pub lanes: Vec<u32>,
    pub mode: Mode,
    pub polygon: MultiPolygon<f64>,
    pub stop_line: Pt,
    /// The highest OSM level (`layer`/`bridge`/`tunnel`) the zone's own road
    /// or any road it extends back through sits at. A zone at a different
    /// level than a neighbour crosses it in plan but shares no ground.
    pub layer: i32,
    pub length_meters: f64,
    /// Both flat ends of every band the zone is drawn from.
    pub ends: Vec<BandEnd>,
    /// The junctions the zone ends at — the carriageways of their other
    /// arms — which it must stay out of.
    pub keep_out: MultiPolygon<f64>,
}

/// A flat end of a lane band: its middle, the direction pointing out of the
/// band, and the band's width.
#[derive(Clone, Copy)]
pub struct BandEnd {
    pub point: Pt,
    pub outward: Pt,
    pub width: f64,
}

pub fn zone_id(edge_id: &str, directions: &BTreeSet<Direction>) -> String {
    let labels: Vec<&str> = directions.iter().map(|d| d.label()).collect();
    format!("{edge_id}_{}", labels.join("+"))
}

/// Limits on how far a zone reaches back from its stop line.
#[derive(Clone, Copy)]
pub struct Reach {
    /// Cap on the zone's whole length back from its stop line. `None` (the
    /// default) leaves it uncapped: the zone covers its approach and, past
    /// its start, every road that only leads into it, until a fork or a
    /// signal stops the walk.
    pub max_length: Option<f64>,
}

#[allow(clippy::too_many_arguments)]
pub fn build(
    net: &Network,
    graph: &Graph,
    controlled: &HashSet<NodeId>,
    approach: usize,
    directions: BTreeSet<Direction>,
    lanes: Vec<u32>,
    setback: f64,
    reach: Reach,
) -> Option<VehicleZone> {
    let edge = &graph.edges[approach];
    let stop = (edge.length - setback).max(edge.length.min(MIN_APPROACH_METERS));
    let along_approach = reach.max_length.map_or(stop, |cap| cap.min(stop));
    // The remaining length the walk may add before the approach's own start:
    // unlimited when uncapped, otherwise whatever the cap leaves over.
    let budget = match reach.max_length {
        Some(cap) => (cap - along_approach).max(0.0),
        None => f64::INFINITY,
    };

    // Every upstream route, as chains of (edge, from, to) pieces ordered
    // from the approach backwards.
    let mut routes: Vec<Vec<(usize, f64, f64)>> = Vec::new();
    let mut visited = HashSet::from([approach]);
    extend(
        net,
        graph,
        controlled,
        approach,
        budget,
        vec![(approach, stop - along_approach, stop)],
        &mut visited,
        &mut routes,
    );

    let mut polygon = MultiPolygon::new(Vec::new());
    let mut ends = Vec::new();
    // The junction at the zone's head, and at the upstream end of every
    // route that runs all the way back to one (a fork, or another
    // controller): the zone stops at their edge instead of reaching their
    // middle (seen on real Barcelona data, `577690679#0_right` reaching
    // into the Rotonda de l'Alguer), cut square across its lanes.
    let head_ground = junction_ground(net, graph, edge.to, approach);
    let mut keep_out = MultiPolygon::new(Vec::new());
    let mut length_meters: f64 = 0.0;
    for route in &routes {
        length_meters = length_meters.max(route.iter().map(|(_, from, to)| to - from).sum());
        // Contiguous lanes are one band the group's whole width, following
        // the group's own centre: unioning one band per lane instead leaves
        // a hairline crack wherever two lanes' bands miss meeting exactly,
        // drawn on a map as a line through the zone (confirmed on real
        // Barcelona data, `673926024#1_straight`). Lanes that aren't
        // contiguous (a group split by a lane serving other turns) still get
        // a band each.
        let tail_ground = match route.last() {
            Some(&(e, from, _)) if from <= 1e-6 => {
                junction_ground(net, graph, graph.edges[e].from, e)
            }
            _ => JunctionGround::none(),
        };
        let contiguous = lanes.windows(2).all(|w| w[1] == w[0] + 1);
        let bands: Vec<Vec<u32>> = if contiguous {
            vec![lanes.clone()]
        } else {
            lanes.iter().map(|&l| vec![l]).collect()
        };
        for band in &bands {
            // One polyline per band per route, upstream end first: the
            // route's own centreline shifted onto the band's lanes all at
            // once, each vertex by its own piece's shift. Shifting each
            // piece on its own leaves their ends apart wherever the road
            // bends at the node between them (each piece's end follows its
            // own last segment), and drawing those apart ends as separate
            // runs broke every such corner (seen on real Barcelona data,
            // `47395959#1_straight+left+right`); drawing them as one line
            // with a sideways jump bulged into the next lane over
            // (`40762312#5_straight`). A node between two pieces whose lanes
            // sit differently across the road takes the middle of both, so
            // the band slides across over the segments either side.
            let mut centreline: Vec<Pt> = Vec::new();
            let mut shifts: Vec<f64> = Vec::new();
            for &(e, from, to) in route.iter().rev() {
                let piece = &graph.edges[e];
                let offsets: Vec<f64> = band
                    .iter()
                    .map(|&l| piece.lane_offset(l.min(piece.lanes - 1)))
                    .collect();
                let shift = offsets.iter().sum::<f64>() / offsets.len() as f64;
                for (k, point) in geometry::sub_polyline(&piece.points, from, to)
                    .into_iter()
                    .enumerate()
                {
                    let joint = k == 0
                        && centreline
                            .last()
                            .is_some_and(|&q| geometry::dist(q, point) <= JOINT_METERS);
                    if joint {
                        let last = shifts.len() - 1;
                        shifts[last] = (shifts[last] + shift) / 2.0;
                    } else if centreline.last() != Some(&point) {
                        centreline.push(point);
                        shifts.push(shift);
                    }
                }
            }
            if centreline.len() < 2 {
                continue;
            }
            let line = geometry::offset_each(&centreline, &shifts);
            let runs = geometry::split_at_sharp_turns(&line, SHARP_TURN_DEGREES);
            let (Some(first), Some(last)) = (runs.first(), runs.last()) else {
                continue;
            };
            let width = edge.lane_width * band.len() as f64;
            let reversed: Vec<Pt> = first.iter().rev().copied().collect();
            for (end, direction, ground) in [
                (last.last(), geometry::end_direction(last), &head_ground),
                (
                    first.first(),
                    geometry::end_direction(&reversed),
                    &tail_ground,
                ),
            ] {
                if let (Some(&point), Some(outward)) = (end, direction) {
                    let end = BandEnd {
                        point,
                        outward,
                        width,
                    };
                    keep_out = keep_out.union(&square_cut(&end, ground));
                    ends.push(end);
                }
            }
            for run in &runs {
                polygon = polygon.union(&geometry::stroke(run, width));
            }
        }
    }
    if polygon.0.is_empty() {
        return None;
    }
    // Where two upstream routes part and meet again they can enclose a
    // sliver between their bands: the zone is the whole area, hole included.
    let polygon = geometry::without_holes(polygon);

    // The highest level any road on the route sits at: a zone that climbs
    // onto a bridge is a bridge zone, and does not share ground with a
    // street crossing underneath it.
    let layer = routes
        .iter()
        .flatten()
        .map(|(e, _, _)| net.roads[graph.edges[*e].road].layer)
        .max()
        .unwrap_or(0);

    // The stop line: half a metre behind it, on the group's own middle
    // lane — the group's lanes needn't be contiguous, and the average of two
    // of them could land on a lane another zone owns.
    let middle = lanes[lanes.len() / 2];
    let centre_line = edge.lane_line(middle);
    let stop_line = *geometry::sub_polyline(
        &centre_line,
        0.0,
        (stop - STOP_LINE_SETBACK_METERS).max(MIN_STOP_LINE_OFFSET_METERS),
    )
    .last()?;

    Some(VehicleZone {
        id: zone_id(&edge.id, &directions),
        approach,
        directions,
        lanes,
        mode: edge.mode,
        polygon,
        stop_line,
        layer,
        length_meters,
        ends,
        keep_out,
    })
}

/// Walks upstream from `edge`, adding every predecessor that isn't at a
/// controlled node and only leads on to `edge`, until `budget` runs out
/// (unlimited — `f64::INFINITY` — when the zone is uncapped, in which case
/// only a fork or a signal ends the walk).
/// Through a junction only where it's a pure merge — `edge` its one way out,
/// so every car arriving there queues here (Barcelona's `332179965`, where
/// a Ronda slip road and Mestre Lluís Millet both feed `584980793`); a
/// junction any of whose traffic can leave another way ends the zone
/// (`590815629`, where the Plaça de Josep Irla link carries on as well as
/// turning into `698127484`).
#[allow(clippy::too_many_arguments)]
fn extend(
    net: &Network,
    graph: &Graph,
    controlled: &HashSet<NodeId>,
    edge: usize,
    budget: f64,
    route: Vec<(usize, f64, f64)>,
    visited: &mut HashSet<usize>,
    routes: &mut Vec<Vec<(usize, f64, f64)>>,
) {
    let current = &graph.edges[edge];
    let start = current.from;
    // A parking aisle is not part of the street: a queue carries on past the
    // entrance it opens onto, so it never counts as a way out of the node.
    let parking = |e: usize| net.roads[graph.edges[e].road].is_parking;
    let only_way_out = || {
        graph
            .outgoing(start)
            .iter()
            .all(|&c| c == edge || graph.edges[c].mode != current.mode || parking(c))
    };
    let predecessors: Vec<usize> = if budget <= 0.0
        || controlled.contains(&start)
        || (net.is_junction(start) && !only_way_out())
    {
        Vec::new()
    } else {
        graph
            .incoming(start)
            .iter()
            .copied()
            .filter(|&p| {
                Some(p) != current.reverse
                    && graph.edges[p].mode == current.mode
                    && !visited.contains(&p)
            })
            .filter(|&p| {
                let continuations: Vec<usize> = graph
                    .outgoing(start)
                    .iter()
                    .copied()
                    .filter(|&c| {
                        Some(c) != graph.edges[p].reverse
                            && graph.edges[c].mode == current.mode
                            && !parking(c)
                    })
                    .collect();
                continuations == [edge]
            })
            .collect()
    };
    if predecessors.is_empty() {
        routes.push(route);
        return;
    }
    for p in predecessors {
        visited.insert(p);
        let length = graph.edges[p].length;
        let mut next = route.clone();
        next.push((p, (length - budget).max(0.0), length));
        extend(
            net,
            graph,
            controlled,
            p,
            budget - length,
            next,
            visited,
            routes,
        );
    }
}

/// How far into a band from its end a junction's ground may reach and
/// still be cut off it.
const JUNCTION_SEARCH_METERS: f64 = 15.0;

/// The part of a band to cut off at `end` so it stays out of `ground`: from
/// the band's end in to a straight line across it at the angle the
/// junction's edge crosses the band (from where each side of the band
/// leaves the ground), pushed in until no ground is left on the band's side
/// of it — a straight edge along the junction's, whatever shape its
/// outline has.
fn square_cut(end: &BandEnd, junction: &JunctionGround) -> MultiPolygon<f64> {
    let ground = &junction.ground;
    let inward = [-end.outward[0], -end.outward[1]];
    let side = geometry::right_of(inward);
    let at = |lateral: f64, depth: f64| {
        [
            end.point[0] + side[0] * lateral + inward[0] * depth,
            end.point[1] + side[1] * lateral + inward[1] * depth,
        ]
    };
    let along =
        |c: &geo::Coord<f64>| (c.x - end.point[0]) * inward[0] + (c.y - end.point[1]) * inward[1];
    // The junction's ground over the band's end: the parts of it within
    // the band that reach its end.
    let window = geometry::rectangle(
        at(0.0, JUNCTION_SEARCH_METERS / 2.0),
        inward,
        JUNCTION_SEARCH_METERS,
        end.width + 2.0 * CUT_MARGIN_METERS,
    );
    let touching = MultiPolygon::new(
        window
            .intersection(ground)
            .0
            .into_iter()
            .filter(|part| {
                part.exterior()
                    .coords()
                    .map(along)
                    .fold(f64::INFINITY, f64::min)
                    <= CUT_MARGIN_METERS
            })
            .collect(),
    );
    // How far in that ground reaches along the line `lateral` metres off
    // the band's middle.
    let depth = |lateral: f64| {
        let strip = geometry::rectangle(
            at(lateral, JUNCTION_SEARCH_METERS / 2.0),
            inward,
            JUNCTION_SEARCH_METERS,
            CUT_MARGIN_METERS,
        );
        strip
            .intersection(&touching)
            .0
            .iter()
            .flat_map(|part| part.exterior().coords().map(along).collect::<Vec<_>>())
            .fold(0.0, f64::max)
    };
    let half = end.width / 2.0;
    let samples: Vec<(f64, f64)> = (0..=CUT_SAMPLES)
        .map(|k| {
            let lateral = -half + end.width * k as f64 / CUT_SAMPLES as f64;
            (lateral, depth(lateral))
        })
        .collect();
    if samples.iter().all(|&(_, d)| d <= 0.0) {
        return MultiPolygon::new(Vec::new());
    }
    // The cut's slope across the band: square where the band's own road
    // carries on through the junction, along the road it meets otherwise,
    // and along the junction's own edge where no road runs straight across.
    let slope = match junction.cut {
        Cut::Square => 0.0,
        Cut::Along(d) => {
            let (across, into) = (
                d[0] * side[0] + d[1] * side[1],
                d[0] * inward[0] + d[1] * inward[1],
            );
            if across.abs() < MIN_CUT_ACROSS {
                0.0
            } else {
                into / across
            }
        }
        Cut::Edge => (samples[CUT_SAMPLES].1 - samples[0].1) / end.width,
    };
    let lift = samples
        .iter()
        .filter(|&&(_, d)| d > 0.0)
        .map(|&(lateral, d)| d - slope * lateral)
        .fold(f64::NEG_INFINITY, f64::max)
        .max(0.0);
    let line = |lateral: f64| lift + slope * lateral;
    let reach = half + CUT_MARGIN_METERS;
    geometry::polygon(vec![
        at(-reach, -CUT_MARGIN_METERS),
        at(-reach, line(-reach).max(0.0)),
        at(reach, line(reach).max(0.0)),
        at(reach, -CUT_MARGIN_METERS),
    ])
}

/// A road meeting the band this close to lengthwise (the cosine of its angle
/// to the band's sides) gives no usable cut; the cut is square instead.
const MIN_CUT_ACROSS: f64 = 0.3;
/// Two arms of a junction this close to straight on (degrees) are one road
/// running through it.
const THROUGH_DEGREES: f64 = 150.0;

/// Points across a band's width where a cut checks the junction's depth.
const CUT_SAMPLES: usize = 8;

/// A cut reaches this far past the band's sides and end, so no sliver of
/// the band survives beside it.
const CUT_MARGIN_METERS: f64 = 0.1;

/// What a zone ending at a junction must stay out of, and how to cut it.
pub struct JunctionGround {
    ground: MultiPolygon<f64>,
    cut: Cut,
}

impl JunctionGround {
    fn none() -> Self {
        JunctionGround {
            ground: MultiPolygon::new(Vec::new()),
            cut: Cut::Square,
        }
    }
}

/// The angle a band's end is cut at.
enum Cut {
    /// Square across the band: its own road carries on through.
    Square,
    /// Along this direction: the road the band runs into.
    Along(Pt),
    /// Along the junction's own edge: no road runs straight across.
    Edge,
}

/// The ground of junction `node`, as far as a zone on `edge` is concerned:
/// the carriageways of its other arms, each [`ARM_METERS`] from the node.
/// Empty where `node` isn't a junction.
fn junction_ground(net: &Network, graph: &Graph, node: NodeId, edge: usize) -> JunctionGround {
    let mut ground = MultiPolygon::new(Vec::new());
    if !net.is_junction(node) {
        return JunctionGround::none();
    }
    // Every arm's direction away from the node, one per road arm (a
    // two-way road's two edges are one arm).
    let away = |e: usize| {
        let points = &graph.edges[e].points;
        let n = points.len();
        let (from, to) = if graph.edges[e].to == node {
            (points[n - 1], points[n - 2])
        } else {
            (points[0], points[1])
        };
        geometry::unit([to[0] - from[0], to[1] - from[1]])
    };
    let through = THROUGH_DEGREES.to_radians().cos();
    let same_arm = SAME_ARM_DEGREES.to_radians().cos();
    let own = away(edge);
    let mut arms: Vec<Pt> = Vec::new();
    for &e in graph.incoming(node).iter().chain(graph.outgoing(node)) {
        if let Some(d) = away(e)
            && own.is_none_or(|o| o[0] * d[0] + o[1] * d[1] < same_arm)
            && arms.iter().all(|a| a[0] * d[0] + a[1] * d[1] < same_arm)
        {
            arms.push(d);
        }
    }
    let dot = |a: Pt, b: Pt| a[0] * b[0] + a[1] * b[1];
    // The road running straightest through the junction: the pair of arms
    // closest to opposite. The band's own road, if it's part of that pair,
    // carries on — cut square; otherwise the band runs into that road — cut
    // along it (a branch leaving a main road at a shallow angle is the
    // branch, however close to straight on it looks from the main road).
    // A slip road (`highway=*_link`) is itself a branch, never the street a
    // queue carries on along, even where it lines up with another arm: the
    // road crossing it is what cuts the zone, so its cut follows that
    // crossing road rather than running square (Barcelona's
    // `698127484#0_right`, collinear with the Plaça de Josep Irla link
    // `46270533` but cut by Mestre Lluís Millet crossing it).
    let mut all: Vec<(bool, Pt)> = arms.iter().map(|&a| (false, a)).collect();
    if let Some(o) = own {
        all.push((true, o));
    }
    let straightest = all
        .iter()
        .enumerate()
        .flat_map(|(i, &a)| all[i + 1..].iter().map(move |&b| (a, b)))
        .filter(|&((_, a), (_, b))| dot(a, b) < through)
        .min_by(|x, y| dot(x.0.1, x.1.1).total_cmp(&dot(y.0.1, y.1.1)));
    let mut cut = match straightest {
        Some(((true, _), _) | (_, (true, _))) => Cut::Square,
        Some(((_, a), _)) => Cut::Along(a),
        None => Cut::Edge,
    };
    if matches!(cut, Cut::Square)
        && net.roads[graph.edges[edge].road].is_link
        && let Some(o) = own
        && let Some(&a) = arms
            .iter()
            .min_by(|x, y| dot(**x, o).abs().total_cmp(&dot(**y, o).abs()))
    {
        cut = Cut::Along(a);
    }
    let own = [Some(edge), graph.edges[edge].reverse];
    // Where every arm (the zone's own included) meets the node, the two
    // corners of its carriageway: the box they span is the junction's
    // floor. Straight-edged, so a zone ends on a straight line across its
    // lanes, not an arc.
    let mut corners: Vec<geo::Point<f64>> = Vec::new();
    for &arm in graph.incoming(node).iter().chain(graph.outgoing(node)) {
        let e = &graph.edges[arm];
        let width = net.roads[e.road].width();
        // The arm's direction where it meets the node (either way round:
        // only its normal is used).
        let (near, at_node) = if e.to == node {
            let near = geometry::sub_polyline(&e.points, e.length - ARM_METERS, e.length);
            let at_node = geometry::end_direction(&near);
            (near, at_node)
        } else {
            let near = geometry::sub_polyline(&e.points, 0.0, ARM_METERS);
            let reversed: Vec<Pt> = near.iter().rev().copied().collect();
            (near, geometry::end_direction(&reversed))
        };
        if let Some(d) = at_node {
            let (n, c) = (geometry::right_of(d), net.positions[&node]);
            for side in [-0.5, 0.5] {
                corners.push(geo::Point::new(
                    c[0] + n[0] * width * side,
                    c[1] + n[1] * width * side,
                ));
            }
        }
        if !own.contains(&Some(arm)) {
            // The arm's own carriageway, at its mapped lane count (OSM
            // rarely maps one, so a one-lane road) rather than a fixed
            // floor: a crossing street cuts the zone where its own edge
            // is, not where a wider guess would put it.
            let across = width.max(crate::network::CAR_LANE_WIDTH_METERS);
            ground = ground.union(&geometry::stroke(&near, across));
        }
    }
    use geo::ConvexHull;
    let hull = geo::MultiPoint::new(corners).convex_hull();
    JunctionGround {
        ground: ground.union(&MultiPolygon::new(vec![hull])),
        cut,
    }
}

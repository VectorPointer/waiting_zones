//! Generates vehicle waiting zones — one
//! [`E3Detector`](sumo_types::additional::domain::E3Detector) per zone — at
//! traffic-light-controlled junctions.
//!
//! There's no project-specific "waiting zone" type: `sumo_types` already
//! models exactly what one is (an area delimited by entry/exit gates), so
//! this module builds its `E3Detector` values directly rather than
//! maintaining a parallel type that would only ever get converted into one.
//! `id`/`entries`/`exits`/`icon_position` are set here; `file` (SUMO
//! requires the attribute, but the destination path is
//! [`crate::zone_output`]'s concern, not this module's) is left empty and
//! patched in before writing — see that module's own docs.
//!
//! Zones are grouped — and identified — by **movement**: two lanes belong
//! to the same zone only if they carry traffic from the same source edge in
//! the same turn direction(s). A junction with independent signals for
//! "straight" and "right turn" still gets two zones, not one, but that now
//! falls out of the lanes turning differently, not of the signal program
//! giving them different characters.
//!
//! This is deliberately decoupled from `tlLogic`. An earlier version grouped
//! (and named) zones by shared signal head — lanes that carry the same
//! character in every phase, in program order — but a signal program is
//! edited far more often than the road itself: reordering or adding a phase
//! left the junction physically untouched yet changed which lanes counted as
//! sharing a head, and the identity built on that reshuffled with it. Edge
//! and turn direction are geometry, not program, so [`vehicle_zones`] never
//! reads what state a phase assigns — only whether a connection's
//! `linkIndex` resolves at all, which is still the test for "is this
//! genuinely signal-controlled".
//!
//! Each zone spans the full length of its underlying lane by default: the
//! exit boundary sits at the lane's end (the stop line) and the entry at
//! its start — kept exactly as before *and* extended, via
//! [`extended_entry_lanes`], with one more entry gate on every predecessor
//! lane that feeds unambiguously into that start, and their own
//! predecessors in turn, for as long as each step is a real 1:1 hand-off
//! (a predecessor with exactly one outgoing connection, so *all* of its own
//! traffic is headed here) rather than a fork — additive, not a
//! replacement, so a vehicle spawned directly on the original short lane
//! (never having driven the extended approach at all) is still detected
//! exactly as it always was. A short lane is real
//! Barcelona `netconvert` output more often than not — a turn pocket or a
//! stub `netconvert` split off right at a junction, sometimes under a
//! metre long — and without this, a zone's own reported size would be
//! capped at that stub's own tiny length even though the traffic actually
//! queuing for it plainly extends much further back, all the way to the
//! last real fork in the road. This is a maximization, applied to every
//! *vehicle* zone rather than only ones that look suspiciously short: a
//! longer lane with a long, fork-free approach of its own genuinely does
//! queue that far back too — but not to pedestrian zones at all (see
//! [`full_lane_boundaries`]'s own docs on `extend_backward`): a fork-free
//! stretch of sidewalk doesn't mean "committed to this crossing" the way a
//! fork-free stretch of road means "committed to this queue" for a driver,
//! since a pedestrian can stop, turn around, or peel off into a shop
//! anywhere along it. `max_zone_length` still caps the *innermost* lane's
//! own entry the way it always has (`length - max_zone_length`, clamped to
//! the lane's start); extension (where it applies at all) only kicks in
//! once that capped entry actually reaches position 0, i.e. there's
//! nothing for `max_zone_length` left to cut short.
//!
//! Each signal-controlled [`Connection`] names its own controlling program
//! directly (`connection.traffic_light`, SUMO's `tl` attribute) — that, not
//! the junction's own id, is what resolves a lane's phase-state sequence.
//! It has to be: SUMO can merge several physically adjacent junctions under
//! one shared program (`joinTLS`), in which case the program's id is a
//! combined one (`joinedS_<id>_<id>_..._#Nmore`, `#Nmore` truncating the
//! list once it gets long — the abbreviated member ids don't even appear in
//! the string), and none of the junctions it covers have a program of their
//! own named after them. Matching by the junction's id instead — this
//! module's first approach — silently produced zero waiting zones for every
//! joined junction: correct-looking code (a real lane, a real program,
//! individually plausible), wrong on any network that actually uses
//! `joinTLS`, which real-world SUMO networks (Barcelona among them) do
//! heavily. A lane whose connections don't resolve a controlling program at
//! all (an unsignalized approach, or `JunctionKind::TrafficLightUnregulated`
//! — a kind that legitimately has none) is skipped with a per-lane warning
//! rather than guessing.
//!
//! A joined program can also produce two *separate* movement groups
//! ([`group_key_for_lane`]'s own identity, `(from_edge, directions)`) for
//! what is, physically, one queue: a single upstream lane forks — at an
//! ordinary, unsignalized junction, not the cluster itself — into two
//! lanes that each land on a *different* member junction of the same
//! `joinTLS` cluster, both still governed by the identical combined
//! program and both still going the same direction on the far side.
//! Confirmed on real Barcelona data: `50926859#0` and `50926861#0`, two
//! lanes of "Avinguda de Salvador Espriu" forking off one upstream edge,
//! land 3.5m apart on two junctions joined under one `joinedS_...`
//! program — a driver in either lane queues for the exact same red light,
//! so reporting them as two independent "straight" zones is a modelling
//! artifact, not two real movements. [`merge_fork_sibling_groups`] folds
//! such siblings back into one zone (both lanes as entries *and* exits —
//! the queue really does span both) after every junction's own groups are
//! built, using exactly the same non-cryptographic signal `zone_generator`
//! already trusts elsewhere for this: matching turn direction plus a
//! shared *controlling program* (not just "governs the same physical
//! spot", which a large `joinTLS` cluster could satisfy for two genuinely
//! different streets) plus a shared *immediate* predecessor edge (proof
//! the two lanes are actually the same fork, not just coincidentally
//! agreeing on direction and program). Any one of the three alone isn't
//! enough — only together do they pin down "provably one fork, both
//! branches of which report to the same physical light".
//!
//! Pedestrian waiting zones follow the exact same movement identity: SUMO
//! already lists a walkingarea lane among a signalized junction's
//! `incLanes`, right alongside the vehicle lanes it shares the junction
//! with, and gives the walkingarea's connection into the crossing the same
//! `tl`/`linkIndex` a vehicle connection would get (leaving the crossing
//! again is unconstrained, so that connection never resolves a program and
//! plays no part in grouping). [`group_key_for_lane`] and
//! [`full_lane_boundaries`] therefore apply completely unchanged — only
//! which lanes qualify ([`is_pedestrian_only`] instead of its negation) and
//! what marks the resulting `E3Detector` as a pedestrian zone
//! (`detectPersons="walk"`, an `_ped`-suffixed id) differ, in
//! [`pedestrian_zones`] itself. The zone's lane is the walkingarea
//! *before* the crossing, not the crossing itself: a pedestrian on the
//! crossing is actively walking across, not waiting for it.

use anstream::eprintln;
use anstyle::{AnsiColor, Style};
use std::collections::{HashMap, HashSet};
use sumo_types::additional::domain::{DetectorGate, DetectorId, E3Detector, LanePosition, LaneRef, PersonMode};
use sumo_types::domain::{
    Connection, ConnectionDirection, EdgeFunction, EdgeId, Junction, JunctionId, JunctionKind,
    LaneId, LaneIndex, LinkIndex, Network, TrafficLightId, TrafficLightProgram,
};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

/// Styles the "warning:" prefix on this module's own messages the way
/// `cargo`/`rustc` style theirs — bold yellow — so a skipped junction/lane
/// reads as a warning at a glance rather than blending into the rest of the
/// line. `{WARNING}` switches it on, `{WARNING:#}` (the alternate format)
/// resets it — `anstyle::Style`'s own `Display` impl.
///
/// The `eprintln!` these are used with is [`anstream`]'s, shadowing
/// `std`'s: it strips the escape codes when stderr isn't a terminal
/// (redirected to a file, `NO_COLOR` set, piped into another program, ...),
/// so non-interactive output isn't full of raw `\x1b[...m` sequences.
const WARNING: Style = AnsiColor::Yellow.on_default().bold();

/// Junction kinds that control right-of-way with a traffic light, i.e. the
/// ones where vehicles queue up waiting for a green phase. Rail-specific
/// kinds ([`JunctionKind::RailSignal`]) are deliberately excluded: they
/// don't carry road traffic.
fn is_traffic_light(kind: JunctionKind) -> bool {
    matches!(
        kind,
        JunctionKind::TrafficLight
            | JunctionKind::TrafficLightUnregulated
            | JunctionKind::TrafficLightRightOnRed
    )
}

/// A lane's length together with whether it's restricted to pedestrians
/// only — used to exclude sidewalk/walkingarea lanes that SUMO lists among
/// a junction's `incLanes` alongside the real vehicle lanes, straight from
/// SUMO's own vClass permissions (`Lane::is_pedestrian_only`) instead of
/// the edge's `function` label.
struct LaneInfo {
    length: Length,
    pedestrian_only: bool,
}

/// Whether `link_index` indexes into every phase of `program` — i.e.
/// whether the connection naming it is genuinely resolvable. This is the
/// only thing grouping still needs from the program; it never reads what
/// state any phase actually assigns (see the module docs).
fn link_index_in_range(program: &TrafficLightProgram, link_index: LinkIndex) -> bool {
    let Ok(index) = usize::try_from(link_index.0) else {
        return false;
    };
    program
        .phases
        .iter()
        .all(|phase| phase.state.chars().nth(index).is_some())
}

/// Generates vehicle waiting zones for every traffic-light junction in
/// `network`. A lane whose connections resolve no controlling program is
/// skipped, with a warning printed to stderr (see the module docs for why
/// that's a per-lane check, not a per-junction one).
///
/// `max_zone_length` caps how far each zone's entry extends from the stop
/// line; `None` means the full lane, as before.
///
/// `stop_at_complex_intersections`: see [`extended_entry_lanes`]'s own docs
/// on the stopping condition it adds.
pub fn generate(
    network: &Network,
    max_zone_length: Option<Length>,
    stop_at_complex_intersections: bool,
) -> Vec<E3Detector> {
    let lanes: HashMap<&LaneId, LaneInfo> = network
        .edges
        .iter()
        .flat_map(|edge| &edge.lanes)
        .map(|lane| {
            (
                &lane.id,
                LaneInfo {
                    length: lane.length,
                    pedestrian_only: lane.is_pedestrian_only(),
                },
            )
        })
        .collect();

    // Needed to resolve a `Connection`'s `(edge, lane index)` pair (how
    // lanes reference each other) into the actual `LaneId` the rest of the
    // domain model — and `lanes` above — keys off.
    let lane_ids_by_edge_and_index: HashMap<(&EdgeId, LaneIndex), &LaneId> = network
        .edges
        .iter()
        .flat_map(|edge| {
            edge.lanes
                .iter()
                .map(move |lane| ((&edge.id, lane.index), &lane.id))
        })
        .collect();

    // The reverse of the edge -> lanes relationship `network.edges` already
    // encodes directly — which edge a given lane belongs to. Only needed by
    // `merge_fork_sibling_groups` (via `predecessor_edges`), to turn a
    // predecessor *lane* (what `predecessors_by_lane` walks in) into the
    // predecessor *edge* two fork siblings need to share (see the module
    // docs) — a fork's two branches are almost always two different lanes
    // of that one shared upstream edge, not the same lane.
    let edge_by_lane: HashMap<&LaneId, &EdgeId> = network
        .edges
        .iter()
        .flat_map(|edge| edge.lanes.iter().map(move |lane| (&lane.id, &edge.id)))
        .collect();

    // Every edge SUMO itself generated to model the physical path *through*
    // a junction (a turn's curve, e.g. `:12913883279_0`) rather than a real
    // stretch of road — excluded below from every connection index this
    // function builds. `.net.xml` already encodes the useful part of that
    // path — one real edge to the next — directly on the real-to-real
    // connection itself (SUMO's own `via` attribute names the internal
    // lane, but the connection endpoints stay real), and *also* emits the
    // internal lane's own connections to/from it as further, separate
    // entries; walking through the latter instead would mean
    // `extended_entry_lanes` doesn't stop at the near edge of the junction
    // behind this one, but threads all the way through *its* own internal
    // turning geometry too — which real junctions cram at close quarters,
    // so two zones extended through different turns of the very same
    // upstream junction routinely ended up with wildly overlapping
    // polygons before this filter existed.
    let internal_edges: HashSet<&EdgeId> = network
        .edges
        .iter()
        .filter(|edge| edge.function == EdgeFunction::Internal)
        .map(|edge| &edge.id)
        .collect();
    let is_real_to_real =
        |c: &&Connection| !internal_edges.contains(&c.from_edge) && !internal_edges.contains(&c.to_edge);

    // Every connection originating from a given lane, indexed once up
    // front. Without this, grouping lanes by signal (see
    // `group_key_for_lane`) would re-scan every connection in the network
    // for every lane — quadratic in network size, and prohibitively slow
    // on a real city-scale network (multiple minutes on ~10k junctions).
    let mut connections_by_from_lane: HashMap<&LaneId, Vec<&Connection>> = HashMap::new();
    for connection in network.connections.iter().filter(is_real_to_real) {
        if let Some(&lane_id) =
            lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane))
        {
            connections_by_from_lane
                .entry(lane_id)
                .or_default()
                .push(connection);
        }
    }

    // The reverse of `connections_by_from_lane`: every lane that feeds
    // directly into a given lane. Needed by `extended_entry_lanes` to walk
    // a zone's entry backward (see the module docs); built once up front
    // for the same quadratic-blowup reason `connections_by_from_lane` is.
    let mut predecessors_by_lane: HashMap<&LaneId, HashSet<&LaneId>> = HashMap::new();
    for connection in network.connections.iter().filter(is_real_to_real) {
        if let (Some(&from_lane), Some(&to_lane)) = (
            lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane)),
            lane_ids_by_edge_and_index.get(&(&connection.to_edge, connection.to_lane)),
        ) {
            predecessors_by_lane.entry(to_lane).or_default().insert(from_lane);
        }
    }

    // The internal lane (if any) SUMO's own `via` names for a real-to-real
    // hop `(from, to)` — the physical curve *through* the junction between
    // them, excluded above from `predecessors_by_lane` itself (walking it
    // as its own hop is exactly what let `extended_entry_lanes` wander
    // into a different junction's own turning geometry — see the module
    // docs), but still real ground the zone's own polygon needs to cover:
    // without it, `geojson_output` draws the lane on each side of a
    // junction as its own disconnected rectangle, visibly not touching,
    // even though they're the same zone. `extended_entry_lanes` adds an
    // entry for it alongside `from` itself for every hop it actually
    // walks, purely to fill that gap — it plays no part in the walk's own
    // stop/continue decision, which stays entirely about the real lanes on
    // either side of it.
    let mut via_lane_between: HashMap<(&LaneId, &LaneId), &LaneId> = HashMap::new();
    for connection in network.connections.iter().filter(is_real_to_real) {
        if let (Some(&from_lane), Some(&to_lane), Some(via)) = (
            lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane)),
            lane_ids_by_edge_and_index.get(&(&connection.to_edge, connection.to_lane)),
            connection.via.as_ref(),
        ) {
            via_lane_between.insert((from_lane, to_lane), via);
        }
    }

    // Every lane that's itself someone's own signal-controlled approach —
    // a real `tl`/`linkIndex` connection out of it, at any junction, not
    // just the one `generate` happens to be building this particular
    // zone for. `extended_entry_lanes` never walks through one: it
    // already has (or, once `generate` gets to its own junction, will
    // have) a waiting zone of its own, and a lane can only ever be one
    // zone's own controlled approach — extending through it here would
    // draw a rectangle directly on top of that zone's own, not adjacent
    // to it. This is the same idea as excluding internal edges above,
    // just at the *other* junction's real, named approach lane rather
    // than its internal turning geometry.
    let signal_controlled_lanes: HashSet<&LaneId> = network
        .connections
        .iter()
        .filter(is_real_to_real)
        .filter(|connection| connection.traffic_light.is_some() && connection.link_index.is_some())
        .filter_map(|connection| {
            lane_ids_by_edge_and_index.get(&(&connection.from_edge, connection.from_lane))
        })
        .copied()
        .collect();

    // A `tlLogic` id can repeat (multiple programs sharing one id, e.g. a
    // normal one and a night-time one); keep the first one encountered, in
    // file order. Keyed by whatever `connection.traffic_light` names —
    // which, for a joined program, is the combined `joinedS_...` id, not
    // any one junction's own id (see the module docs).
    let mut programs_by_id: HashMap<&TrafficLightId, &TrafficLightProgram> = HashMap::new();
    for program in &network.traffic_light_programs {
        programs_by_id.entry(&program.id).or_insert(program);
    }

    // Every real edge's own start junction — needed only alongside
    // `stop_at_complex_intersections` below. A connection's own signal sits
    // at the junction joining its `from_edge` to its `to_edge`, which is
    // that `to_edge`'s own start (equivalently, `from_edge`'s own end —
    // same junction, named from whichever side is convenient); a
    // predecessor's connection into a given `lane_id` always lands at
    // `lane_id`'s own edge's start, by definition of "predecessor", so this
    // is also exactly what `extended_entry_lanes` needs to name the
    // junction it would be crossing into on its way to a given predecessor.
    let edge_start_junction: HashMap<&EdgeId, &JunctionId> = network
        .edges
        .iter()
        .filter_map(|edge| Some((&edge.id, edge.from.as_ref()?)))
        .collect();

    // The same lookup, keyed by lane instead of edge — what
    // `extended_entry_lanes` actually has on hand (a `LaneId`) when it
    // needs to name the junction a given predecessor's connection lands at.
    let edge_start_junction_by_lane: HashMap<&LaneId, &JunctionId> = network
        .edges
        .iter()
        .filter_map(|edge| Some((edge.from.as_ref()?, &edge.lanes)))
        .flat_map(|(junction, lanes)| lanes.iter().map(move |lane| (&lane.id, junction)))
        .collect();

    // Every junction that's part of some `joinTLS`-merged program spanning
    // more than one junction — a real, physically complex intersection SUMO
    // modeled as a cluster of closely-spaced nodes linked by near-zero-
    // length edges (see the module docs on why a joined program's id, not
    // any one junction's own id, is the only thing naming it). Grouping
    // every real-to-real connection by its own `tl` id and collecting each
    // group's distinct junctions finds this without ever having to parse
    // that id string — `joinedS_<id>_..._#Nmore` truncates the member list
    // once it's long, so the member ids it hides couldn't be recovered from
    // it even if this did try to parse it. A program naming only one
    // junction (the overwhelming majority — an ordinary single-junction
    // signal) never contributes anything here, so this changes nothing
    // about the ordinary case.
    let mut junctions_by_tl_id: HashMap<&TrafficLightId, HashSet<&JunctionId>> = HashMap::new();
    for connection in network.connections.iter().filter(is_real_to_real) {
        if let (Some(tl_id), Some(&junction)) =
            (&connection.traffic_light, edge_start_junction.get(&connection.to_edge))
        {
            junctions_by_tl_id.entry(tl_id).or_default().insert(junction);
        }
    }
    let complex_intersection_junctions: HashSet<&JunctionId> = junctions_by_tl_id
        .values()
        .filter(|junctions| junctions.len() > 1)
        .flatten()
        .copied()
        .collect();

    let graph = ConnectivityGraph {
        lane_ids_by_edge_and_index: &lane_ids_by_edge_and_index,
        connections_by_from_lane: &connections_by_from_lane,
        predecessors_by_lane: &predecessors_by_lane,
        signal_controlled_lanes: &signal_controlled_lanes,
        via_lane_between: &via_lane_between,
        lanes: &lanes,
        edge_start_junction_by_lane: &edge_start_junction_by_lane,
        complex_intersection_junctions: &complex_intersection_junctions,
        edge_by_lane: &edge_by_lane,
        stop_at_complex_intersections,
    };

    let traffic_light_junctions: Vec<&Junction> = network
        .junctions
        .iter()
        .filter(|junction| is_traffic_light(junction.kind))
        .collect();

    // Vehicle zones are collected across *every* traffic-light junction at
    // once, not one at a time — `merge_fork_sibling_groups` (see the
    // module docs) needs to see fork siblings that land on two different
    // junctions before it can recognize them as the same movement, which a
    // per-junction pass could never do regardless of what it checked
    // internally. Pedestrian zones have no such cross-junction case (a
    // walkingarea's own entry is never extended or merged — see
    // [`pedestrian_zones`]'s own docs), so they stay a straightforward
    // per-junction pass.
    let mut zones = vehicle_zones(&traffic_light_junctions, &lanes, &graph, &programs_by_id, max_zone_length);
    for junction in &traffic_light_junctions {
        zones.extend(pedestrian_zones(junction, &lanes, &graph, &programs_by_id, max_zone_length));
    }
    zones
}

/// The lane-connectivity indices [`full_lane_boundaries`] needs to walk a
/// zone's entry backward (see the module docs) and [`group_key_for_lane`]
/// already needed for grouping — bundled together so neither function's own
/// signature has to grow a parameter for each one individually.
struct ConnectivityGraph<'a> {
    lane_ids_by_edge_and_index: &'a HashMap<(&'a EdgeId, LaneIndex), &'a LaneId>,
    connections_by_from_lane: &'a HashMap<&'a LaneId, Vec<&'a Connection>>,
    predecessors_by_lane: &'a HashMap<&'a LaneId, HashSet<&'a LaneId>>,
    signal_controlled_lanes: &'a HashSet<&'a LaneId>,
    via_lane_between: &'a HashMap<(&'a LaneId, &'a LaneId), &'a LaneId>,
    /// Every lane's own length — [`extended_entry_lanes`]'s own budget
    /// accounting needs each predecessor's real length as it walks
    /// backward, the same source [`full_lane_boundaries`] already reads
    /// lengths from for every other lane.
    lanes: &'a HashMap<&'a LaneId, LaneInfo>,
    /// The junction a given lane's own edge starts at — only consulted when
    /// `stop_at_complex_intersections` is set (see
    /// [`extended_entry_lanes`]'s own docs on the stopping condition it
    /// adds).
    edge_start_junction_by_lane: &'a HashMap<&'a LaneId, &'a JunctionId>,
    /// Every junction that's part of some `joinTLS`-merged traffic light
    /// program spanning more than one junction (see [`generate`]'s own docs
    /// on how this is computed) — only consulted when
    /// `stop_at_complex_intersections` is set.
    complex_intersection_junctions: &'a HashSet<&'a JunctionId>,
    /// Every lane's own edge — [`predecessor_edges`]'s own lookup, turning
    /// a predecessor *lane* into the edge [`merge_fork_sibling_groups`]
    /// actually compares (see that function's own docs on why a fork's two
    /// branches are almost always two different lanes of one shared
    /// upstream edge, not the same lane).
    edge_by_lane: &'a HashMap<&'a LaneId, &'a EdgeId>,
    /// See [`extended_entry_lanes`]'s own docs on the stopping condition
    /// this gates. Off by default ([`crate::config::Config`]'s own docs on
    /// the CLI flag) — it trades away ever reaching a genuinely upstream
    /// signal beyond a complex intersection for guaranteed-simple geometry
    /// through it, and that trade isn't free everywhere it'd apply.
    stop_at_complex_intersections: bool,
}

/// Builds entry/exit detector gates for each lane in `lane_ids` that's known
/// to `lanes`. The exit always sits at the lane's end (the stop line), one
/// per lane in `lane_ids` — unchanged by extension, always the group's own
/// controlled lane. The entry normally sits at the lane's own start too,
/// but when `reach.extend_backward` is set, is instead one gate per lane
/// [`extended_entry_lanes`] finds walking backward from there (see the
/// module docs) whenever that start isn't itself capped short by
/// `reach.max_zone_length`: a cap narrow enough to keep the entry inside
/// `lane_id` itself already answers "how far back does this zone reach" on
/// its own, so extension only adds gates once there's nothing left for it
/// to cut short.
///
/// `extend_backward` is `false` for [`pedestrian_zones`]: a vehicle lane is
/// a one-way commitment — once a driver has taken it, `netconvert`'s own
/// routing means every metre back to the last real fork genuinely is the
/// same queue — but a walkingarea isn't. A pedestrian can stop, turn
/// around, or peel off into a shop at any point along a sidewalk, so
/// "unambiguous, fork-free" doesn't mean "committed to this crossing" the
/// way it does for a vehicle, and chaining across a whole block's worth of
/// fork-free sidewalk (common — a single stretch of pavement between two
/// corners rarely branches) produced a "waiting zone" that was really just
/// most of the block, several times too generous to mean anything.
///
/// Every gate is [`LanePosition::FromStart`]: a waiting zone's boundaries
/// are computed from a lane's own length, so there's never a reason to
/// express one as `FromEnd` instead.
///
/// Every gate also sets `friendlyPos`: an exit sits at exactly `length`,
/// the `.net.xml`'s own reported lane length, but netedit computes a
/// lane's *geometric* length from its shape, which can differ from that
/// attribute by the last handful of floating-point digits. Without
/// `friendlyPos`, netedit rejects a `pos` fractionally beyond what it
/// computes as "Invalid position over lane" — real Barcelona data hits
/// this. `friendlyPos` clamps a mismatch like that into range instead of
/// erroring, which is exactly what it exists for (see
/// `DetectorGate::friendly_position`'s own docs) and costs nothing when
/// the two lengths already agree.
fn full_lane_boundaries(
    lane_ids: &[LaneId],
    lanes: &HashMap<&LaneId, LaneInfo>,
    graph: &ConnectivityGraph<'_>,
    reach: EntryReach,
) -> (Vec<DetectorGate>, Vec<DetectorGate>) {
    let gate = |lane_id: &LaneId, position: Length| DetectorGate {
        lane: LaneRef(lane_id.0.clone()),
        position: LanePosition::FromStart(position),
        friendly_position: Some(true),
    };

    let mut entries = Vec::with_capacity(lane_ids.len());
    let mut exits = Vec::with_capacity(lane_ids.len());

    for lane_id in lane_ids {
        let Some(length) = lanes.get(lane_id).map(|info| info.length) else {
            continue;
        };
        exits.push(gate(lane_id, length));

        let entry_position = match reach.max_zone_length {
            Some(max) if max < length => length - max,
            _ => Length::new::<meter>(0.0),
        };
        if entry_position > Length::new::<meter>(0.0) {
            entries.push(gate(lane_id, entry_position));
            continue;
        }

        // The lane's own start always gets a gate — unchanged from before
        // extension existed, so a vehicle spawned directly on `lane_id`
        // (never having driven through any ancestor) is still detected —
        // plus, for a vehicle zone, one more for every ancestor
        // `extended_entry_lanes` can reach, covering the *whole*
        // unambiguous approach with no gap for `geojson_output`'s own
        // polygon to fall into.
        entries.push(gate(lane_id, Length::new::<meter>(0.0)));
        if reach.extend_backward {
            let mut visited = HashSet::new();
            let budget = reach.max_zone_length.unwrap_or(Length::new::<meter>(DEFAULT_EXTENSION_METERS));
            for ancestor in extended_entry_lanes(lane_id, graph, &mut visited, budget) {
                entries.push(gate(ancestor, Length::new::<meter>(0.0)));
            }
        }
    }

    (entries, exits)
}

/// Every distinct lane a connection targets from `lane_id` — used only to
/// tell "exactly one" (safe to walk through backward — see
/// [`extended_entry_lanes`]) from "more than one" (a real fork); the
/// specific count past 1 is never otherwise meaningful.
fn successor_lane_count(lane_id: &LaneId, graph: &ConnectivityGraph<'_>) -> usize {
    graph
        .connections_by_from_lane
        .get(lane_id)
        .map(|connections| {
            connections
                .iter()
                .filter_map(|connection| {
                    graph
                        .lane_ids_by_edge_and_index
                        .get(&(&connection.to_edge, connection.to_lane))
                })
                .collect::<HashSet<_>>()
                .len()
        })
        .unwrap_or(0)
}

/// Every predecessor lane reachable from `lane_id` by walking backward
/// through connections that don't fork and aren't already spoken for —
/// stopping at whichever comes first of:
///
/// - A predecessor with more than one outgoing connection: a real fork, so
///   only *some* of its traffic is actually headed here, not all of it —
///   walking through it would silently claim traffic that's headed
///   somewhere else as part of this zone.
/// - A predecessor that's itself a signal-controlled approach lane at some
///   junction (any junction, not only the one this zone belongs to) — it
///   already has its own waiting zone, and extending through it would draw
///   a rectangle right on top of that zone's own rather than next to it
///   (`signal_controlled_lanes`, checked via `graph`).
/// - (only when `graph.stop_at_complex_intersections` is set) `lane_id`
///   itself already sits at a junction that's part of a `joinTLS`-merged
///   program spanning more than one junction
///   (`graph.complex_intersection_junctions`) — a real, physically complex
///   intersection SUMO modeled as a cluster of closely-spaced nodes linked
///   by near-zero-length edges. Nothing inside that cluster individually
///   looks like a fork or an existing signal (`successor_lane_count` is 1,
///   `signal_controlled_lanes` doesn't contain it), so without this,
///   extension walks straight through the whole cluster, unioning dozens of
///   tiny, oddly-angled lane buffers into one zone polygon — confirmed on
///   real Barcelona data (`203480266#0_straight`), where this produced a
///   ring with over a dozen self-intersections. This is `false` by default
///   (`crate::config::Config`'s own docs on the CLI flag it comes from): it
///   trades away ever reaching a genuinely upstream signal beyond the
///   cluster for guaranteed-simple geometry through it, and that trade
///   isn't free everywhere it'd apply — an ordinary single-junction signal
///   never has more than one junction in its own program, so this never
///   changes anything about the overwhelming majority of zones regardless
///   of the flag.
///
/// Every lane along the way up to (but not past) either stopping point is
/// included, not only the furthest-back one — [`full_lane_boundaries`]
/// adds an entry gate for each, so the zone's approach is covered with no
/// gap for `geojson_output`'s own polygon to fall into. That includes the
/// internal lane (if any — `graph.via_lane_between`) physically bridging
/// each hop: real edges only meet at a junction *through* the curve of
/// their own internal geometry, so a hop from one real lane to the next
/// without it would leave a real, visible gap between the two — the same
/// zone's own polygon looking like two disconnected ones. It's included
/// purely to fill that gap, never consulted for the walk's own stop/go
/// decision, which stays entirely about the real lane on each side of it.
/// Doesn't include `lane_id` itself — every lane always gets an entry gate
/// on its own start regardless of extension, so [`full_lane_boundaries`]
/// adds that one separately.
///
/// `visited` guards against a cycle — a single-lane roundabout with no
/// other connections is the only real-world shape that could otherwise
/// loop forever — by treating a lane already on the current path as
/// nothing further to add, stopping the walk there instead.
///
/// `remaining_budget` is the third stopping condition, and the one that
/// actually bounds a real walk rather than just a pathological one: a
/// long, straight street whose only cross traffic comes from minor,
/// unsignalized side streets (never adding a second outgoing connection to
/// the *through* lane itself, so `successor_lane_count` never trips) has
/// no fork and no signal to stop at for block after block — confirmed on
/// real Barcelona data, where this walked clean across a dozen-plus
/// unrelated blocks before this budget existed, several hundred metres
/// past anything a real queue could be. A predecessor that would exceed
/// what's left is skipped outright (not partially included): every lane
/// this function returns gets a *whole*-lane entry gate
/// ([`full_lane_boundaries`]'s own docs), so there's no way to claim only
/// part of one.
fn extended_entry_lanes<'a>(
    lane_id: &'a LaneId,
    graph: &ConnectivityGraph<'a>,
    visited: &mut HashSet<&'a LaneId>,
    remaining_budget: Length,
) -> Vec<&'a LaneId> {
    if remaining_budget <= Length::new::<meter>(0.0) || !visited.insert(lane_id) {
        return Vec::new();
    }
    if graph.stop_at_complex_intersections
        && graph
            .edge_start_junction_by_lane
            .get(lane_id)
            .is_some_and(|junction| graph.complex_intersection_junctions.contains(junction))
    {
        return Vec::new();
    }
    let Some(predecessors) = graph.predecessors_by_lane.get(lane_id) else {
        return Vec::new();
    };

    let mut extended = Vec::new();
    for &predecessor in predecessors {
        let already_has_its_own_zone = graph.signal_controlled_lanes.contains(predecessor);
        let predecessor_length = graph.lanes.get(predecessor).map(|info| info.length);
        let Some(predecessor_length) = predecessor_length else { continue };
        if !already_has_its_own_zone
            && successor_lane_count(predecessor, graph) == 1
            && predecessor_length < remaining_budget
        {
            if let Some(&via) = graph.via_lane_between.get(&(predecessor, lane_id)) {
                extended.push(via);
            }
            extended.push(predecessor);
            extended.extend(extended_entry_lanes(
                predecessor,
                graph,
                visited,
                remaining_budget - predecessor_length,
            ));
        }
    }
    extended
}

/// Why [`group_key_for_lane`] couldn't compute a group for a lane. Named
/// (rather than a bare `None`) so the warning at the call site says which
/// of three genuinely different things actually happened, instead of one
/// generic "no signal-controlled connection" for all of them — the
/// difference matters: only [`Self::LinkIndexOutOfRange`] points at a
/// malformed `.net.xml`, the other two are ordinary network shapes.
#[derive(Debug, PartialEq)]
enum UngroupedReason {
    /// The lane has no outgoing connection in the network at all. Real
    /// networks have these — a lane SUMO's own import kept geometrically
    /// but never gave a legal movement — there's nothing to group either
    /// way.
    NoOutgoingConnection,
    /// The lane has outgoing connections, but none of them are
    /// signal-controlled: no `tl` attribute at all (e.g. a connection into
    /// a walkingarea), or the `tl` they do name isn't any program this
    /// file defines — which, before `connection.traffic_light` replaced
    /// matching by the junction's own id, was silently the common case for
    /// every `joinTLS`-merged junction (see the module docs).
    NoResolvableProgram,
    /// A connection named a real program, but its `linkIndex` doesn't
    /// index into any of that program's phases — the program's `state`
    /// strings are shorter than the network's connections claim. Unlike
    /// the other two, this *is* a malformed-input signal.
    LinkIndexOutOfRange {
        program: TrafficLightId,
        index: LinkIndex,
    },
}

impl std::fmt::Display for UngroupedReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NoOutgoingConnection => {
                write!(f, "no outgoing connection in the network at all")
            }
            Self::NoResolvableProgram => write!(
                f,
                "none of its connections are signal-controlled (no `tl` naming a program this file defines)"
            ),
            Self::LinkIndexOutOfRange { program, index } => write!(
                f,
                "connection's linkIndex {index} is out of range for tlLogic \"{program}\"'s phases"
            ),
        }
    }
}

/// The source edge and the combined turn directions (sorted, deduplicated)
/// of every signal-controlled connection originating from `lane_id` — the
/// movement side of a waiting zone's identity. A lane can carry more than
/// one direction (e.g. a shared straight+right lane), in which case it
/// forms its own group distinct from lanes with a single movement — it
/// never changes state independently of either.
///
/// Every connection from one lane shares the same source edge by
/// construction (`connections_by_from_lane` is built from it), so the first
/// one resolved is as good as any.
///
/// Each connection's controlling program is looked up by its own
/// `traffic_light` id in `programs` — not assumed to be a single program
/// shared by the whole lane — so a lane whose connections are split across
/// more than one program (unusual, but the data model doesn't rule it out)
/// still gets a correct, if unusual, key instead of a wrong one silently
/// computed against the wrong program. The program itself is consulted only
/// to confirm a connection's `linkIndex` actually resolves — never to read
/// what a phase assigns it, which is exactly what made the old scheme
/// unstable across a signal-program edit.
///
/// `Err` if `lane_id` has no connection that resolves a controlling program
/// at all — the caller should skip the lane (and warn, naming the
/// [`UngroupedReason`]) rather than lump it in. When more than one
/// connection fails, [`UngroupedReason::LinkIndexOutOfRange`] wins over
/// [`UngroupedReason::NoResolvableProgram`] in the reported reason: it's
/// the more specific, more actionable diagnosis of the two.
/// A waiting zone's movement identity before it's formatted into an id
/// string (see [`movement_id`]): the source edge and the combined turn
/// direction(s) a lane's signal-controlled connections carry.
type MovementKey = (EdgeId, Vec<ConnectionDirection>);

fn group_key_for_lane(
    lane_id: &LaneId,
    connections_by_from_lane: &HashMap<&LaneId, Vec<&Connection>>,
    programs: &HashMap<&TrafficLightId, &TrafficLightProgram>,
) -> Result<MovementKey, UngroupedReason> {
    let connections = connections_by_from_lane
        .get(lane_id)
        .ok_or(UngroupedReason::NoOutgoingConnection)?;

    let mut from_edge = None;
    let mut directions = Vec::new();
    let mut out_of_range = None;

    for connection in connections {
        let (Some(tl_id), Some(link_index)) =
            (connection.traffic_light.as_ref(), connection.link_index)
        else {
            continue;
        };
        let Some(program) = programs.get(tl_id) else {
            continue;
        };

        if link_index_in_range(program, link_index) {
            from_edge.get_or_insert_with(|| connection.from_edge.clone());
            directions.push(connection.direction);
        } else {
            out_of_range.get_or_insert_with(|| (tl_id.clone(), link_index));
        }
    }

    match from_edge {
        Some(from_edge) => {
            directions.sort();
            directions.dedup();
            Ok((from_edge, directions))
        }
        None => Err(match out_of_range {
            Some((program, index)) => UngroupedReason::LinkIndexOutOfRange { program, index },
            None => UngroupedReason::NoResolvableProgram,
        }),
    }
}

/// Groups `lane_ids` — every incoming lane of `junction` the caller has
/// already filtered to the ones it cares about — by movement (see
/// [`group_key_for_lane`]), skipping (and warning about) any that don't
/// resolve one. Shared by [`vehicle_zones`] and [`pedestrian_zones`]:
/// grouping is identical for both, only which lanes qualify and what
/// becomes of each group differs.
///
/// Sorted only for reproducible output ordering between runs — the id
/// itself no longer comes from this position, so nothing about identity
/// depends on it.
fn group_lanes<'a>(
    junction: &Junction,
    lane_ids: impl Iterator<Item = &'a LaneId>,
    connections_by_from_lane: &HashMap<&LaneId, Vec<&Connection>>,
    programs: &HashMap<&TrafficLightId, &TrafficLightProgram>,
) -> Vec<(MovementKey, Vec<LaneId>)> {
    let mut groups: HashMap<MovementKey, Vec<LaneId>> = HashMap::new();

    for lane_id in lane_ids {
        match group_key_for_lane(lane_id, connections_by_from_lane, programs) {
            Ok(key) => groups.entry(key).or_default().push(lane_id.clone()),
            Err(reason) => eprintln!(
                "{WARNING}warning:{WARNING:#} lane \"{lane_id}\" at junction \"{}\" skipped: {reason}",
                junction.id
            ),
        }
    }

    let mut groups: Vec<_> = groups.into_iter().collect();
    groups.sort_by(|(a, _), (b, _)| a.cmp(b));
    groups
}

/// [`extended_entry_lanes`]'s own backward-walk budget when the caller
/// doesn't set `max_zone_length` at all. Unlike the innermost lane's own
/// entry — which really is meant to reach the lane's true start when
/// nothing caps it, per `Config::max_zone_length`'s own CLI docs — an
/// *unbounded* walk through however many fork-free blocks a real street
/// happens to have was never a real queue: confirmed on real Barcelona
/// data walking clean across more than a dozen unrelated blocks before
/// this existed.
///
/// A first pass at this used 300m, reasoning from `engine.md`'s own
/// "arrive in time not to stop" range (~125m at 30 km/h, ~210m at 50
/// km/h) — but that range is the length a waiting zone would *need* to
/// guarantee that property, not a realistic queue's own typical length,
/// and 300m of accumulated real edges still read as several unrelated
/// blocks stitched into one zone on a real map, not a single coherent
/// waiting area. 120m instead: `control_loop::runner::DEFAULT_CONFIG`'s
/// own vehicle `spacing` (7.5m) times a generously long real queue (16
/// vehicles) — long enough that a genuinely short stub lane still gets a
/// meaningfully extended approach, short enough that it stays one
/// recognizable piece of road.
const DEFAULT_EXTENSION_METERS: f64 = 120.0;

/// `max_zone_length`/`extend_backward` together — bundled so
/// [`zone_from_group`] and [`full_lane_boundaries`] each take one
/// parameter for "how far back does an entry reach" instead of two.
#[derive(Clone, Copy)]
struct EntryReach {
    max_zone_length: Option<Length>,
    /// `false` for a pedestrian zone — see [`full_lane_boundaries`]'s own
    /// docs for why a fork-free stretch of sidewalk doesn't mean
    /// "committed to this crossing" the way a fork-free stretch of road
    /// means "committed to this queue" for a vehicle.
    extend_backward: bool,
}

/// Builds the `E3Detector` for one movement group, or `None` if none of its
/// lanes are known to `lanes` (mirrors [`full_lane_boundaries`]'s own
/// empty-entries case). Shared by [`vehicle_zones`] and
/// [`pedestrian_zones`]; `id` and `detect_persons` are the only things that
/// actually differ between the two.
fn zone_from_group(
    id: String,
    lane_ids: &[LaneId],
    lanes: &HashMap<&LaneId, LaneInfo>,
    graph: &ConnectivityGraph<'_>,
    junction: &Junction,
    reach: EntryReach,
    detect_persons: Vec<PersonMode>,
) -> Option<E3Detector> {
    let (entries, exits) = full_lane_boundaries(lane_ids, lanes, graph, reach);
    if entries.is_empty() {
        return None;
    }

    Some(E3Detector {
        id: DetectorId(id),
        entries,
        exits,
        // Not this module's concern — see the module docs.
        // `zone_output::write` fills this in before serializing.
        file: String::new(),
        icon_position: Some(junction.position),
        period: None,
        name: None,
        speed_threshold: None,
        time_threshold: None,
        open_entry: None,
        detect_persons,
    })
}

/// One vehicle movement group before it becomes an `E3Detector`: which
/// junction it was grouped at, its movement identity, and its lanes. Kept
/// as a value (rather than folded straight into [`zone_from_group`] the
/// way [`pedestrian_zones`] does) only so [`merge_fork_sibling_groups`]
/// can see every junction's groups side by side first — see the module
/// docs on why that has to happen across junctions, not within one.
struct VehicleGroup<'a> {
    /// Every source edge this group covers — exactly one unless
    /// [`merge_fork_sibling_groups`] folded fork siblings together, in
    /// which case one per sibling, sorted, so the merged id is the same
    /// regardless of which junction happened to be visited first.
    from_edges: Vec<EdgeId>,
    directions: Vec<ConnectionDirection>,
    lane_ids: Vec<LaneId>,
    /// The junction whose `incLanes` this group was built from — the
    /// zone's own `icon_position`. For a merged group, the sibling with the
    /// smallest `from_edge` wins, again purely for determinism.
    junction: &'a Junction,
}

/// The vehicle waiting zones queued on every junction in `junctions`'
/// incoming lanes, one per distinct movement — with fork siblings landing
/// on different junctions of one `joinTLS` cluster folded into a single
/// zone first (see [`merge_fork_sibling_groups`]).
fn vehicle_zones(
    junctions: &[&Junction],
    lanes: &HashMap<&LaneId, LaneInfo>,
    graph: &ConnectivityGraph<'_>,
    programs: &HashMap<&TrafficLightId, &TrafficLightProgram>,
    max_zone_length: Option<Length>,
) -> Vec<E3Detector> {
    let mut groups = Vec::new();
    for &junction in junctions {
        // sidewalk/walkingarea lanes feeding into the junction are
        // pedestrians' concern (`pedestrian_zones`), not vehicles'.
        let lane_ids = junction
            .incoming_lanes
            .iter()
            .filter(|lane_id| !lanes.get(lane_id).is_some_and(|info| info.pedestrian_only));
        for ((from_edge, directions), lane_ids) in
            group_lanes(junction, lane_ids, graph.connections_by_from_lane, programs)
        {
            groups.push(VehicleGroup { from_edges: vec![from_edge], directions, lane_ids, junction });
        }
    }

    merge_fork_sibling_groups(groups, graph)
        .into_iter()
        .filter_map(|group| {
            zone_from_group(
                merged_movement_id(&group.from_edges, &group.directions),
                &group.lane_ids,
                lanes,
                graph,
                group.junction,
                EntryReach { max_zone_length, extend_backward: true },
                Vec::new(),
            )
        })
        .collect()
}

/// Every real edge that feeds directly into any lane of `lane_ids` —
/// the "immediate predecessor edge" side of [`merge_fork_sibling_groups`]'s
/// own three-way check. Edges rather than lanes: a fork's two branches
/// are almost always two different lanes of one shared upstream edge
/// (confirmed on the real Barcelona case the module docs describe:
/// `50926865#6` lane 1 feeds `50926859#0`, lane 2 feeds `50926861#0`),
/// so comparing predecessor *lanes* would never find them equal.
///
/// A predecessor that is itself a signal-controlled approach is left out:
/// the module docs' own criterion is a fork "at an ordinary, unsignalized
/// junction, not the cluster itself" — two lanes forking off an already
/// signalized approach queue at *that* light first, each behind its own
/// downstream one, and folding them together there would draw one zone
/// across two genuinely separate queues.
fn predecessor_edges<'a>(lane_ids: &[LaneId], graph: &ConnectivityGraph<'a>) -> HashSet<&'a EdgeId> {
    lane_ids
        .iter()
        .filter_map(|lane_id| graph.predecessors_by_lane.get(lane_id))
        .flatten()
        .filter(|predecessor| !graph.signal_controlled_lanes.contains(*predecessor))
        .filter_map(|predecessor| graph.edge_by_lane.get(predecessor).copied())
        .collect()
}

/// Every traffic-light program controlling some connection out of a lane
/// in `lane_ids` — the "shared controlling program" side of
/// [`merge_fork_sibling_groups`]'s own three-way check.
fn controlling_programs<'a>(lane_ids: &[LaneId], graph: &ConnectivityGraph<'a>) -> HashSet<&'a TrafficLightId> {
    lane_ids
        .iter()
        .filter_map(|lane_id| graph.connections_by_from_lane.get(lane_id))
        .flatten()
        .filter_map(|connection| connection.traffic_light.as_ref())
        .collect()
}

/// Folds fork siblings back into one group — see the module docs for the
/// real Barcelona case this exists for. Two groups merge when *all three*
/// hold: identical turn directions, at least one controlling program in
/// common, and at least one immediate (unsignalized) predecessor edge in
/// common; merging is transitive, so three-way forks fold into one too.
/// Any group that matches nothing passes through untouched, which on an
/// ordinary single-junction signal is every group.
///
/// Output order is by the merged group's own sorted `from_edges`, so
/// `generate`'s own output stays reproducible run to run — the groups
/// arrive here in junction order, which is file order, but a merged group
/// belongs to two junctions at once and needs a single, stable place.
fn merge_fork_sibling_groups<'a>(groups: Vec<VehicleGroup<'a>>, graph: &ConnectivityGraph<'_>) -> Vec<VehicleGroup<'a>> {
    // Union-find over group indices, keyed by every (directions, program,
    // predecessor edge) triple a group can claim: two groups claiming the
    // same triple are siblings.
    let mut parent: Vec<usize> = (0..groups.len()).collect();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }

    let mut by_triple: HashMap<(&[ConnectionDirection], &TrafficLightId, &EdgeId), usize> = HashMap::new();
    for (index, group) in groups.iter().enumerate() {
        let predecessors = predecessor_edges(&group.lane_ids, graph);
        for program in controlling_programs(&group.lane_ids, graph) {
            for &predecessor in &predecessors {
                match by_triple.entry((group.directions.as_slice(), program, predecessor)) {
                    std::collections::hash_map::Entry::Vacant(slot) => {
                        slot.insert(index);
                    }
                    std::collections::hash_map::Entry::Occupied(slot) => {
                        let (a, b) = (root(&mut parent, *slot.get()), root(&mut parent, index));
                        if a != b {
                            parent[b] = a;
                        }
                    }
                }
            }
        }
    }

    let mut merged: HashMap<usize, VehicleGroup<'a>> = HashMap::new();
    for (index, group) in groups.into_iter().enumerate() {
        let key = root(&mut parent, index);
        match merged.entry(key) {
            std::collections::hash_map::Entry::Vacant(slot) => {
                slot.insert(group);
            }
            std::collections::hash_map::Entry::Occupied(mut slot) => {
                let existing = slot.get_mut();
                if group.from_edges < existing.from_edges {
                    existing.junction = group.junction;
                }
                existing.from_edges.extend(group.from_edges);
                existing.from_edges.sort();
                existing.from_edges.dedup();
                existing.lane_ids.extend(group.lane_ids);
            }
        }
    }

    let mut merged: Vec<VehicleGroup<'a>> = merged.into_values().collect();
    merged.sort_by(|a, b| (&a.from_edges, &a.directions).cmp(&(&b.from_edges, &b.directions)));
    merged
}

/// [`movement_id`] for a group covering one or more source edges: a
/// single edge gives exactly the id it always has; a merged fork-sibling
/// group joins its (sorted) edges with `+` in front of the shared
/// direction suffix, e.g. `50926859#0+50926861#0_straight`. A single id
/// for the single zone, rather than one sibling's id "winning" — either
/// choice would silently reuse an id that used to name a different,
/// narrower zone.
fn merged_movement_id(from_edges: &[EdgeId], directions: &[ConnectionDirection]) -> String {
    let directions_suffix = movement_id(&EdgeId(String::new()), directions);
    let edges = from_edges
        .iter()
        .map(|edge| edge.0.strip_prefix(':').unwrap_or(&edge.0))
        .collect::<Vec<_>>()
        .join("+");
    format!("{edges}{directions_suffix}")
}

/// The pedestrian waiting zones approaching `junction`'s signalized
/// crossings, one per distinct movement — the walkingarea(s) leading into a
/// given crossing, mirroring how [`vehicle_zones`] groups the lanes leading
/// into a given turn. See the module docs for why this needs nothing
/// pedestrian-specific beyond the lane filter and the two fields
/// [`zone_from_group`] takes.
///
/// `detectPersons="walk"` (below) is kept for what it's still good for —
/// marking a zone as pedestrian (`Zone::is_pedestrian` in `territory`, this
/// crate's own `E3Detector::detect_persons`) and giving `control_loop` a
/// real lane/edge and signal phase to drive off — not because SUMO's own
/// person-tracking on this attribute is trusted. It isn't: `MSE3Collector`'s
/// per-step person scan (`detectorUpdate` → `notifyMovePerson`) can
/// permanently miss a pedestrian who's already halted the first time it
/// reaches them (a real, still-open upstream SUMO defect — see
/// `traci::Client::edge_last_step_person_ids`'s own docs in `control_loop`
/// for the mechanism and a reproduction), so this detector's own reported
/// occupancy (`vehicleSum`, `meanSpeedWithin`, ... in its output XML, or
/// `multientryexit.getLastStepVehicleIDs` over TraCI) can silently
/// undercount — down to zero for a crossing people are demonstrably using.
/// `control_loop::runner::update_occupancy` already never asks this
/// detector who's inside it for that reason; it polls the walkingarea
/// edge's own live position (`Edge.getLastStepPersonIDs`) instead. Anything
/// new reading this attribute's own detection output should do the same,
/// not trust it directly.
///
/// Unlike a vehicle zone's, a pedestrian zone's own entry gates are never
/// backward-extended (`full_lane_boundaries`'s own `extend_backward` is
/// `false` here) — see that function's own docs for why a fork-free
/// stretch of sidewalk doesn't mean "committed to this crossing" the way a
/// fork-free stretch of road means "committed to this queue" for a
/// vehicle. The "real lane/edge" `control_loop` polls still comes from the
/// zone's *exit* side (`territory::zones::Zone::edge`, derived from
/// `E3Detector::exits`) rather than its entries regardless, on the same
/// general principle extension exists under: exits are always the group's
/// own controlled lane, entries aren't guaranteed to be.
fn pedestrian_zones(
    junction: &Junction,
    lanes: &HashMap<&LaneId, LaneInfo>,
    graph: &ConnectivityGraph<'_>,
    programs: &HashMap<&TrafficLightId, &TrafficLightProgram>,
    max_zone_length: Option<Length>,
) -> Vec<E3Detector> {
    let lane_ids = junction
        .incoming_lanes
        .iter()
        .filter(|lane_id| lanes.get(lane_id).is_some_and(|info| info.pedestrian_only));

    group_lanes(junction, lane_ids, graph.connections_by_from_lane, programs)
        .into_iter()
        .filter_map(|((from_edge, directions), lane_ids)| {
            zone_from_group(
                format!("{}_ped", movement_id(&from_edge, &directions)),
                &lane_ids,
                lanes,
                graph,
                junction,
                EntryReach { max_zone_length, extend_backward: false },
                vec![PersonMode::Walk],
            )
        })
        .collect()
}

/// A waiting zone's id: the source edge and its turn direction(s), joined
/// deterministically. Never the junction id (regeneration can change it)
/// and never a position in a sorted list (a signal-program edit can change
/// what that list contains without the road changing at all) — both were
/// the prior scheme's actual failure modes.
fn movement_id(from_edge: &EdgeId, directions: &[ConnectionDirection]) -> String {
    let directions = directions
        .iter()
        .map(|direction| direction_label(*direction))
        .collect::<Vec<_>>()
        .join("+");
    // SUMO spells an internal edge's id with a leading `:` — its own
    // marker that the edge is internal, not part of the name itself. A
    // walkingarea (the only kind of edge this function ever sees one of,
    // via `pedestrian_zones`) is one of these, and keeping the `:` in a
    // zone id derived from it doesn't just look wrong: `zone_output`
    // builds the detector's `file` attribute from this same id, and SUMO's
    // own `OutputDevice` factory reads any `:` in an output filename as a
    // `host:port` remote-socket spec, not literal text. Left in, this
    // produces a `.add.xml` real SUMO refuses to load ("Given port number
    // '...' is not numeric") — caught by `sumo_validates_output`'s
    // real-loader check, not by this crate's own unit tests, which never
    // shell out to `sumo` at all.
    let from_edge = from_edge.0.strip_prefix(':').unwrap_or(&from_edge.0);
    format!("{from_edge}_{directions}")
}

fn direction_label(direction: ConnectionDirection) -> &'static str {
    match direction {
        ConnectionDirection::Straight => "straight",
        ConnectionDirection::Turn => "turn",
        ConnectionDirection::TurnLeftHand => "turn_left_hand",
        ConnectionDirection::Left => "left",
        ConnectionDirection::Right => "right",
        ConnectionDirection::PartialLeft => "partial_left",
        ConnectionDirection::PartialRight => "partial_right",
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use sumo_types::domain::{
        ConnectionDirection, Edge, EdgeFunction, EdgeId, Junction, JunctionId, JunctionKind, Lane,
        LaneId, LaneIndex, LinkIndex, LinkState, Phase, Point, TrafficLightId, TrafficLightKind,
    };
    use sumo_types::uom::si::f64::{Time, Velocity};
    use sumo_types::uom::si::time::second;
    use sumo_types::uom::si::velocity::meter_per_second;

    fn lane(id: &str, length_m: f64) -> Lane {
        Lane {
            id: LaneId(id.into()),
            index: LaneIndex(0),
            speed: Velocity::new::<meter_per_second>(10.0),
            length: Length::new::<meter>(length_m),
            width: Length::new::<meter>(3.2),
            end_offset: Length::new::<meter>(0.0),
            shape: Default::default(),
            allow: vec![],
            disallow: vec![],
        }
    }

    fn indexed_lane(id: &str, index: usize, length_m: f64) -> Lane {
        Lane {
            index: LaneIndex(index),
            ..lane(id, length_m)
        }
    }

    fn pedestrian_lane(id: &str, length_m: f64) -> Lane {
        Lane {
            allow: vec!["pedestrian".into()],
            ..lane(id, length_m)
        }
    }

    fn edge(id: &str, lanes: Vec<Lane>) -> Edge {
        edge_with_function(id, EdgeFunction::Normal, lanes)
    }

    fn edge_with_function(id: &str, function: EdgeFunction, lanes: Vec<Lane>) -> Edge {
        Edge {
            id: EdgeId(id.into()),
            function,
            from: None,
            to: None,
            name: None,
            priority: None,
            length: None,
            shape: None,
            spread_type: None,
            lanes,
        }
    }

    fn junction(id: &str, kind: JunctionKind, incoming_lanes: Vec<&str>) -> Junction {
        Junction {
            id: JunctionId(id.into()),
            position: Point {
                x: 0.0,
                y: 0.0,
                z: 0.0,
            },
            kind,
            incoming_lanes: incoming_lanes
                .into_iter()
                .map(|l| LaneId(l.into()))
                .collect(),
            internal_lanes: vec![],
            shape: None,
            name: None,
        }
    }

    /// Only `state` matters to the grouping under test, so every phase gets
    /// the same placeholder duration; `program_id` is the id SUMO gives a
    /// junction's first program.
    fn program(id: &str, phases: Vec<&str>) -> TrafficLightProgram {
        TrafficLightProgram {
            id: TrafficLightId(id.into()),
            program_id: "0".into(),
            kind: Some(TrafficLightKind::Static),
            offset: None,
            phases: phases
                .into_iter()
                .map(|state| Phase {
                    duration: Time::new::<second>(30.0),
                    state: state.into(),
                })
                .collect(),
        }
    }

    fn vehicle_connection(
        from_edge: &str,
        from_lane: usize,
        tl: &str,
        link_index: i32,
    ) -> Connection {
        Connection {
            from_edge: EdgeId(from_edge.into()),
            to_edge: EdgeId("out".into()),
            from_lane: LaneIndex(from_lane),
            to_lane: LaneIndex(0),
            direction: ConnectionDirection::Straight,
            state: LinkState::Major,
            via: None,
            traffic_light: Some(TrafficLightId(tl.into())),
            link_index: Some(LinkIndex(link_index)),
            pass: false,
            keep_clear: true,
        }
    }

    /// A plain, non-signal-controlled connection between two ordinary
    /// lanes — the kind that links a lane to its predecessor(s) upstream
    /// of a junction, as opposed to [`vehicle_connection`]'s own
    /// tl-controlled connection right at one. Exactly what
    /// [`extended_entry_lanes`]'s own tests need to build a predecessor
    /// chain without also (accidentally) making every hop of it look
    /// signal-controlled.
    fn plain_connection(from_edge: &str, from_lane: usize, to_edge: &str, to_lane: usize) -> Connection {
        Connection {
            from_edge: EdgeId(from_edge.into()),
            to_edge: EdgeId(to_edge.into()),
            from_lane: LaneIndex(from_lane),
            to_lane: LaneIndex(to_lane),
            direction: ConnectionDirection::Straight,
            state: LinkState::Major,
            via: None,
            traffic_light: None,
            link_index: None,
            pass: false,
            keep_clear: true,
        }
    }

    #[test]
    fn groups_lanes_from_the_same_edge_and_direction_into_one_zone() {
        let network = Network {
            edges: vec![edge(
                "e0",
                vec![indexed_lane("e0_0", 0, 25.0), indexed_lane("e0_1", 1, 25.0)],
            )],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", "e0_1"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e0", 1, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "rr"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);

        assert_eq!(
            zones.len(),
            1,
            "both lanes are on e0 going straight -> one movement, one zone"
        );
        assert_eq!(zones[0].entries.len(), 2);
        assert_eq!(zones[0].id, DetectorId("e0_straight".into()));
    }

    #[test]
    fn keeps_lanes_from_different_edges_in_separate_zones_even_with_identical_signal_state() {
        // link 0 and link 1 share the exact same character in every phase --
        // under the old signal-head scheme that merged them into one zone.
        // Identity no longer reads `tlLogic` at all, so two different
        // approaches never merge just because they happen to move together.
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 40.0)]),
            ],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", "e1_0"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 0, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "rr"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);

        assert_eq!(zones.len(), 2, "different edges are different movements");
        assert_eq!(zones[0].id, DetectorId("e0_straight".into()));
        assert_eq!(zones[1].id, DetectorId("e1_straight".into()));
    }

    /// Regression test for the bug the movement-based scheme replaces: the id
    /// used to be `{junction_id}_{index}` after sorting groups by their
    /// phase-state signature, so reordering the phases in a `tlLogic`
    /// program — junction and lanes completely untouched — could reshuffle
    /// which id pointed at which group.
    #[test]
    fn waiting_zone_id_is_unaffected_by_reordering_the_signal_program() {
        let network_with_phases = |phases: Vec<&str>| Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 40.0)]),
            ],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", "e1_0"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 0, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", phases)],
            ..Default::default()
        };

        let original = generate(&network_with_phases(vec!["Gr", "rG"]), None, false);
        let reordered = generate(&network_with_phases(vec!["rG", "Gr"]), None, false);

        let ids = |zones: &[E3Detector]| {
            let mut ids: Vec<_> = zones.iter().map(|z| z.id.0.clone()).collect();
            ids.sort();
            ids
        };

        assert_eq!(
            ids(&original),
            ids(&reordered),
            "swapping the two phases must not rename either zone"
        );
    }

    #[test]
    fn skips_every_lane_when_the_network_has_no_traffic_light_programs_at_all() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLightUnregulated,
                vec!["e0_0"],
            )],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![], // no program at all
            ..Default::default()
        };

        assert!(generate(&network, None, false).is_empty());
    }

    /// Regression test for the bug this module's docs describe at length:
    /// matching a lane's controlling program by the *junction's* id instead
    /// of the *connection's* `tl` silently produced zero zones for every
    /// SUMO `joinTLS`-merged junction — no error, just nothing generated,
    /// on real networks (Barcelona among them) where that's the majority of
    /// traffic lights, not an edge case.
    #[test]
    fn resolves_a_lanes_program_from_its_connections_tl_not_the_junctions_own_id() {
        // No `tlLogic` here has id "j0" at all — only the connection says
        // which program actually controls it, the way a joined program's
        // id never matches any one of the junctions it covers.
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0"])],
            connections: vec![vehicle_connection("e0", 0, "joinedS_j0_j1", 0)],
            traffic_light_programs: vec![program("joinedS_j0_j1", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);

        assert_eq!(
            zones.len(),
            1,
            "the connection names \"joinedS_j0_j1\" as its controlling program directly; \
             the junction's own id (\"j0\") never has to match anything"
        );
    }

    #[test]
    fn skips_lane_and_warns_when_it_has_no_signal_controlled_connection() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0"])],
            connections: vec![], // no connection for e0_0 at all
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        assert!(generate(&network, None, false).is_empty());
    }

    #[test]
    fn excludes_pedestrian_only_lanes_from_vehicle_zones() {
        // Mirrors a real SUMO quirk: a traffic-light junction's `incLanes`
        // can include a walkingarea lane alongside the vehicle lanes.
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 2.0)],
                ),
            ],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", ":j0_w0_0"],
            )],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);

        assert_eq!(zones.len(), 1, "the walkingarea lane isn't a vehicle lane");
        assert_eq!(zones[0].entries.len(), 1);
        assert_eq!(zones[0].entries[0].lane, LaneRef("e0_0".into()));
    }

    /// Mirrors real SUMO's own shape for a signalized crossing: the
    /// walkingarea (`:j0_w0_0`) is itself one of the junction's
    /// `incLanes`, and its tl-controlled connection leads into the
    /// crossing (`:j0_c0_0`), not into the junction directly — the same
    /// structure `test_4x4_ped`'s fixture network uses.
    #[test]
    fn generates_a_pedestrian_zone_for_a_walkingarea_leading_into_a_crossing() {
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 3.3)],
                ),
                edge_with_function(
                    ":j0_c0",
                    EdgeFunction::Crossing,
                    vec![pedestrian_lane(":j0_c0_0", 6.4)],
                ),
            ],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", ":j0_w0_0"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection(":j0_w0", 0, "j0", 1), // walkingarea -> crossing, tl-controlled
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "rr"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);
        let pedestrian_zone = zones
            .iter()
            .find(|zone| zone.detect_persons.contains(&PersonMode::Walk))
            .expect("a pedestrian zone for the walkingarea");

        assert_eq!(zones.len(), 2, "one vehicle zone and one pedestrian zone");
        assert_eq!(pedestrian_zone.entries.len(), 1);
        assert_eq!(pedestrian_zone.entries[0].lane, LaneRef(":j0_w0_0".into()));
        assert!(
            pedestrian_zone.id.0.ends_with("_ped"),
            "{:?} should be marked as a pedestrian zone",
            pedestrian_zone.id
        );
        assert!(
            zones
                .iter()
                .all(|zone| zone.detect_persons.is_empty() || zone == pedestrian_zone),
            "only the pedestrian zone should set detectPersons"
        );
    }

    #[test]
    fn pedestrian_zones_never_extend_backward_even_through_an_unambiguous_chain() {
        // e_sidewalk -> :j0_w0 is exactly the fork-free, single-predecessor
        // shape `extends_a_short_lanes_entry_back_through_an_unambiguous_predecessor_chain`
        // proves a *vehicle* zone does extend through -- a pedestrian zone
        // must not (see `full_lane_boundaries`'s own docs on
        // `extend_backward`).
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge_with_function(
                    "e_sidewalk",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane("e_sidewalk_0", 20.0)],
                ),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 0.1)],
                ),
            ],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", ":j0_w0_0"],
            )],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                plain_connection("e_sidewalk", 0, ":j0_w0", 0),
                vehicle_connection(":j0_w0", 0, "j0", 1),
            ],
            traffic_light_programs: vec![program("j0", vec!["GG", "rr"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);
        let pedestrian_zone = zones
            .iter()
            .find(|zone| zone.detect_persons.contains(&PersonMode::Walk))
            .expect("a pedestrian zone for the walkingarea");

        assert_eq!(
            pedestrian_zone.entries.len(),
            1,
            "e_sidewalk doesn't fork -- a vehicle zone in this exact shape would extend \
             through it, but a pedestrian zone must stay just its own gate: {:?}",
            pedestrian_zone.entries
        );
        assert_eq!(pedestrian_zone.entries[0].lane, LaneRef(":j0_w0_0".into()));
    }

    /// Regression test: SUMO spells a walkingarea's own edge id with a
    /// leading `:` (`:j0_w0`, its own marker for "this is an internal
    /// edge"), and an earlier version of `movement_id` carried that
    /// straight into the zone id. `zone_output` builds the detector's
    /// `file` attribute from that same id, and SUMO's own `OutputDevice`
    /// factory reads any `:` inside an output filename as a `host:port`
    /// remote-socket spec rather than literal text — a zone id like
    /// `:j0_w0_straight_ped` therefore produced a `.add.xml` real SUMO
    /// refused to load ("Given port number '...' is not numeric"), caught
    /// by `sumo_validates_output`'s real-loader check rather than by any
    /// unit test, since none of them shell out to `sumo` at all.
    #[test]
    fn pedestrian_zone_id_has_no_leading_colon_despite_the_walkingareas_own_internal_edge_id() {
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge_with_function(
                    ":j0_w0",
                    EdgeFunction::Walkingarea,
                    vec![pedestrian_lane(":j0_w0_0", 3.3)],
                ),
            ],
            junctions: vec![junction(
                "j0",
                JunctionKind::TrafficLight,
                vec!["e0_0", ":j0_w0_0"],
            )],
            connections: vec![vehicle_connection(":j0_w0", 0, "j0", 0)],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);

        assert_eq!(zones.len(), 1);
        assert_eq!(zones[0].id, DetectorId("j0_w0_straight_ped".into()));
    }

    #[test]
    fn max_zone_length_caps_the_entry_but_keeps_the_exit_at_the_stop_line() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 100.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0"])],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, Some(Length::new::<meter>(20.0)), false);

        assert_eq!(zones.len(), 1);
        assert_eq!(
            zones[0].entries[0].position,
            LanePosition::FromStart(Length::new::<meter>(80.0))
        );
        assert_eq!(
            zones[0].exits[0].position,
            LanePosition::FromStart(Length::new::<meter>(100.0))
        );
    }

    #[test]
    fn max_zone_length_longer_than_the_lane_has_no_effect() {
        let network = Network {
            edges: vec![edge("e0", vec![indexed_lane("e0_0", 0, 25.0)])],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e0_0"])],
            connections: vec![vehicle_connection("e0", 0, "j0", 0)],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, Some(Length::new::<meter>(1000.0)), false);

        assert_eq!(
            zones[0].entries[0].position,
            LanePosition::FromStart(Length::new::<meter>(0.0))
        );
    }

    #[test]
    fn zones_at_the_same_junction_share_the_junction_position_as_icon_position() {
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 1, 40.0)]),
            ],
            junctions: vec![Junction {
                id: JunctionId("j0".into()),
                position: Point {
                    x: 42.0,
                    y: 7.0,
                    z: 0.0,
                },
                kind: JunctionKind::TrafficLight,
                incoming_lanes: vec![LaneId("e0_0".into()), LaneId("e1_0".into())],
                internal_lanes: vec![],
                shape: None,
                name: None,
            }],
            connections: vec![
                vehicle_connection("e0", 0, "j0", 0),
                vehicle_connection("e1", 1, "j0", 1),
            ],
            // link 0 and link 1 diverge in phase 2 -> two separate zones
            traffic_light_programs: vec![program("j0", vec!["GG", "Gr"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);

        assert_eq!(zones.len(), 2, "two signal groups at the same junction");
        for zone in &zones {
            assert_eq!(
                zone.icon_position,
                Some(Point {
                    x: 42.0,
                    y: 7.0,
                    z: 0.0
                }),
                "zone {:?} should be anchored at the junction's own position",
                zone.id
            );
        }
    }

    // The three tests below exercise `group_key_for_lane` directly rather
    // than through `generate()`: the only way `generate()`'s own tests can
    // observe *why* a lane was skipped is by reading the warning printed to
    // stderr, and asserting on that would make the test suite fragile
    // against wording changes for no real gain. Calling the private
    // function itself and matching on the returned `UngroupedReason` checks
    // the thing that actually matters — which diagnosis a given network
    // shape produces — without caring how it's phrased.

    #[test]
    fn group_key_for_lane_reports_no_outgoing_connection_at_all() {
        let lane_id = LaneId("e0_0".into());
        let connections_by_from_lane = HashMap::new();
        let programs = HashMap::new();

        assert_eq!(
            group_key_for_lane(&lane_id, &connections_by_from_lane, &programs),
            Err(UngroupedReason::NoOutgoingConnection)
        );
    }

    #[test]
    fn group_key_for_lane_reports_no_resolvable_program() {
        let lane_id = LaneId("e0_0".into());
        // Names "j0" as its controlling program, but `programs` below is
        // empty — mirrors a `.net.xml` where the connection's `tl` doesn't
        // match any `tlLogic` this crate could find (including, before the
        // fix documented at the top of this module, every `joinTLS`-merged
        // junction).
        let connection = vehicle_connection("e0", 0, "j0", 0);
        let connections_by_from_lane = HashMap::from([(&lane_id, vec![&connection])]);
        let programs = HashMap::new();

        assert_eq!(
            group_key_for_lane(&lane_id, &connections_by_from_lane, &programs),
            Err(UngroupedReason::NoResolvableProgram)
        );
    }

    #[test]
    fn group_key_for_lane_reports_link_index_out_of_range() {
        let lane_id = LaneId("e0_0".into());
        // linkIndex 5, but the program's own phases are 1 character long —
        // a malformed `.net.xml`, the one case of the three that actually
        // is one.
        let connection = vehicle_connection("e0", 0, "j0", 5);
        let connections_by_from_lane = HashMap::from([(&lane_id, vec![&connection])]);
        let prog = program("j0", vec!["G"]);
        let tl_id = TrafficLightId("j0".into());
        let programs = HashMap::from([(&tl_id, &prog)]);

        assert_eq!(
            group_key_for_lane(&lane_id, &connections_by_from_lane, &programs),
            Err(UngroupedReason::LinkIndexOutOfRange {
                program: TrafficLightId("j0".into()),
                index: LinkIndex(5),
            })
        );
    }

    // The tests below exercise `extended_entry_lanes` (see the module
    // docs' "maximizing a waiting zone's own physical size") through
    // `generate()` end to end, the same way every other test in this file
    // does — except the last one, which calls it directly to prove
    // termination on a shape no real network can actually produce (see its
    // own docs).

    #[test]
    fn extends_a_short_lanes_entry_back_through_an_unambiguous_predecessor_chain() {
        // e0 -> e1 is the only way in or out of e1 -- e0 doesn't fork, so
        // the entry should walk all the way back onto e0's own start
        // instead of stopping at e1's tiny 0.1m (a real Barcelona
        // `netconvert` stub-lane length).
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 0.1)]),
            ],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e1_0"])],
            connections: vec![
                plain_connection("e0", 0, "e1", 0),
                vehicle_connection("e1", 0, "j0", 0),
            ],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);
        assert_eq!(zones.len(), 1);
        let zone = &zones[0];

        assert_eq!(zone.exits.len(), 1, "the exit stays on the real controlled lane");
        assert_eq!(zone.exits[0].lane, LaneRef("e1_0".into()));

        // e1's own gate is kept (so a vehicle spawned directly on it is
        // still detected) *and* extended onto e0's own start.
        let mut entry_lanes: Vec<String> = zone.entries.iter().map(|g| g.lane.0.clone()).collect();
        entry_lanes.sort();
        assert_eq!(entry_lanes, vec!["e0_0".to_string(), "e1_0".to_string()]);
        assert!(
            zone.entries
                .iter()
                .all(|g| g.position == LanePosition::FromStart(Length::new::<meter>(0.0)))
        );
    }

    #[test]
    fn extending_through_a_hop_also_adds_its_own_internal_via_lane() {
        // e0 -> e1 with an internal lane (":j0_0_0") bridging them --
        // real edges only meet *through* a junction's own internal
        // geometry, so without this the drawn zone would show a gap
        // between e0's and e1's own rectangles even though they're the
        // same zone (see zone_feature's own docs on why `geojson_output`
        // draws one independent rectangle per entry).
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge_with_function(
                    ":j0_0",
                    EdgeFunction::Internal,
                    vec![indexed_lane(":j0_0_0", 0, 4.0)],
                ),
                edge("e1", vec![indexed_lane("e1_0", 0, 0.1)]),
            ],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e1_0"])],
            connections: vec![
                Connection {
                    via: Some(LaneId(":j0_0_0".into())),
                    ..plain_connection("e0", 0, "e1", 0)
                },
                vehicle_connection("e1", 0, "j0", 0),
            ],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);
        let zone = &zones[0];

        let mut entry_lanes: Vec<String> = zone.entries.iter().map(|g| g.lane.0.clone()).collect();
        entry_lanes.sort();
        assert_eq!(
            entry_lanes,
            vec![":j0_0_0".to_string(), "e0_0".to_string(), "e1_0".to_string()],
            "the internal lane bridging e0 and e1 should be drawn too, not just the two \
             real lanes on either side of it"
        );
    }

    #[test]
    fn stops_extending_at_a_predecessor_that_forks() {
        // e0 leads to *both* e1 and e2 -- not all of e0's own traffic is
        // headed into this zone, so extension must not walk through it.
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 0.1)]),
                edge("e2", vec![indexed_lane("e2_0", 0, 10.0)]),
            ],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e1_0"])],
            connections: vec![
                plain_connection("e0", 0, "e1", 0),
                plain_connection("e0", 0, "e2", 0),
                vehicle_connection("e1", 0, "j0", 0),
            ],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);
        let zone = &zones[0];

        assert_eq!(zone.entries.len(), 1);
        assert_eq!(
            zone.entries[0].lane,
            LaneRef("e1_0".into()),
            "e0 forks (leads to both e1 and e2) -- extension must stop before it, not \
             silently claim traffic that's actually headed to e2 as part of this zone"
        );
    }

    #[test]
    fn stops_extending_at_a_predecessor_that_already_has_its_own_signal_controlled_zone() {
        // e0 doesn't fork (its only real-to-real successor is e1), but e0
        // is *itself* a signal-controlled approach at a different junction
        // (j_upstream) -- it already gets its own waiting zone there, so
        // j0's own zone must not also draw a rectangle on top of it.
        let network = Network {
            edges: vec![
                edge("e_further_back", vec![indexed_lane("e_further_back_0", 0, 40.0)]),
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 0.1)]),
            ],
            junctions: vec![
                junction("j_upstream", JunctionKind::TrafficLight, vec!["e0_0"]),
                junction("j0", JunctionKind::TrafficLight, vec!["e1_0"]),
            ],
            connections: vec![
                plain_connection("e_further_back", 0, "e0", 0),
                vehicle_connection("e0", 0, "j_upstream", 0),
                plain_connection("e0", 0, "e1", 0),
                vehicle_connection("e1", 0, "j0", 0),
            ],
            traffic_light_programs: vec![
                program("j_upstream", vec!["G"]),
                program("j0", vec!["G"]),
            ],
            ..Default::default()
        };

        let zones = generate(&network, None, false);
        let j0_zone = zones
            .iter()
            .find(|z| z.exits.iter().any(|g| g.lane == LaneRef("e1_0".into())))
            .expect("a zone for j0");

        let entry_lanes: Vec<String> = j0_zone.entries.iter().map(|g| g.lane.0.clone()).collect();
        assert!(
            entry_lanes.contains(&"e1_0".to_string()),
            "e1's own gate is always kept"
        );
        assert!(
            !entry_lanes.iter().any(|l| l == "e0_0" || l == "e_further_back_0"),
            "e0 already has its own zone at j_upstream (and doesn't fork) -- j0's zone must \
             not extend through it (or past it, onto e_further_back) and draw a rectangle on \
             top of j_upstream's own: {entry_lanes:?}"
        );
    }

    #[test]
    fn extends_through_every_predecessor_that_individually_only_leads_here() {
        // e0 and e2 both feed *only* into e1 -- a merge, not a fork from
        // either predecessor's own point of view, so both extend.
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e2", vec![indexed_lane("e2_0", 0, 15.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 0.1)]),
            ],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e1_0"])],
            connections: vec![
                plain_connection("e0", 0, "e1", 0),
                plain_connection("e2", 0, "e1", 0),
                vehicle_connection("e1", 0, "j0", 0),
            ],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, None, false);
        let zone = &zones[0];

        let mut entry_lanes: Vec<String> = zone.entries.iter().map(|g| g.lane.0.clone()).collect();
        entry_lanes.sort();
        assert_eq!(
            entry_lanes,
            vec!["e0_0".to_string(), "e1_0".to_string(), "e2_0".to_string()],
            "e1's own gate is kept in addition to both extended ancestors"
        );
    }

    #[test]
    fn max_zone_length_suppresses_extension_when_it_caps_the_entry_short_of_the_lanes_own_start() {
        // 50m lane capped to the last 20m: the entry lands at position 30,
        // nowhere near e1's own start, so there's nothing for extension to
        // pick up from -- it should stay exactly where max_zone_length put
        // it, on e1 itself.
        let network = Network {
            edges: vec![
                edge("e0", vec![indexed_lane("e0_0", 0, 25.0)]),
                edge("e1", vec![indexed_lane("e1_0", 0, 50.0)]),
            ],
            junctions: vec![junction("j0", JunctionKind::TrafficLight, vec!["e1_0"])],
            connections: vec![
                plain_connection("e0", 0, "e1", 0),
                vehicle_connection("e1", 0, "j0", 0),
            ],
            traffic_light_programs: vec![program("j0", vec!["G"])],
            ..Default::default()
        };

        let zones = generate(&network, Some(Length::new::<meter>(20.0)), false);
        let zone = &zones[0];

        assert_eq!(zone.entries.len(), 1);
        assert_eq!(zone.entries[0].lane, LaneRef("e1_0".into()));
        assert_eq!(
            zone.entries[0].position,
            LanePosition::FromStart(Length::new::<meter>(30.0))
        );
    }

    #[test]
    fn extended_entry_lanes_terminates_on_a_cycle_no_real_network_would_produce() {
        // A pure a<->b loop with nothing else attached to either lane --
        // topologically impossible to reach from a real signal-controlled
        // lane (something has to break the cycle to let traffic leave it
        // at all, which always shows up as a fork somewhere -- see
        // `stops_extending_at_a_predecessor_that_forks`), but
        // `extended_entry_lanes` shouldn't infinite-loop on it regardless
        // of whether real `netconvert` output could ever produce it.
        let lane_a = LaneId("a_0".into());
        let lane_b = LaneId("b_0".into());
        let edge_a = EdgeId("a".into());
        let edge_b = EdgeId("b".into());

        let connection_a_to_b = plain_connection("a", 0, "b", 0);
        let connection_b_to_a = plain_connection("b", 0, "a", 0);

        let lane_ids_by_edge_and_index: HashMap<(&EdgeId, LaneIndex), &LaneId> = HashMap::from([
            ((&edge_a, LaneIndex(0)), &lane_a),
            ((&edge_b, LaneIndex(0)), &lane_b),
        ]);
        let connections_by_from_lane: HashMap<&LaneId, Vec<&Connection>> = HashMap::from([
            (&lane_a, vec![&connection_a_to_b]),
            (&lane_b, vec![&connection_b_to_a]),
        ]);
        let predecessors_by_lane: HashMap<&LaneId, HashSet<&LaneId>> = HashMap::from([
            (&lane_a, HashSet::from([&lane_b])),
            (&lane_b, HashSet::from([&lane_a])),
        ]);
        let signal_controlled_lanes: HashSet<&LaneId> = HashSet::new();
        let via_lane_between: HashMap<(&LaneId, &LaneId), &LaneId> = HashMap::new();
        let lanes: HashMap<&LaneId, LaneInfo> = HashMap::from([
            (&lane_a, LaneInfo { length: Length::new::<meter>(10.0), pedestrian_only: false }),
            (&lane_b, LaneInfo { length: Length::new::<meter>(10.0), pedestrian_only: false }),
        ]);
        let edge_start_junction_by_lane: HashMap<&LaneId, &JunctionId> = HashMap::new();
        let complex_intersection_junctions: HashSet<&JunctionId> = HashSet::new();
        let edge_by_lane: HashMap<&LaneId, &EdgeId> = HashMap::new();
        let graph = ConnectivityGraph {
            lane_ids_by_edge_and_index: &lane_ids_by_edge_and_index,
            connections_by_from_lane: &connections_by_from_lane,
            predecessors_by_lane: &predecessors_by_lane,
            signal_controlled_lanes: &signal_controlled_lanes,
            via_lane_between: &via_lane_between,
            lanes: &lanes,
            edge_start_junction_by_lane: &edge_start_junction_by_lane,
            complex_intersection_junctions: &complex_intersection_junctions,
            edge_by_lane: &edge_by_lane,
            stop_at_complex_intersections: false,
        };

        let mut visited = HashSet::new();
        let budget = Length::new::<meter>(DEFAULT_EXTENSION_METERS);
        let frontier = extended_entry_lanes(&lane_a, &graph, &mut visited, budget);
        assert!(!frontier.is_empty(), "must terminate with a real answer, not hang");
    }

    #[test]
    fn extended_entry_lanes_stops_once_the_budget_runs_out() {
        // A straight chain of three equal-length lanes (c -> b -> a), each
        // with exactly one outgoing connection and none of them signal-
        // controlled -- nothing but the budget itself would ever stop this
        // walk. Each lane is sized to just over half of
        // `DEFAULT_EXTENSION_METERS`, so a single one fits the budget
        // (covering `b`) but two in a row don't (excluding `c`, on top of
        // `b`) -- the exact "long straight street with only minor cross
        // traffic" shape that walked clean across a dozen-plus real
        // Barcelona blocks before this budget existed. Expressed as a
        // fraction of the constant itself, not a hard-coded metre figure,
        // so this keeps meaning the same thing if that default ever gets
        // retuned.
        let segment_length = Length::new::<meter>(DEFAULT_EXTENSION_METERS / 2.0 + 1.0);

        let lane_a = LaneId("a_0".into());
        let lane_b = LaneId("b_0".into());
        let lane_c = LaneId("c_0".into());
        let edge_a = EdgeId("a".into());
        let edge_b = EdgeId("b".into());
        let edge_c = EdgeId("c".into());

        let connection_b_to_a = plain_connection("b", 0, "a", 0);
        let connection_c_to_b = plain_connection("c", 0, "b", 0);

        let lane_ids_by_edge_and_index: HashMap<(&EdgeId, LaneIndex), &LaneId> = HashMap::from([
            ((&edge_a, LaneIndex(0)), &lane_a),
            ((&edge_b, LaneIndex(0)), &lane_b),
            ((&edge_c, LaneIndex(0)), &lane_c),
        ]);
        let connections_by_from_lane: HashMap<&LaneId, Vec<&Connection>> = HashMap::from([
            (&lane_b, vec![&connection_b_to_a]),
            (&lane_c, vec![&connection_c_to_b]),
        ]);
        let predecessors_by_lane: HashMap<&LaneId, HashSet<&LaneId>> = HashMap::from([
            (&lane_a, HashSet::from([&lane_b])),
            (&lane_b, HashSet::from([&lane_c])),
        ]);
        let signal_controlled_lanes: HashSet<&LaneId> = HashSet::new();
        let via_lane_between: HashMap<(&LaneId, &LaneId), &LaneId> = HashMap::new();
        let lanes: HashMap<&LaneId, LaneInfo> = HashMap::from([
            (&lane_a, LaneInfo { length: segment_length, pedestrian_only: false }),
            (&lane_b, LaneInfo { length: segment_length, pedestrian_only: false }),
            (&lane_c, LaneInfo { length: segment_length, pedestrian_only: false }),
        ]);
        let edge_start_junction_by_lane: HashMap<&LaneId, &JunctionId> = HashMap::new();
        let complex_intersection_junctions: HashSet<&JunctionId> = HashSet::new();
        let edge_by_lane: HashMap<&LaneId, &EdgeId> = HashMap::new();
        let graph = ConnectivityGraph {
            lane_ids_by_edge_and_index: &lane_ids_by_edge_and_index,
            connections_by_from_lane: &connections_by_from_lane,
            predecessors_by_lane: &predecessors_by_lane,
            signal_controlled_lanes: &signal_controlled_lanes,
            via_lane_between: &via_lane_between,
            lanes: &lanes,
            edge_start_junction_by_lane: &edge_start_junction_by_lane,
            complex_intersection_junctions: &complex_intersection_junctions,
            edge_by_lane: &edge_by_lane,
            stop_at_complex_intersections: false,
        };

        let mut visited = HashSet::new();
        let budget = Length::new::<meter>(DEFAULT_EXTENSION_METERS);
        let frontier = extended_entry_lanes(&lane_a, &graph, &mut visited, budget);
        assert_eq!(frontier, vec![&lane_b], "b (150m) fits the 300m budget, c (300m more) doesn't");
    }

    #[test]
    fn stop_at_complex_intersections_flag_gates_the_new_stopping_condition() {
        // A plain a <- b <- c chain (no fork, no signal anywhere) where b's
        // own edge starts at `junction` -- the only thing that's supposed
        // to make a difference here is whether `junction` is in
        // `complex_intersection_junctions` *and* the flag asking to treat
        // that as a stop is actually on. With the flag off, this is just
        // an ordinary unbounded walk (see
        // `extended_entry_lanes_stops_once_the_budget_runs_out`'s own
        // fixture) and reaches all the way to `c`; with it on, `b` is
        // still included (its own predecessor's connection lands there
        // regardless), but the walk doesn't go looking at *b's own*
        // predecessors once it's standing at a junction that's part of a
        // real, physically complex intersection.
        let lane_a = LaneId("a_0".into());
        let lane_b = LaneId("b_0".into());
        let lane_c = LaneId("c_0".into());
        let edge_a = EdgeId("a".into());
        let edge_b = EdgeId("b".into());
        let edge_c = EdgeId("c".into());
        let junction = JunctionId("complex_junction".into());

        let connection_b_to_a = plain_connection("b", 0, "a", 0);
        let connection_c_to_b = plain_connection("c", 0, "b", 0);

        let lane_ids_by_edge_and_index: HashMap<(&EdgeId, LaneIndex), &LaneId> = HashMap::from([
            ((&edge_a, LaneIndex(0)), &lane_a),
            ((&edge_b, LaneIndex(0)), &lane_b),
            ((&edge_c, LaneIndex(0)), &lane_c),
        ]);
        let connections_by_from_lane: HashMap<&LaneId, Vec<&Connection>> = HashMap::from([
            (&lane_b, vec![&connection_b_to_a]),
            (&lane_c, vec![&connection_c_to_b]),
        ]);
        let predecessors_by_lane: HashMap<&LaneId, HashSet<&LaneId>> = HashMap::from([
            (&lane_a, HashSet::from([&lane_b])),
            (&lane_b, HashSet::from([&lane_c])),
        ]);
        let signal_controlled_lanes: HashSet<&LaneId> = HashSet::new();
        let via_lane_between: HashMap<(&LaneId, &LaneId), &LaneId> = HashMap::new();
        let lanes: HashMap<&LaneId, LaneInfo> = HashMap::from([
            (&lane_a, LaneInfo { length: Length::new::<meter>(10.0), pedestrian_only: false }),
            (&lane_b, LaneInfo { length: Length::new::<meter>(10.0), pedestrian_only: false }),
            (&lane_c, LaneInfo { length: Length::new::<meter>(10.0), pedestrian_only: false }),
        ]);
        // Only `b` sits at the complex junction — `a` and `c` are ordinary
        // ground, so this only ever changes what happens *at* `b`.
        let edge_start_junction_by_lane: HashMap<&LaneId, &JunctionId> =
            HashMap::from([(&lane_b, &junction)]);
        let complex_intersection_junctions: HashSet<&JunctionId> = HashSet::from([&junction]);
        let edge_by_lane: HashMap<&LaneId, &EdgeId> = HashMap::new();

        let budget = Length::new::<meter>(DEFAULT_EXTENSION_METERS);

        let graph_flag_off = ConnectivityGraph {
            lane_ids_by_edge_and_index: &lane_ids_by_edge_and_index,
            connections_by_from_lane: &connections_by_from_lane,
            predecessors_by_lane: &predecessors_by_lane,
            signal_controlled_lanes: &signal_controlled_lanes,
            via_lane_between: &via_lane_between,
            lanes: &lanes,
            edge_start_junction_by_lane: &edge_start_junction_by_lane,
            complex_intersection_junctions: &complex_intersection_junctions,
            edge_by_lane: &edge_by_lane,
            stop_at_complex_intersections: false,
        };
        let mut visited = HashSet::new();
        let frontier = extended_entry_lanes(&lane_a, &graph_flag_off, &mut visited, budget);
        assert_eq!(frontier, vec![&lane_b, &lane_c], "flag off: an ordinary unbounded walk reaches c");

        let graph_flag_on = ConnectivityGraph { stop_at_complex_intersections: true, ..graph_flag_off };
        let mut visited = HashSet::new();
        let frontier = extended_entry_lanes(&lane_a, &graph_flag_on, &mut visited, budget);
        assert_eq!(
            frontier,
            vec![&lane_b],
            "flag on: b is still included (a's own predecessor), but the walk stops \
             there instead of also looking at b's own predecessor c"
        );
    }
}

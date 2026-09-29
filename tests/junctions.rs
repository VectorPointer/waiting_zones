//! End-to-end behaviour on small synthetic junctions: which zones come out,
//! and which phases of the program each one is green in.

use waiting_zones::output::{Class, JunctionProgram};
use waiting_zones::zones::Reach;
use waiting_zones::{Generated, generate, osm, program};

const REACH: Reach = Reach { max_length: None };

fn run(xml: &str) -> Generated {
    let generated = generate(&osm::read_from(xml.as_bytes()).expect("valid OSM"), REACH);
    for plan in &generated.plans {
        let problems = program::violations(plan);
        assert!(
            problems.is_empty(),
            "unsound program:\n{}",
            problems.join("\n")
        );
    }
    generated
}

fn vehicle_ids(generated: &Generated) -> Vec<&str> {
    let mut ids: Vec<&str> = generated
        .zones
        .iter()
        .filter(|z| z.class != Class::Pedestrian)
        .map(|z| z.id.as_str())
        .collect();
    ids.sort_unstable();
    ids
}

fn green_phases(program: &JunctionProgram) -> Vec<usize> {
    (0..program.program.phases.len()).step_by(2).collect()
}

fn phases_of<'a>(program: &'a JunctionProgram, zone: &str) -> &'a [i32] {
    &program
        .zones
        .iter()
        .find(|z| z.detector_id == zone)
        .unwrap_or_else(|| panic!("no zone {zone}"))
        .phases
}

/// A signalized cross: north-south way 10 through node 1, east-west way 20
/// through node 1, arms ~110m long. `extra` goes inside `<osm>`.
fn cross(ew_tags: &str, extra: &str) -> String {
    format!(
        r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="3" lat="41.3990" lon="2.2000"/>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <way id="10"><nd ref="2"/><nd ref="1"/><nd ref="3"/><tag k="highway" v="residential"/></way>
        <way id="20"><nd ref="5"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="primary"/>{ew_tags}</way>
        {extra}
        </osm>"#
    )
}

#[test]
fn a_plain_cross_pairs_opposite_approaches_with_permissive_lefts() {
    let generated = run(&cross("", ""));
    assert_eq!(
        vehicle_ids(&generated),
        vec![
            "-10#1_straight+left+right",
            "-20#1_straight+left+right",
            "10#0_straight+left+right",
            "20#0_straight+left+right",
        ]
    );
    assert_eq!(generated.programs.len(), 1);
    let program = &generated.programs[0];
    assert_eq!(program.tls_id, "1");
    assert_eq!(
        green_phases(program).len(),
        2,
        "one phase per pair of opposite approaches"
    );
    // Opposite approaches share a phase; perpendicular ones never do.
    assert_eq!(
        phases_of(program, "10#0_straight+left+right"),
        phases_of(program, "-10#1_straight+left+right")
    );
    assert_eq!(
        phases_of(program, "20#0_straight+left+right"),
        phases_of(program, "-20#1_straight+left+right")
    );
    assert_ne!(
        phases_of(program, "10#0_straight+left+right"),
        phases_of(program, "20#0_straight+left+right")
    );
    // Single-lane approaches: the left turn shares its lane, so it yields.
    let green = &program.program.phases[0].1;
    assert!(green.contains('g'), "a permissive left in {green}");
}

#[test]
fn painted_lanes_split_zones_and_earn_a_protected_left() {
    let generated = run(&cross(
        r#"<tag k="lanes" v="6"/><tag k="turn:lanes:forward" v="left|through|through;right"/><tag k="turn:lanes:backward" v="left|through|through;right"/>"#,
        "",
    ));
    let ids = vehicle_ids(&generated);
    for id in [
        "20#0_left",
        "20#0_straight",
        "20#0_straight+right",
        "-20#1_left",
        "-20#1_straight",
        "-20#1_straight+right",
    ] {
        assert!(ids.contains(&id), "{id} missing from {ids:?}");
    }
    let program = &generated.programs[0];
    assert_eq!(
        green_phases(program).len(),
        3,
        "north-south, east-west, east-west protected left"
    );
    // The dedicated left lanes get their own phase, apart from the straight.
    let left = phases_of(program, "20#0_left");
    let straight = phases_of(program, "20#0_straight");
    assert!(
        left.iter().any(|p| !straight.contains(p)),
        "left {left:?} vs straight {straight:?}"
    );
}

#[test]
fn a_t_junctions_stem_gets_a_phase_of_its_own() {
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <way id="10"><nd ref="2"/><nd ref="1"/><tag k="highway" v="residential"/></way>
        <way id="20"><nd ref="5"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="primary"/></way>
        </osm>"#);
    assert!(vehicle_ids(&generated).contains(&"10#0_left+right"));
    let program = &generated.programs[0];
    assert_eq!(green_phases(program).len(), 2);
    let stem = phases_of(program, "10#0_left+right");
    assert!(
        stem.iter()
            .all(|p| !phases_of(program, "20#0_straight+left").contains(p))
    );
}

#[test]
fn a_dual_carriageway_crossing_is_one_controller() {
    let generated = run(r#"<osm>
        <node id="11" lat="41.40007" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="12" lat="41.39993" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="3" lat="41.3990" lon="2.2000"/>
        <node id="21" lat="41.40007" lon="2.2013"/>
        <node id="22" lat="41.40007" lon="2.1987"/>
        <node id="31" lat="41.39993" lon="2.1987"/>
        <node id="32" lat="41.39993" lon="2.2013"/>
        <way id="10"><nd ref="2"/><nd ref="11"/><nd ref="12"/><nd ref="3"/><tag k="highway" v="residential"/></way>
        <way id="30"><nd ref="21"/><nd ref="11"/><nd ref="22"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        <way id="40"><nd ref="31"/><nd ref="12"/><nd ref="32"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        </osm>"#);
    assert_eq!(
        generated.programs.len(),
        1,
        "both junction nodes share one controller"
    );
    let edges: Vec<&str> = generated.programs[0]
        .zones
        .iter()
        .map(|z| z.edge.as_str())
        .collect();
    // The two short segments between the carriageways are inside the junction.
    assert!(
        !edges.contains(&"10#1") && !edges.contains(&"-10#1"),
        "{edges:?}"
    );
    for approach in ["10#0", "-10#2", "30#0", "40#0"] {
        assert!(
            edges.contains(&approach),
            "{approach} missing from {edges:?}"
        );
    }
}

#[test]
fn a_no_left_turn_restriction_removes_only_that_turn() {
    // OSM splits restriction ways at the via node: way 20 ends at node 1
    // coming from the west, way 21 carries on east.
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="3" lat="41.3990" lon="2.2000"/>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <way id="10"><nd ref="2"/><nd ref="1"/><nd ref="3"/><tag k="highway" v="residential"/></way>
        <way id="20"><nd ref="5"/><nd ref="1"/><tag k="highway" v="primary"/></way>
        <way id="21"><nd ref="1"/><nd ref="4"/><tag k="highway" v="primary"/></way>
        <relation id="9">
            <member type="way" ref="20" role="from"/><member type="node" ref="1" role="via"/><member type="way" ref="10" role="to"/>
            <tag k="type" v="restriction"/><tag k="restriction" v="no_left_turn"/></relation>
        </osm>"#);
    let ids = vehicle_ids(&generated);
    // Eastbound (way 20) can no longer turn left onto way 10; westbound
    // (way 21) is untouched.
    assert!(ids.contains(&"20#0_straight+right"), "{ids:?}");
    assert!(ids.contains(&"-21#0_straight+left+right"), "{ids:?}");
}

#[test]
fn zones_extend_upstream_until_a_fork() {
    // Way 50 (A→B) feeds way 51 (B→junction 1) with nothing else at B.
    let straight_on = r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.3990" lon="2.2000"/>
        <node id="6" lat="41.4010" lon="2.2000"/>
        <node id="7" lat="41.4000" lon="2.1990"/>
        <node id="8" lat="41.4000" lon="2.1970"/>
        <way id="50"><nd ref="8"/><nd ref="7"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        <way id="51"><nd ref="7"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        <way id="10"><nd ref="6"/><nd ref="1"/><nd ref="5"/><tag k="highway" v="residential"/></way>
        FORK
        </osm>"#;
    let length = |xml: &str| {
        let generated = run(xml);
        generated.programs[0]
            .zones
            .iter()
            .find(|z| z.edge == "51#0")
            .expect("zone on 51#0")
            .length_meters
    };
    let through = length(&straight_on.replace("FORK", ""));
    let forked = length(&straight_on.replace(
        "FORK",
        r#"<node id="9" lat="41.4010" lon="2.1990"/><way id="52"><nd ref="7"/><nd ref="9"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>"#,
    ));
    // 51#0 is ~83m to the junction; uncapped, 50#0 adds its whole ~167m.
    assert!((235.0..260.0).contains(&through), "through: {through}");
    assert!(
        forked < 90.0,
        "a fork at node 7 stops the extension: {forked}"
    );
}

#[test]
fn a_mid_block_crossing_on_a_one_lane_one_way_road_stops_the_traffic() {
    // The car's lane runs exactly through the crossing node: the crossing
    // must still be seen as crossed, and get its own phase.
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="crossing"/><tag k="crossing" v="traffic_signals"/></node>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <node id="6" lat="41.40004" lon="2.2000"/>
        <node id="7" lat="41.39996" lon="2.2000"/>
        <way id="20"><nd ref="5"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>
        <way id="30"><nd ref="6"/><nd ref="1"/><nd ref="7"/><tag k="highway" v="footway"/><tag k="footway" v="crossing"/></way>
        </osm>"#);
    let program = &generated.programs[0];
    assert_eq!(
        green_phases(program).len(),
        2,
        "{:?}",
        program.program.phases
    );
    let cars = phases_of(program, "20#0_straight");
    let walkers = phases_of(program, "1_ped");
    assert!(
        cars.iter().all(|p| !walkers.contains(p)),
        "cars {cars:?} walkers {walkers:?}"
    );
}

#[test]
fn two_one_lane_one_way_roads_crossing_at_a_node_never_share_green() {
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="3" lat="41.3990" lon="2.2000"/>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <way id="10"><nd ref="2"/><nd ref="1"/><nd ref="3"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>
        <way id="20"><nd ref="5"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>
        </osm>"#);
    let program = &generated.programs[0];
    let south = phases_of(program, "10#0_straight+left");
    let east = phases_of(program, "20#0_straight+right");
    assert!(
        south.iter().all(|p| !east.contains(p)),
        "south {south:?} east {east:?}"
    );
}

#[test]
fn lights_on_the_arms_before_the_junction_still_separate_crossing_traffic() {
    // The usual OSM mapping: no light on the junction node, one on each arm
    // ~10m before it, so each approach stops right at its own end.
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"/>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="3" lat="41.3990" lon="2.2000"/>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <node id="12" lat="41.40009" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="13" lat="41.39991" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="14" lat="41.4000" lon="2.20012"><tag k="highway" v="traffic_signals"/></node>
        <node id="15" lat="41.4000" lon="2.19988"><tag k="highway" v="traffic_signals"/></node>
        <way id="10"><nd ref="2"/><nd ref="12"/><nd ref="1"/><nd ref="13"/><nd ref="3"/><tag k="highway" v="residential"/></way>
        <way id="20"><nd ref="5"/><nd ref="15"/><nd ref="1"/><nd ref="14"/><nd ref="4"/><tag k="highway" v="primary"/></way>
        </osm>"#);
    assert_eq!(
        generated.programs.len(),
        1,
        "the four lights and the junction are one controller"
    );
    let program = &generated.programs[0];
    assert_eq!(
        green_phases(program).len(),
        2,
        "{:?}",
        program.program.phases
    );
    let north_south = phases_of(program, "10#0_straight+left+right");
    let east_west = phases_of(program, "20#0_straight+left+right");
    assert!(
        north_south.iter().all(|p| !east_west.contains(p)),
        "{north_south:?} vs {east_west:?}"
    );
}

#[test]
fn vehicle_zones_run_up_to_a_diagonal_crosswalk_without_overlapping_it() {
    // A mid-block crossing drawn ~37° off square, leaning away from both
    // lanes where they reach it: a lane band cut square at
    // its stop line would leave a wedge of road between the cars and the
    // crosswalk. Each vehicle zone must reach the pedestrian zone across its
    // whole width and share only a border with it.
    use geo::{BooleanOps, Contains, Point};
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="crossing"/><tag k="crossing" v="traffic_signals"/></node>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <node id="6" lat="41.40004" lon="2.19996"/>
        <node id="7" lat="41.39996" lon="2.20004"/>
        <way id="20"><nd ref="5"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="residential"/></way>
        <way id="30"><nd ref="6"/><nd ref="1"/><nd ref="7"/><tag k="highway" v="footway"/><tag k="footway" v="crossing"/></way>
        </osm>"#);
    let zone = |id: &str| {
        &generated
            .zones
            .iter()
            .find(|z| z.id == id)
            .unwrap_or_else(|| panic!("no zone {id}; have {:?}", vehicle_ids(&generated)))
            .polygon
    };
    let walkers = zone("1_ped");
    let centre = generated.projection.to_local([2.2000, 41.4000]);
    // Eastbound keeps right (south of the centreline), westbound north.
    for (id, sides, heading) in [
        ("20#0_straight", [-0.3, -1.4, -2.5], 1.0),
        ("-20#1_straight", [0.3, 1.4, 2.5], -1.0),
    ] {
        let cars = zone(id);
        let shared = cars.intersection(walkers);
        assert!(
            geo::Area::unsigned_area(&shared) < 0.5,
            "{id} overlaps the crosswalk by {}m²",
            geo::Area::unsigned_area(&shared)
        );
        for side in sides {
            // Drive along the lane towards the crossing until entering the
            // pedestrian zone: the ground just before it is the cars'.
            let at = |s: f64| Point::new(centre[0] + heading * s, centre[1] + side);
            let entry = (0..600)
                .map(|k| -20.0 + k as f64 * 0.05)
                .find(|&s| walkers.contains(&at(s)))
                .unwrap_or_else(|| panic!("{id}: lane at {side}m never meets the crosswalk"));
            assert!(
                cars.contains(&at(entry - 0.15)),
                "{id}: gap between the cars and the crosswalk at {side}m across"
            );
        }
    }
}

#[test]
fn a_zone_ending_at_an_upstream_fork_stays_out_of_that_junction() {
    // Way 50 (A→B) feeds way 51 (B→junction 1), and way 52 leaves B
    // northwards: the zone on 51#0 can't extend past B, and must stop at
    // the edge of B's junction instead of reaching its middle.
    use geo::{Contains, Point};
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.3990" lon="2.2000"/>
        <node id="6" lat="41.4010" lon="2.2000"/>
        <node id="7" lat="41.4000" lon="2.1990"/>
        <node id="8" lat="41.4000" lon="2.1970"/>
        <node id="9" lat="41.4010" lon="2.1990"/>
        <way id="50"><nd ref="8"/><nd ref="7"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        <way id="51"><nd ref="7"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        <way id="52"><nd ref="7"/><nd ref="9"/><tag k="highway" v="residential"/><tag k="oneway" v="yes"/></way>
        <way id="10"><nd ref="6"/><nd ref="1"/><nd ref="5"/><tag k="highway" v="residential"/></way>
        </osm>"#);
    let zone = &generated
        .zones
        .iter()
        .find(|z| z.id.starts_with("51#0_"))
        .expect("zone on 51#0")
        .polygon;
    let fork = generated.projection.to_local([2.1990, 41.4000]);
    let east = |metres: f64| Point::new(fork[0] + metres, fork[1]);
    assert!(
        !zone.contains(&east(0.5)),
        "the zone reaches into the fork's junction"
    );
    assert!(
        zone.contains(&east(8.0)),
        "the zone stops well short of the fork's junction"
    );
}

#[test]
fn a_light_just_after_a_junction_on_a_one_way_exit_leaves_the_junction_alone() {
    // Way 20 is one-way eastbound through junction 1; its light (12) and
    // signalized crossing (13) stand ~10m *after* the junction, where cars
    // that have already left it stop. Junction 1 itself (way 10 through it)
    // is unsignalized: only the light and the crosswalk form a controller.
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"/>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="3" lat="41.3990" lon="2.2000"/>
        <node id="12" lat="41.4000" lon="2.20012"><tag k="highway" v="traffic_signals"/></node>
        <node id="13" lat="41.4000" lon="2.20016"><tag k="highway" v="crossing"/><tag k="crossing" v="traffic_signals"/></node>
        <way id="20"><nd ref="5"/><nd ref="1"/><nd ref="12"/><nd ref="13"/><nd ref="4"/><tag k="highway" v="tertiary"/><tag k="oneway" v="yes"/></way>
        <way id="10"><nd ref="2"/><nd ref="1"/><nd ref="3"/><tag k="highway" v="residential"/></way>
        </osm>"#);
    assert_eq!(generated.programs.len(), 1);
    assert_eq!(generated.programs[0].tls_id, "12");
    let ids = vehicle_ids(&generated);
    assert!(
        ids.iter().all(|id| id.starts_with("20#")),
        "only way 20's traffic meets the light: {ids:?}"
    );
}

#[test]
fn a_cycleway_touching_the_road_between_a_crosswalk_and_its_junction_splits_nothing() {
    // Way 20 (one-way eastbound) crosses way 10 at junction 1; its
    // signalized crosswalk (13) stands ~10m before the junction, and a
    // cycleway touches way 20 at node 14 in between. The crosswalk still
    // belongs to junction 1's controller, and the few metres from it to the
    // junction are no approach of their own.
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.2000"><tag k="highway" v="traffic_signals"/></node>
        <node id="2" lat="41.4010" lon="2.2000"/>
        <node id="3" lat="41.3990" lon="2.2000"/>
        <node id="4" lat="41.4000" lon="2.2013"/>
        <node id="5" lat="41.4000" lon="2.1987"/>
        <node id="13" lat="41.4000" lon="2.19988"><tag k="highway" v="crossing"/><tag k="crossing" v="traffic_signals"/></node>
        <node id="14" lat="41.4000" lon="2.19994"/>
        <node id="15" lat="41.4003" lon="2.19994"/>
        <way id="20"><nd ref="5"/><nd ref="13"/><nd ref="14"/><nd ref="1"/><nd ref="4"/><tag k="highway" v="tertiary"/><tag k="oneway" v="yes"/></way>
        <way id="10"><nd ref="2"/><nd ref="1"/><nd ref="3"/><tag k="highway" v="residential"/></way>
        <way id="30"><nd ref="14"/><nd ref="15"/><tag k="highway" v="cycleway"/></way>
        </osm>"#);
    let car_programs: Vec<&str> = generated
        .programs
        .iter()
        .map(|p| p.tls_id.as_str())
        .collect();
    assert_eq!(car_programs, vec!["1"], "one controller");
    let ids = vehicle_ids(&generated);
    assert!(
        ids.iter().any(|id| id.starts_with("20#0_")),
        "way 20's approach runs up to the crosswalk: {ids:?}"
    );
    assert!(
        !ids.iter()
            .any(|id| id.starts_with("20#1_") || id.starts_with("20#2_")),
        "no approach between the crosswalk and the junction: {ids:?}"
    );
}

#[test]
fn a_two_stage_crossing_with_a_light_per_carriageway_is_two_zones() {
    // A dual carriageway (ways 20 eastbound, 21 westbound, ~12m apart),
    // crossed in two stages: crossing ways 30 and 31 meet on the median
    // (node 42), each with a signalized node on its own carriageway.
    let generated = run(r#"<osm>
        <node id="1" lat="41.4000" lon="2.1990"/>
        <node id="2" lat="41.4000" lon="2.2000"><tag k="highway" v="crossing"/><tag k="crossing" v="traffic_signals"/></node>
        <node id="3" lat="41.4000" lon="2.2010"/>
        <node id="4" lat="41.40011" lon="2.2010"/>
        <node id="5" lat="41.40011" lon="2.2000"><tag k="highway" v="crossing"/><tag k="crossing" v="traffic_signals"/></node>
        <node id="6" lat="41.40011" lon="2.1990"/>
        <node id="40" lat="41.39995" lon="2.2000"/>
        <node id="42" lat="41.400055" lon="2.2000"/>
        <node id="44" lat="41.40016" lon="2.2000"/>
        <way id="20"><nd ref="1"/><nd ref="2"/><nd ref="3"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        <way id="21"><nd ref="4"/><nd ref="5"/><nd ref="6"/><tag k="highway" v="primary"/><tag k="oneway" v="yes"/></way>
        <way id="30"><nd ref="40"/><nd ref="2"/><nd ref="42"/><tag k="highway" v="footway"/><tag k="footway" v="crossing"/></way>
        <way id="31"><nd ref="42"/><nd ref="5"/><nd ref="44"/><tag k="highway" v="footway"/><tag k="footway" v="crossing"/></way>
        </osm>"#);
    let mut pedestrian: Vec<&str> = generated
        .zones
        .iter()
        .filter(|z| z.class == Class::Pedestrian)
        .map(|z| z.id.as_str())
        .collect();
    pedestrian.sort_unstable();
    assert_eq!(pedestrian, vec!["2_ped", "5_ped"]);
}

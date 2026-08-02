# Waiting Zones

Command-line tool that turns a [SUMO](https://sumo.dlr.de/) road network
(`.net.xml`) into a `.waiting-zones.add.xml` file describing E3 detector
zones. The output format is meant to be consumed directly by a user device,
which has no knowledge of SUMO's XML formats.

## Usage

```sh
cargo run -- path/to/network.net.xml
```

By default the output is written next to the input file, replacing the
`.net.xml` suffix with `.waiting-zones.add.xml`. Use `-o`/`--output` to
choose a different path:

```sh
cargo run -- path/to/network.net.xml -o path/to/output.xml
```

By default a zone's entry spans the full length of its lane. Use
`--max-zone-length` (meters) to cap how far it extends from the stop line /
crossing:

```sh
cargo run -- path/to/network.net.xml --max-zone-length 20
```

## How it works

SUMO ships XML Schema (XSD) definitions for its file formats under `xsd/`.
At build time, `build.rs` feeds `xsd/net_file.xsd` to
[`xsd-parser`](https://crates.io/crates/xsd-parser) to generate matching Rust
types, patching a few SUMO-specific XSD quirks along the way (unresolved DTD
entities, type names that collide with XSD primitives, and a custom naming
strategy that keeps case-sensitive enumeration values like `state="M"` vs.
`state="m"` from colliding into the same Rust identifier).

The project follows a 3-layer model:

1. **`schema`** (generated, see `build.rs`) — an almost literal mirror of the
   SUMO XSDs. Not meant to be used directly outside of the conversion layer.
2. **`domain`** (`src/domain.rs`) — the project's own types (`Network`,
   `Edge`, `Lane`, `Junction`, `Connection`, ...), independent of SUMO/XSD.
   This is what the rest of the project is meant to build on.
3. **`schema_mapper`** (`src/schema_mapper.rs`) — converts layer 1 into
   layer 2, interpreting SUMO's text-encoded positions, shapes, boundaries,
   and enumerations along the way.

`src/processor.rs` drives the actual `.net.xml` → `.waiting-zones.add.xml`
conversion, and `src/config.rs` handles CLI argument parsing.

## Status

The schema generation, the schema-to-domain conversion layer, and the
`.net.xml` read / `.waiting-zones.add.xml` write pipeline in
`src/processor.rs` are in place and tested. `src/zone_generator.rs`
generates, per traffic-light junction, one vehicle waiting zone per
incoming lane's **signal group** (lanes whose `tlLogic` state character is
identical in every phase — e.g. a protected right turn gets its own zone,
separate from the straight-ahead lanes) and, symmetrically, one pedestrian
zone per signal group among the junction's crossings (with
`detectPersons="walk"`), spanning the walkingarea lane that approaches each
crossing (found via the `tl`-controlled `connection` from the walkingarea
into the crossing) rather than the crossing lane itself — a pedestrian on
the crossing is actively walking across, not waiting. If a traffic-light
junction has no matching `tlLogic` program (e.g.
`JunctionKind::TrafficLightUnregulated`, or an incomplete `.net.xml`), it's
skipped with a warning on stderr rather than guessing a grouping. Vehicle
vs. pedestrian lanes are told apart from SUMO's own `allow`/vClass data
(`allow="pedestrian"`), not the edge's `function` label — this also
correctly excludes sidewalk lanes and walkingareas that SUMO lists among a
junction's `incLanes` alongside the real vehicle lanes. Each generated
`e3Detector` also gets a `pos` (its icon position in editors like netedit) —
always the junction's own position, so every zone belonging to that
junction (vehicle and pedestrian alike) shares one icon in the editor
instead of each rendering separately along its own lane. Purely cosmetic:
SUMO itself ignores `pos` for detection.

Not yet addressed: gating pedestrian messages on being fully stopped (vs.
just present in the zone) — SUMO's `speedThreshold`/`timeThreshold`
attributes on `e3Detector` were the planned mechanism, but need their exact
semantics confirmed (they may only affect aggregated statistics, not
entry/exit membership) before wiring them in.

## Development

```sh
cargo build
cargo test
cargo clippy
```

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
separate from the straight-ahead lanes). If a traffic-light junction has no
matching `tlLogic` program (e.g. `JunctionKind::TrafficLightUnregulated`, or
an incomplete `.net.xml`), it's skipped with a warning on stderr rather
than guessing a grouping. Vehicle lanes are told apart from pedestrian ones
using SUMO's own `allow`/vClass data (`allow="pedestrian"`), not the edge's
`function` label — this correctly excludes sidewalk lanes and walkingareas
that SUMO lists among a junction's `incLanes` alongside the real vehicle
lanes. Each generated `e3Detector` also gets a `pos` (its icon position in
editors like netedit) — always the junction's own position, so every zone
belonging to that junction shares one icon in the editor instead of each
rendering separately along its own lane. Purely cosmetic: SUMO itself
ignores `pos` for detection.

### Pedestrian waiting zones: on hold

Pedestrian waiting zones (at signalized crossings) were implemented and
then removed. SUMO 1.26.0 has a reproducible crash in
`MSE3Collector::detectorUpdate` (confirmed with a gdb backtrace) when an
`e3Detector` with `detectPersons="walk"` is combined with real pedestrian
traffic. A workaround (`--pedestrian.model nonInteracting`) avoids the
crash, but then silently stops counting anyone — consistent with an
upstream comment noting that model doesn't fire the moveReminders detectors
rely on.

This looks like an architectural mismatch (vehicle-oriented detectors
retrofitted for pedestrians) rather than something fixable from this
project's side. Filed as a feature request for a dedicated pedestrian
detector: <https://github.com/eclipse-sumo/sumo/issues/18230>. Pedestrian
waiting zones will be revisited once SUMO has reliable native support.

## Development

```sh
cargo build
cargo test
cargo clippy
```

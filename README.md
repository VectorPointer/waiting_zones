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

Reading and modelling the SUMO network, and writing the `.add.xml` back out,
are **not** part of this repository: both are
[`sumo-types`](https://crates.io/crates/sumo-types), a separate crate
published to crates.io, resolved from there in `Cargo.toml` rather than by
path — a standalone clone of this repository builds on its own. It turns a
`.net.xml` into well-typed Rust structs (`Network`, `Edge`, `Lane`,
`Junction`, `Connection`, ...) and an E3 detector zone into `.add.xml`
(`additional::domain::E3Detector`/`DetectorGate`), and knows nothing about
waiting zones specifically — there's no "waiting zone" type of its own to
find in this crate either: an `E3Detector` already models exactly what one
is, so `zone_generator` builds `sumo_types` values directly instead of
maintaining a parallel type.

Everything left here is specific to waiting zones:

- `src/zone_generator.rs` — derives `E3Detector`s from a
  `sumo_types::Network`.
- `src/zone_output.rs` — fills in each detector's output-file path and
  writes them out as `.waiting-zones.add.xml`.
- `src/processor.rs` — drives the `.net.xml` → `.waiting-zones.add.xml`
  conversion, and `src/config.rs` handles CLI argument parsing.

## Status

The `.net.xml` reader (now in `sumo-types`) and the
`.waiting-zones.add.xml` write pipeline in
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

`sumo-types` is resolved from crates.io (see "How it works" above), not a
workspace member, so its own tests aren't run by the commands above. Run
those from its own directory if you're changing it too:

```sh
cd ../sumo-types && cargo test
```

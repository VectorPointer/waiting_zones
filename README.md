# Waiting Zones (OSM-only, experimental)

Generates waiting zones **and each junction's signal program** straight from
an OpenStreetMap extract (`.osm`), with no SUMO network in between. It
rebuilds, in its own terms, the slice of what `netconvert` derives that
waiting zones need: lanes, signalized junctions, movements, and a cycle.

```sh
cargo run --release -- data/barcelona/barcelona.osm --geojson out/barcelona.geojson
```

writes:

- `out/barcelona.vehicles.geojson` / `out/barcelona.pedestrians.geojson` —
  same schema as the SUMO-based generator: `waiting_zone_id`,
  `intersection_id` (the junction's `tls_id`), `stop_line`, `modes`;
- `out/barcelona.programs.json` — one entry per signalized junction, shaped
  like `engine_unit::EngineJunction` (`tls_id`, `zones[].phases`,
  `transitions`, `program.phases` as `(seconds, state)`), plus `links` (what
  each character of a state string controls) and `conflicts`.

## Pipeline

| module | what it does |
|---|---|
| `graph` | Roads split at junctions and signal nodes into directed edges (`{way}#{k}`, `-{way}#{k}`), with lanes per direction, `turn:lanes` arrows and lane offsets. |
| `clusters` | Signalized junctions: lights, signalized crossings and junction nodes linked by short edges (< 30m) form one controller — a dual carriageway crossing, or lights on the arms before a junction, are one `tls_id`. A light on a one-way road just *after* a junction stops cars that already left it: it isn't that junction's. |
| `movements` | Exits reachable from each approach through the junction; turn by angle; OSM `type=restriction`; lanes → turns (arrows when mapped, else right lane right, left lane left, the rest straight). |
| `zones` | One vehicle zone per group of an approach's lanes serving the same turns (`{edge}_{straight+left…}`), from the stop line back along the whole approach and on through predecessors that only lead there, until a fork or a signal (`--max-zone-length` caps the whole length). Bands are cut into runs where the lane jumps sideways or turns sharply, and the zone stays out of the junctions it ends at (the other arms' carriageways). |
| `program` | The cycle: opposite approaches paired (straight/right `G`, left `g` when it only yields to the opposite approach), a protected-left phase where the left has its own lane, remaining movements fitted greedily; crosswalks green when nothing crossing them goes straight (turns yield, `g`), a pedestrian phase otherwise; an amber after every green. |
| `pedestrians` | One zone per signalized crossing: the crosswalk (the mapped `footway=crossing`, its pieces chained back together, or a stripe across the road) 4m wide, carried 2m onto the sidewalk at both ends, linked to its junction's phases. An unsignalized crosswalk whose stripe touches a zone is absorbed into it (one that doesn't touch stays out: a zone is one place); signalized ones side by side over the same stretch of road are one zone, otherwise each is its own. |
| `output` | Vehicle zones carried up to the pedestrian zone in front of (or behind) each lane band and then cut by every pedestrian zone, so the two share a border and never ground; same-kind overlap splitting (vehicles along the bisector of their stop lines, pedestrian ground to whichever zone's crosswalk is nearest); holes filled; GeoJSON and `programs.json`. |
| `network_output` | With `--network <dir>`, the road graph itself, split for a simulator: `network.edges.json` (lanes, geometry, speed, successors), `network.junctions.json` (kind, signal, incident edges) and `network.connections.json` (signalized movements, with the approach lanes that serve each), plus `network.lock.json` — the SHA-256 of each data file and the run's `source_digest`. A reader verifies the whole set against the lock, so a file left over from an older run is detected instead of silently mixed in. |

`program::violations` checks every generated program: no two conflicting
movements both `G`, permissive only against the opposite approach,
never two straight movements from perpendicular approaches green together
(independent of the path geometry), no crosswalk green with a straight
movement across it, every link green in some phase, and every green
followed by an amber clearing it. The tests run it on synthetic junctions
and on all of real Barcelona.

## Known limits

- OSM has no signal timings: phases, order and durations follow the rules
  above, not the city's real program.
- `turn:lanes` is rarely mapped, so lane → turn assignment is mostly the
  default rule.
- A light with nothing mapped to alternate with (one approach, no crossing)
  gets a single, always-green phase.
- Movement paths are approximations (lane lines plus curves through the
  junction); crossing conflicts also use topology (a crosswalk on a
  movement's own nodes) to be safe where the geometry misses.
- No `.add.xml` / SUMO lane ids: a deployment whose engine and control plane
  want the neutral `map.json` compiles it from this generator's own output
  with `map-compile --osm-only <zones.programs.json> <zones.vehicles.geojson>
  <zones.pedestrians.geojson> <out/map.json> [territory]` (and
  `integration_test/promote.sh --osm-only`). The SUMO-based `sumo`/`demand`
  stand-ins still run off a `.net.xml`, so a closed loop on the OSM-only map
  is not part of this path.

## Tooling

- `cargo run --release --example compare -- <candidate.geojson> <reference.geojson>`
  compares stop lines and area against another catalogue (e.g. the
  SUMO-based one in `data/*/`).
- `viz/phases.html?net=<name>&tls=<id>&phase=<n>` steps through a junction's
  phases, colouring each zone by its state; generate its data with
  `--geojson data/<name>/zones.geojson` and serve `viz/`.

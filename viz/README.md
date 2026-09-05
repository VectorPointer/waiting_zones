# viz

A Leaflet map for eyeballing this crate's own generated output against the
real network for whichever dataset you pick — every waiting zone's polygon,
colored by pedestrian/vehicle, with search and click-to-highlight.

`viz.html` always fetches its dataset's own GeoJSON fresh on page load —
it's never hardcoded/embedded — so re-running the generator and reloading
the page is enough to see a fresh run, no editing needed.

## Usage

Each dataset lives in its own `viz/data/<name>/` directory. From the
`waiting_zones` crate root, for `barcelona`:

```sh
mkdir -p viz/data/barcelona
cargo run --release -- data/barcelona/barcelona.net.xml \
  -o viz/data/barcelona/zones.add.xml --geojson viz/data/barcelona/zones.geojson
cargo run --release --example net_to_geojson -- \
  data/barcelona/barcelona.net.xml viz/data/barcelona/net.geojson
cd viz && python3 serve.py
```

(`--geojson .../zones.geojson` writes `.../zones.vehicles.geojson` and
`.../zones.pedestrians.geojson` — see `geojson_output::write`'s own docs.
`net_to_geojson` is optional — see "Overlaying the real `.net.xml`" below.)

Then open `http://localhost:8000/viz.html?net=barcelona`, or just
`http://localhost:8000/viz.html` (defaults to `barcelona`) and switch
datasets from the dropdown next to the zone count — it round-trips through
the same `?net=` URL param, so a link to a specific dataset is shareable.
The dropdown's own options are hardcoded in `viz.html`'s `DATASETS`
constant (kept in sync by hand with `serve.py`'s own `KNOWN_DATASETS` —
see its docs on why); add a new dataset to both when you add one.

Must be served over `http://`, not opened as a `file://` URL (browsers
block local `fetch()` for that). Use `serve.py`, not plain
`python3 -m http.server`: the latter sends no `Cache-Control` header, so a
browser can keep serving a stale `viz.html` or `net.geojson` after a
completely normal reload, well after regenerating the real files — see
`serve.py`'s own docs for why. If a page still looks stale served from
here, a hard reload (Ctrl+Shift+R) or private window rules out cache
entirely.

`viz/data/` is gitignored: regenerate a dataset's files with the commands
above rather than expecting them to already be there.

## Overlaying the real `.net.xml`

The basemap is OSM tiles, which can drift from the actual `.net.xml` (OSM
gets edited after the network snapshot is taken). To compare the generated
zones against the network SUMO actually used, rather than against OSM's
current state, generate that dataset's own `net.geojson` (see the `Usage`
command above) and toggle it on under "network (.net.xml)" in the layers
panel (off, and not fetched, by default — at ~35k lane features it's over
10x heavier than `zones.vehicles.geojson` and `zones.pedestrians.geojson`
combined).

It's produced by `examples/net_to_geojson.rs`, which reprojects every lane's
own shape with the same PROJ4 pipeline `geojson_output` uses for zones, so
the two layers share one coordinate frame.

`net.geojson` bundles three toggleable layers:
- **lane centerlines** — one thin line per lane.
- **lane footprints (full width)** — each lane's centerline buffered by its
  own real `width` (same technique `geometry.rs::buffer_shape` uses for
  zones), colored by vehicle/pedestrian to match zone colors — lets you
  check "does this zone span its lane's *full* width" directly instead of
  eyeballing a bare centerline. Off by default and filtered to `normal`
  edges only (junction-internal footprints overlap heavily; see the
  `showAllNetFunctions` flag in `viz.html` for the junction-level detail).
- **traffic lights** — one marker per signal-controlled *lane*, at SUMO's
  own stop-line point, using the exact same rule `zone_generator`'s
  `signal_controlled_lanes` does. Click a marker for its traffic light
  id(s), lane, and one link index/state per movement that lane carries —
  a lane with a shared straight+left+right movement lists all three under
  one marker rather than drawing three indistinguishable ones stacked on
  the same point (`examples/net_to_geojson.rs`'s own docs on the "double
  row of traffic lights" that used to look like on real Eixample data).

## Inspecting a zone's own vertices

Clicking any zone opens a side panel listing every one of its own vertices'
exact `[lon, lat]`, each also marked with a small draggable marker on the
map — clicking a row centres the map on that one vertex, for checking a
suspiciously sharp corner without hunting for it by eye first. Close with
the panel's own `×`, or click its header to collapse/expand it without
losing the selection.

The panel also lets you edit the shape directly, in three ways:
- **Move**: drag any marker on the map to a new position.
- **Delete**: click a row's own `×`, or select it (click its row or
  marker) and press Delete/Supr. Can't shrink a ring below a triangle (3
  vertices).
- **Insert**: click a row's own `+` to add a new vertex right after it, at
  the midpoint of the edge to its own next vertex — then drag it into
  place.

If an edit leaves the ring crossing itself, the panel warns and the save
buttons below refuse to save until it's fixed — `generate()` never
produces a self-intersecting ring, so saving one wouldn't document a
reachable target.

## Jumping to a zone that already has a fixture

The "Zonas con expected…" dropdown next to the dataset selector lists
every `waiting_zone_id` that appears in some `tests/fixtures/*/
expected.geojson` (via `serve.py`'s own `/api/fixture_zone_ids`),
narrowed to whichever of those are actually in the currently loaded
dataset. Picking one does exactly what clicking that zone's own search
result does — pans/zooms to it, highlights it, opens its vertex inspector
— and then resets itself back to the placeholder, so it's a jump-to
action rather than a sticky selection.

**→ and ← then step to the next and previous zone in that list**, so
reviewing every fixture zone in turn is two keys rather than a trip back
to the dropdown for each one. The placeholder doubles as the readout of
where you are (`Zonas con expected (4/139) ←→`), since the menu itself
resets after every pick.

The arrows only belong to the list while a zone from *it* is selected:
picking a zone any other way (the search box, a click on the map) hands
them straight back to Leaflet as the map's own panning controls. Within
the list they stay the list's all the way to both ends, where they simply
do nothing rather than pan the map out from under the zone you're looking
at.

**"solo con diferencias"**, next to the dropdown, narrows that same list
to zones whose committed `expected.geojson` shape and current live
`generate()` output actually disagree — the same "would this fail `cargo
test`" question `tests/zone_fixtures.rs`'s own
`every_fixture_matches_its_own_expected_geojson` answers, just browsable
zone by zone instead of read off a test failure message. Uses the exact
same criterion that test does (`differingAreaFraction`'s own docs): half
a percent of a zone's own area, measured with `turf.difference` rather
than a hand-rolled check, so "shows up here" and "the Rust test flags it"
mean the same thing. A handful of real shapes make `turf.difference`
itself throw (a known gap between what a browser-side library and this
crate's own Rust-side `geo`/`i_overlay` pipeline can reliably compute on
the same near-degenerate vertices — logged once, as a running count, if
it happens) — those pairs fall back to "no difference found" for
whichever direction failed, so a real mismatch could go unflagged here
but never falsely flagged.

## Comparing "actual" against a saved "expected"

The vertex panel has two checkboxes, "Actual" and "Expected". "Actual" is
this zone's own live shape (whatever the currently loaded dataset's own
`generate()` run produced) — on by default. "Expected" is whatever's
saved in a `tests/fixtures/*/expected.geojson` for this zone, if any,
drawn as a green dashed outline; disabled when this zone has no fixture
at all. Turn both on to compare them directly, both off to see neither.

This is also what fixes what used to feel like "saving didn't do
anything" — a successful save now re-fetches and turns "Expected" on by
itself, so the shape that just got written to disk is immediately visible
instead of the map silently looking exactly like it did before you saved.

## Saving a zone as a test fixture

The same panel's "Guardar esta zona" / "Guardar intersección" buttons save
the selected zone's own real `.net.xml` neighbourhood, plus this crate's
own generated output for it, straight into `tests/fixtures/<name>/` (see
that directory's own README for what gets written and why) — no need to
run `extract_fixture`/`write_expected_geojson` by hand. "Guardar esta
zona" names the fixture after the one zone; "Guardar intersección" names
it after the junction and captures every zone touching it. Both are
disabled for a zone with no `intersection_id` (nothing to extract
*around* — see `zone_feature::build_feature`'s own docs on when that's
missing).

This only works for a dataset `serve.py`'s own `KNOWN_DATASETS` recognizes
(kept in sync with `viz.html`'s `DATASETS`, per the `Usage` section above)
and only against that dataset's *own* source `.net.xml` under
`data/<dataset>/<dataset>.net.xml` — not against whatever's currently
loaded into `viz/data/`, which may be a fixture's own small extract rather
than the full network (see "Visualizing a test fixture" below).

**Editing the shape first**: see "Inspecting a zone's own vertices" above
for how to move, delete, or insert a vertex before saving — a spike, a
stray point from a `.net.xml` quirk, a corner that's slightly off.

Saving with any such edit writes an `expected.geojson` the *current* code
does not match — that fixture's own `cargo test` will fail on purpose,
immediately, until the algorithm is fixed to actually produce that shape.
That's the intended use: it documents a concrete, already-diagnosed shape
bug as an executable target instead of a comment. See
`tests/fixtures/README.md`'s own "Aspirational fixtures" section before
committing one of these.

## Visualizing a test fixture

To see a fixture's own small network/output through the exact same
pipeline real data goes through, rather than trusting `expected.geojson`
on the strength of a diff alone:

```sh
mkdir -p viz/data/fixture
cargo run --release -- tests/fixtures/<name>/network.net.xml \
  -o viz/data/fixture/zones.add.xml --geojson viz/data/fixture/zones.geojson
```

Add `"fixture"` to `viz.html`'s `DATASETS` (and `serve.py`'s
`KNOWN_DATASETS`, if you also want to save *from* this view — usually not,
since a fixture's own network is already the extract) locally, then open
`?net=fixture`. Don't commit that `DATASETS` edit; it's a local-only way to
look at one fixture, not a real dataset.

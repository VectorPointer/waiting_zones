# Zone fixtures

One directory per reviewed junction:

- `fixture.json` — `{"dataset": "barcelona", "junction_id": "…"}`;
- `expected.geojson` — that junction's zones as they should be;
- `excluded_zones.json` (optional) — zone ids this fixture doesn't vouch for.

`tests/zone_fixtures.rs` regenerates the dataset from
`data/<dataset>/<dataset>.osm` and compares that junction's zones against
`expected.geojson`: the same ids, and each zone's ground within 0.5% of its
area.

## Adding a fixture

1. Regenerate the viewer's data:
   `cargo run --release -- data/barcelona/barcelona.osm --geojson data/barcelona/zones.geojson`.
2. Serve the viewer with `python3 viz/serve.py 8767` (plain `http.server`
   has no save endpoint) and open `http://127.0.0.1:8767/viz.html`.
3. Pick a zone, optionally hand-edit its vertices, and press "Guardar esta
   zona" (fixture named after the zone) or "Guardar intersección" (named
   after the junction). Every zone of that junction is saved as shown.

## Aspirational fixtures

A hand-edited zone documents the shape it *should* have: its fixture
fails until the generator produces that shape. That's the point — it's a
target, not a verified result.

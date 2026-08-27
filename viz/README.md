# viz

A Leaflet map for eyeballing this crate's own generated output against the
real Barcelona network — every waiting zone's polygon, colored by
pedestrian/vehicle, with search and click-to-highlight.

`viz.html` always fetches `./zones.geojson` fresh on page load — it's never
hardcoded/embedded — so re-running the generator and reloading the page is
enough to see a fresh run, no editing needed.

## Usage

From the `waiting_zones` crate root:

```sh
cargo run --release -- data/barcelona/barcelona.net.xml \
  -o viz/zones.add.xml --geojson viz/zones.geojson
cd viz && python3 -m http.server 8000
```

Then open `http://localhost:8000/viz.html`. (Must be served over `http://`,
not opened as a `file://` URL — browsers block local `fetch()` for that.)

`zones.add.xml`/`zones.geojson` are gitignored: regenerate them with the
command above rather than expecting them to already be there.

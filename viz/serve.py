#!/usr/bin/env python3
"""Serves this directory like `python3 -m http.server`, but with caching
disabled, plus one POST endpoint `viz.html`'s own "save as fixture" button
uses.

Plain `http.server` sends `Last-Modified` but no `Cache-Control` header, so
a browser can (per RFC 7234's heuristic-freshness fallback for a response
with no explicit cache directive) keep serving a stale `viz.html` or
`net.geojson` from its disk cache after a completely normal reload -- not
just the `file://` mistake `viz.html`'s own `#error` box warns about. This
bit real debugging sessions on this exact page more than once: regenerating
the data files and reloading should always be enough to see a fresh run,
and silently wasn't. If it still looks stale even served from here, a hard
reload (Ctrl+Shift+R / Cmd+Shift+R) or a private window rules out cache
entirely.

Usage: python3 serve.py [port]  (default 8000, same as http.server's own)
"""

import http.server
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent

# Kept in sync by hand with `viz.html`'s own `DATASETS` constant -- see that
# file's own comment on why `test_4x4_ped` and `linkoping` aren't in it.
KNOWN_DATASETS = {"barcelona", "eixample", "test_4x4_3lanes"}

# A fixture directory name, or a `.net.xml` junction id -- neither is ever
# meant to hold a path separator or a shell metacharacter, so this doubles
# as the actual security boundary (not just a filesystem-safety check):
# every value that reaches `subprocess.run` below is validated against this
# *before* it's used to build a path or an argument, and `subprocess.run` is
# always called with an argument list (never `shell=True`), so even a
# validation gap here couldn't inject a second command.
SAFE_NAME = re.compile(r"^[A-Za-z0-9_-]{1,100}$")

# The same substitution `_handle_fixture_zone_ids` and
# `_handle_remove_expected_zone` both need: whether a fixture directory's
# own name is a given zone id's "dedicated" one (the "Guardar esta zona"
# naming convention), not one that merely happens to also contain that
# id as an incidental bystander (see `_handle_fixture_zone_ids`'s own
# docs on why that distinction matters).
SAFE_NAME_CHARS = re.compile(r"[^A-Za-z0-9_-]")

CARGO_TIMEOUT_SECONDS = 180


def _validate_geometry(geometry):
    """Structural sanity check on a client-submitted GeoJSON geometry
    (`viz.html`'s own "delete vertex" edit) before it's ever written to
    disk -- this is a local dev tool with no auth, but the payload still
    crosses a trust boundary (browser JS -> this process -> a file this
    process then writes), so a malformed or malicious body should fail
    loudly here rather than corrupt a fixture or crash mid-write.
    """
    if not isinstance(geometry, dict) or geometry.get("type") not in ("Polygon", "MultiPolygon"):
        raise ValueError("edited_geometry must be a Polygon or MultiPolygon")
    coordinates = geometry.get("coordinates")
    if not isinstance(coordinates, list) or not coordinates:
        raise ValueError("edited_geometry has no coordinates")

    def check_ring(ring, path):
        if not isinstance(ring, list) or len(ring) < 4:
            raise ValueError(f"edited_geometry ring at {path} needs at least 4 points (closed >=triangle)")
        for point in ring:
            if not isinstance(point, list) or len(point) != 2 or not all(isinstance(v, (int, float)) for v in point):
                raise ValueError(f"edited_geometry has a non-numeric [lon, lat] pair at {path}")

    if geometry["type"] == "Polygon":
        for i, ring in enumerate(coordinates):
            check_ring(ring, f"ring {i}")
    else:
        for i, polygon in enumerate(coordinates):
            if not isinstance(polygon, list) or len(polygon) != 1:
                raise ValueError(f"edited_geometry part {i} must be a single ring (no holes)")
            check_ring(polygon[0], f"part {i} ring 0")


def _apply_geometry_override(expected_geojson_path, zone_id, geometry):
    """Overwrites one feature's own `geometry` in an already-written
    `expected.geojson` with a hand-edited one (see `viz.html`'s own
    "delete vertex" note) -- this makes that fixture's own comparison
    against a fresh `generate()` run fail on purpose, until the algorithm
    stops producing the deleted point(s); it's a documented target, not a
    verified-correct result.
    """
    collection = json.loads(expected_geojson_path.read_text())
    for feature in collection.get("features", []):
        if feature.get("properties", {}).get("waiting_zone_id") == zone_id:
            feature["geometry"] = geometry
            expected_geojson_path.write_text(json.dumps(collection, indent=2) + "\n")
            return
    raise ValueError(f"no feature with waiting_zone_id {zone_id!r} in {expected_geojson_path}")


class NoCacheHandler(http.server.SimpleHTTPRequestHandler):
    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        if self.path == "/api/fixture_zone_ids":
            self._handle_fixture_zone_ids()
            return
        super().do_GET()

    def do_POST(self):
        if self.path == "/api/save_fixture":
            self._handle_save_fixture()
        elif self.path == "/api/remove_expected_zone":
            self._handle_remove_expected_zone()
        else:
            self.send_error(404)

    def _handle_fixture_zone_ids(self):
        """Every `waiting_zone_id` that appears in some committed (or
        aspirational -- this doesn't distinguish) `tests/fixtures/*/
        expected.geojson`, with that feature's own geometry and which
        fixture it came from -- `viz.html`'s own "zonas con expected" menu
        uses the id list to offer exactly the zones it can jump straight
        to (the same way its search box already does for every zone id in
        the currently loaded dataset), and its "ver expected guardado"
        toggle/`showVertexInspector`'s own re-editing seed use the
        geometry directly, without a second request per zone.

        A fixture's own extraction radius around one junction (see
        `extract_fixture.rs`) sweeps in every zone near it, *regardless*
        of whether "Guardar esta zona" or "Guardar intersección" was
        clicked -- the two differ only in what the fixture gets *named*,
        never in which zones its own `expected.geojson` ends up holding.
        So the *same* `waiting_zone_id` routinely turns up, unedited, as
        an incidental bystander inside some *other* zone's own dedicated
        fixture too -- and picking whichever copy has the newest mtime
        (this function's own first version) got this exactly backwards on
        real data: saving zone B's own fixture regenerates *A*'s own
        bystander copy fresh (live, unedited) with a newer mtime than
        A's own dedicated fixture ever has, so the menu/re-edit seed
        would silently show a hand-edit's *own* zone as if it had never
        been touched, even though the real edit was sitting untouched in
        its own dedicated fixture the whole time.

        Fixed by trusting the naming convention over recency: a fixture
        whose own directory name matches this exact zone id (the "Guardar
        esta zona" case for *this* zone specifically) always wins over
        any fixture that merely happens to also contain it. Only when no
        such dedicated fixture exists at all does mtime decide among the
        remaining (incidental-bystander-only) candidates -- the same
        "latest wins" fallback as before, just no longer the primary rule.
        """
        best = {}  # zone_id -> (is_dedicated, mtime, fixture_name, geometry)
        for expected_path in (PROJECT_ROOT / "tests" / "fixtures").glob("*/expected.geojson"):
            try:
                collection = json.loads(expected_path.read_text())
                mtime = expected_path.stat().st_mtime
            except OSError:
                continue
            except json.JSONDecodeError:
                continue
            fixture_name = expected_path.parent.name
            for feature in collection.get("features", []):
                zone_id = feature.get("properties", {}).get("waiting_zone_id")
                geometry = feature.get("geometry")
                if not zone_id or not geometry:
                    continue
                is_dedicated = fixture_name == SAFE_NAME_CHARS.sub("_", zone_id)
                candidate = (is_dedicated, mtime)
                current = best.get(zone_id)
                if current is None or candidate > (current[0], current[1]):
                    best[zone_id] = (is_dedicated, mtime, fixture_name, geometry)
        zones = {zone_id: {"fixture": v[2], "geometry": v[3]} for zone_id, v in best.items()}
        self._send_json(200, {"zones": zones})

    def _send_json(self, status, payload):
        body = json.dumps(payload).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _run_cargo_example(self, example, args):
        result = subprocess.run(
            ["cargo", "run", "--release", "--example", example, "--", *args],
            cwd=PROJECT_ROOT,
            capture_output=True,
            text=True,
            timeout=CARGO_TIMEOUT_SECONDS,
        )
        if result.returncode != 0:
            raise RuntimeError(f"{example} failed:\n{result.stderr}")

    def _handle_remove_expected_zone(self):
        """Stops vouching for one zone's own `expected.geojson` entry,
        across every fixture that has one -- the server side of
        `viz.html`'s "quitar de expected" button (see that button's own
        docs for why this exists: a zone swept in as an incidental
        bystander by "Guardar interseccion", whose committed shape turned
        out wrong because the fixture's own small-radius extract isn't
        faithful for it specifically, not because `generate()` itself is
        buggy).

        For each fixture directory whose own `expected.geojson` contains
        a feature with this `waiting_zone_id`:

        - If that fixture is this zone's own *dedicated* one (its
          directory name matches the "Guardar esta zona" naming
          convention -- see `SAFE_NAME_CHARS`'s own docs) and removing
          this one feature would leave none behind, the whole fixture
          directory is deleted outright: a fixture that existed only to
          vouch for this one now-known-wrong zone has nothing left to do
          once that vouching is withdrawn, and leaving an empty
          `expected.geojson` plus its own `network.net.xml` behind would
          just be dead weight nobody has a reason to clean up later.
        - Otherwise (an "interseccion" fixture with other, still-valid
          zones, or a dedicated fixture that still has something left):
          the one feature is removed from `expected.geojson`, and this
          zone id is recorded in that fixture's own
          `excluded_zones.json` (created if absent) -- see
          `tests/zone_fixtures.rs`'s own "Excluded zones" docs for why a
          bare deletion alone isn't enough: `generate()` still produces
          this zone from that fixture's own network, so without the
          exclusion list `every_fixture_matches_its_own_expected_geojson`
          would immediately (and correctly, by its own rules) start
          reporting it as "unexpected new".
        """
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length))
            zone_id = body["zone_id"]
            if not isinstance(zone_id, str) or not zone_id:
                raise ValueError("zone_id must be a non-empty string")

            fixtures_root = PROJECT_ROOT / "tests" / "fixtures"
            dedicated_name = SAFE_NAME_CHARS.sub("_", zone_id)
            updated, deleted = [], []
            for expected_path in fixtures_root.glob("*/expected.geojson"):
                fixture_dir = expected_path.parent
                collection = json.loads(expected_path.read_text())
                features = collection.get("features", [])
                remaining = [f for f in features if f.get("properties", {}).get("waiting_zone_id") != zone_id]
                if len(remaining) == len(features):
                    continue  # this fixture never had the zone to begin with

                if fixture_dir.name == dedicated_name and not remaining:
                    shutil.rmtree(fixture_dir)
                    deleted.append(fixture_dir.name)
                    continue

                collection["features"] = remaining
                expected_path.write_text(json.dumps(collection, indent=2) + "\n")

                excluded_path = fixture_dir / "excluded_zones.json"
                excluded = json.loads(excluded_path.read_text()) if excluded_path.is_file() else []
                if zone_id not in excluded:
                    excluded.append(zone_id)
                excluded_path.write_text(json.dumps(sorted(excluded), indent=2) + "\n")
                updated.append(fixture_dir.name)

            if not updated and not deleted:
                raise ValueError(f"zone_id {zone_id!r} isn't in any fixture's own expected.geojson")

            self._send_json(200, {"ok": True, "updated": updated, "deleted": deleted})
        except Exception as error:  # noqa: BLE001 -- reported to the caller, not swallowed
            self._send_json(400, {"ok": False, "error": str(error)})

    def _handle_save_fixture(self):
        try:
            length = int(self.headers.get("Content-Length", 0))
            body = json.loads(self.rfile.read(length))
            dataset = body["dataset"]
            junction_id = body["junction_id"]
            name = body["name"]
            edited_zone_id = body.get("edited_zone_id")
            edited_geometry = body.get("edited_geometry")

            if dataset not in KNOWN_DATASETS:
                raise ValueError(f"unknown dataset {dataset!r}")
            for value, label in [(junction_id, "junction_id"), (name, "name")]:
                if not SAFE_NAME.match(value):
                    raise ValueError(f"{label} {value!r} isn't a safe fixture/junction name")
            if (edited_zone_id is None) != (edited_geometry is None):
                raise ValueError("edited_zone_id and edited_geometry must be given together")
            if edited_geometry is not None:
                _validate_geometry(edited_geometry)

            net_xml = PROJECT_ROOT / "data" / dataset / f"{dataset}.net.xml"
            if not net_xml.is_file():
                raise ValueError(f"{net_xml} does not exist")
            fixture_dir = PROJECT_ROOT / "tests" / "fixtures" / name

            self._run_cargo_example("extract_fixture", [str(net_xml), junction_id, str(fixture_dir)])
            self._run_cargo_example("write_expected_geojson", [str(fixture_dir)])

            if edited_geometry is not None:
                _apply_geometry_override(fixture_dir / "expected.geojson", edited_zone_id, edited_geometry)

            self._send_json(200, {"ok": True, "path": f"tests/fixtures/{name}"})
        except Exception as error:  # noqa: BLE001 -- reported to the caller, not swallowed
            self._send_json(400, {"ok": False, "error": str(error)})


if __name__ == "__main__":
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 8000
    with http.server.ThreadingHTTPServer(("", port), NoCacheHandler) as httpd:
        print(f"Serving on http://localhost:{port} (Cache-Control: no-store)")
        httpd.serve_forever()

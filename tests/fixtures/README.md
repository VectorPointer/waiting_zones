# tests/fixtures

Small, *real* `.net.xml` extracts — a handful of junctions cut out of a
real city network, not a hand-built synthetic one — each checked by
`tests/zone_fixtures.rs` against its own committed `expected.geojson`.

## What "matches" means

**By area, not vertex by vertex.** Two things are compared per fixture:

- the *set* of `waiting_zone_id`s, exactly — an id is a discrete name, so
  a zone appearing, vanishing or being renamed always wants a human;
- each zone's *ground*, to within half a percent of its own area — how
  much the committed and freshly generated polygons disagree about,
  measured symmetrically so growing and shrinking are caught alike.

This used to be an exact coordinate comparison to within a tenth of a
millimetre, and that was the wrong contract. A zone's polygon comes out
of a long chain of boolean operations over floating-point geometry, and
that output is *stable* but not *canonical*: a `HashMap` iteration order
feeding a union in a different sequence, a `simplify` pass keeping a
neighbouring point instead, a cleanup pass dropping a vertex worth a
square millimetre — each rewrites the vertex list while describing the
same piece of road. Under exact comparison every one of those read as a
regression, so fixtures failed for reasons nobody could act on and the
only available response was to regenerate them, which silently accepts
whatever *else* changed too. That is how a golden-file suite stops being
worth reading.

Half a percent of a zone's area is well below anything a client can
observe — it geofences one GPS position against the polygon
(`resolver::catalogue`), and a GPS fix resolves metres — while every
regression worth catching (an extension reaching a block further, an
overlap cut taking the stop-line end, a zone landing on the neighbouring
lane) moves far more, and is reported as a percentage and an area in m²
rather than as a coordinate diff nobody can read.

Real extracts, not synthetic networks, because the messiest defects this
crate has actually hit only show up in real `netconvert` output's own
quirks — a `joinTLS` program spanning 15 junctions, a 0.2m connector edge,
a via lane's own sharply curved shape — none of which anyone would think
to reproduce by hand, and a hand-built fixture that doesn't reproduce them
can't catch a regression in handling them.

## Layout

```
tests/fixtures/<name>/
  network.net.xml    # small, real extract — committed
  expected.geojson   # this crate's own output for it, last confirmed correct — committed
```

`tests/zone_fixtures.rs` discovers every subdirectory with a
`network.net.xml` automatically — adding a fixture never touches that
file.

## Adding a fixture

1. **Extract** a small real subset around the junction you care about,
   from any `.net.xml` you already have (real Barcelona data, or your
   own):

   ```sh
   cargo run --release --example extract_fixture -- \
     <source.net.xml> <junction-id> tests/fixtures/<name>
   ```

   `<junction-id>` is the junction whose own signal controls the zone you
   want covered — see `examples/extract_fixture.rs`'s own docs for exactly
   what gets kept (everything within a real, not hop-count, walking
   distance along the connection graph — 150m by default, comfortably past
   `zone_generator::DEFAULT_EXTENSION_METERS`'s own 120m).

2. **Verify the extract is faithful** before trusting it for anything:
   run the normal CLI against both the *original* source network and the
   new extract, and confirm your target zone's own feature is
   byte-identical between the two (the extract should never change a
   zone's own output, only which *other* zones exist alongside it).

3. **Generate `expected.geojson`**:

   ```sh
   cargo test --release -- --ignored regenerate_expected_geojson
   ```

4. **Visually confirm it** — see `viz/README.md`'s own "visualizing a test
   fixture" section. Trusting a fixture's `expected.geojson` on the
   strength of a diff alone defeats the point of having one.

5. Commit both `network.net.xml` and `expected.geojson`.

## Updating a fixture after a deliberate geometry change

Same steps 3–5 above: regenerate, visually re-confirm, commit. If the
regenerated shape *isn't* an improvement, that's `cargo test` doing its
job — figure out why before regenerating over it.

## Aspirational fixtures (a zone whose current shape is wrong)

`viz.html`'s vertex inspector (see `viz/README.md`) lets you delete,
move, or insert a vertex on a zone — a spike, a stray point from a
`.net.xml` quirk, a corner that's slightly off — before clicking "Guardar
esta zona"/"Guardar intersección". None of that is guaranteed to land
back on anything `generate()` itself would ever produce: this is a
*hand-authored target*, not a verified-correct result the way a plain,
unedited save is. The vertex panel refuses to save a self-intersecting
edit (`generate()` never produces one, so it couldn't document a
reachable target either), but beyond that, nothing stops you from
authoring a shape the algorithm has no path to today — that's expected,
and exactly what the rest of this section is about.

Saving with any such edit writes an `expected.geojson` that the *current*
code does **not** match — `cargo test`'s `every_fixture_matches_its_own_
expected_geojson` will fail for that fixture, on purpose, immediately.
That failure is the point: it documents "the shape should look like this"
as an executable target, not a lie that pretends the algorithm already
gets it right. Treat a failing fixture here the same way as a failing
test anywhere else in this crate — a real, tracked bug — and don't commit
a red one silently:

- If you're about to fix the underlying algorithm bug this fixture
  documents, leave it red until your fix makes it pass again.
- If you're just capturing the target for someone else (or later) to fix,
  say so in the commit message (e.g. "known-red: <fixture> documents the
  spike at vertex N until <issue/description> is fixed") so a red
  `cargo test` doesn't look like an accidental regression.

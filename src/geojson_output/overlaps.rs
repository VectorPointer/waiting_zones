use anyhow::Result;
use anstream::eprintln;
use anstyle::{AnsiColor, Style};
use geo::algorithm::area::Area;
use geo::{
    BooleanOps, BoundingRect, Contains, Coord, LineString, MapCoords, MultiPolygon, Point as GeoPoint,
    Polygon as GeoPolygon,
};
use geojson::{Feature, FeatureCollection, Position};
use std::collections::HashMap;
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::Lane;
use crate::geojson_output::geometry::zone_polygon;
use crate::geojson_output::reprojection::MIN_DRAWN_LANE_LENGTH_METERS;

pub const ERROR: Style = AnsiColor::Red.on_default().bold();

pub fn feature_rings(feature: &Feature) -> Vec<&[Position]> {
    let Some(geojson::GeometryValue::MultiPolygon { coordinates }) =
        feature.geometry.as_ref().map(|g| &g.value)
    else {
        return Vec::new();
    };
    coordinates.iter().filter_map(|polygon| polygon.first().map(Vec::as_slice)).collect()
}

pub fn feature_multipolygon(feature: &Feature) -> MultiPolygon<f64> {
    let rings = feature_rings(feature).into_iter().map(|ring| {
        GeoPolygon::new(LineString::new(ring.iter().map(|p| Coord { x: p[0], y: p[1] }).collect()), Vec::new())
    });
    MultiPolygon::new(rings.collect())
}

pub const OVERLAP_AREA_THRESHOLD_DEG2: f64 = OVERLAP_AREA_THRESHOLD_M2 / (84_000.0 * 111_000.0);

pub fn overlapping_zone_ids(collection: &FeatureCollection) -> Vec<(String, String)> {
    let ids: Vec<Option<String>> = collection
        .features
        .iter()
        .map(|feature| feature.property("waiting_zone_id").and_then(|v| v.as_str()).map(str::to_string))
        .collect();
    let polygons: Vec<MultiPolygon<f64>> = collection.features.iter().map(feature_multipolygon).collect();

    let mut found = Vec::new();
    for i in 0..polygons.len() {
        for j in (i + 1)..polygons.len() {
            if polygons[i].intersection(&polygons[j]).unsigned_area() > OVERLAP_AREA_THRESHOLD_DEG2
                && let (Some(a), Some(b)) = (&ids[i], &ids[j])
            {
                found.push((a.clone(), b.clone()));
            }
        }
    }
    found
}

pub const OVERLAP_AREA_THRESHOLD_M2: f64 = 0.01;

pub fn overlap_area_m2(a: &MultiPolygon<f64>, b: &MultiPolygon<f64>) -> f64 {
    let boxes_overlap = match (a.bounding_rect(), b.bounding_rect()) {
        (Some(ra), Some(rb)) => {
            ra.min().x <= rb.max().x && rb.min().x <= ra.max().x && ra.min().y <= rb.max().y && rb.min().y <= ra.max().y
        }
        _ => false,
    };
    // `intersection` is the expensive part; most pairs across a whole city
    // are nowhere near each other, so a bounding-box check first is worth
    // it the same way it was for the pipeline this replaces.
    if boxes_overlap { a.intersection(b).unsigned_area() } else { 0.0 }
}

pub fn keep_part_near(polygon: MultiPolygon<f64>, reference: Coord<f64>) -> MultiPolygon<f64> {
    if polygon.0.len() <= 1 {
        return polygon;
    }
    let reference_point = GeoPoint::from(reference);
    // Prefer the part that actually contains the stop line first: distance
    // alone can be fooled by a tiny sliver that happens to sit closer to the
    // reference point than the real, substantially larger remaining piece
    // (seen on 50926861#0_straight, where a ~0.28 m^2 scrap sat nearer the
    // stop line than the zone's real remaining ground). Falling back to the
    // largest part by area is a much more robust proxy for "the real
    // surviving ground" than raw nearest-by-distance.
    let containing = polygon
        .0
        .iter()
        .find(|part| part.contains(&reference_point))
        .cloned();
    let chosen = containing.or_else(|| {
        polygon
            .0
            .into_iter()
            .max_by(|a, b| a.unsigned_area().total_cmp(&b.unsigned_area()))
    });
    MultiPolygon::new(chosen.into_iter().collect())
}

pub const MIN_KEPT_PART_AREA_M2: f64 = 0.01;

pub fn drop_slivers(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    MultiPolygon::new(polygon.0.into_iter().filter(|part| part.unsigned_area() > MIN_KEPT_PART_AREA_M2).collect())
}

pub const SNAP_GRID_METERS: f64 = 0.0001;

pub fn snap_coords(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    let snap = |v: f64| (v / SNAP_GRID_METERS).round() * SNAP_GRID_METERS;
    polygon.map_coords(|c| Coord { x: snap(c.x), y: snap(c.y) })
}

pub fn overlapping_pairs(polygons: &[MultiPolygon<f64>]) -> Vec<(usize, usize)> {
    let mut found = Vec::new();
    for i in 0..polygons.len() {
        for j in (i + 1)..polygons.len() {
            if overlap_area_m2(&polygons[i], &polygons[j]) > OVERLAP_AREA_THRESHOLD_M2 {
                found.push((i, j));
            }
        }
    }
    found
}

pub fn still_overlapping(polygons: &[MultiPolygon<f64>], pairs: &[(usize, usize)]) -> Vec<(usize, usize)> {
    pairs
        .iter()
        .copied()
        .filter(|&(i, j)| overlap_area_m2(&polygons[i], &polygons[j]) > OVERLAP_AREA_THRESHOLD_M2)
        .collect()
}

pub fn max_safe_pad(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    upper: f64,
    opposing: &MultiPolygon<f64>,
) -> Result<f64> {
    let overlaps_at = |pad: f64| -> Result<bool> {
        Ok(overlap_area_m2(&zone_polygon(zone, lanes, successors, pad)?, opposing) > OVERLAP_AREA_THRESHOLD_M2)
    };
    if !overlaps_at(upper)? {
        return Ok(upper);
    }

    let (mut safe, mut unsafe_) = (0.0, upper);
    for _ in 0..PAD_SEARCH_STEPS {
        let mid = (safe + unsafe_) / 2.0;
        if overlaps_at(mid)? { unsafe_ = mid } else { safe = mid }
    }
    Ok(safe)
}

pub const PAD_SEARCH_STEPS: u32 = 10;

pub const MAX_RESOLUTION_ROUNDS: u32 = 8;

pub fn resolve_overlaps(
    zones: &[E3Detector],
    lanes: &HashMap<&str, &Lane>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    stop_points: &[Coord<f64>],
    polygons: &mut [MultiPolygon<f64>],
) -> Result<()> {
    // Every zone pair that could possibly still need fixing, for the rest
    // of this function's own life — one real `O(n²)` scan across the whole
    // network, never repeated (see [`still_overlapping`]'s own docs for why
    // that's sound: both fixes below only ever shrink a polygon, so a pair
    // that isn't here yet can never become one later).
    let candidates = overlapping_pairs(polygons);
    if candidates.is_empty() {
        return Ok(());
    }

    // Every zone's current padding target, in metres — absent means still
    // at the full [`MIN_DRAWN_LANE_LENGTH_METERS`] default, never having
    // needed shrinking. [`max_safe_pad`] only ever lowers an entry, never
    // raises one back up: a zone that no longer overlaps anything once its
    // padding is smaller stays that size rather than creeping back toward
    // the look-nicer default, since re-growing it could just reopen the
    // same overlap on a later round.
    let mut pad_meters: HashMap<usize, f64> = HashMap::new();

    for _round in 0..MAX_RESOLUTION_ROUNDS {
        let overlaps = still_overlapping(polygons, &candidates);
        if overlaps.is_empty() {
            return Ok(());
        }

        let mut progressed = false;
        for &(i, j) in &overlaps {
            for &(shrink, opposing) in &[(i, j), (j, i)] {
                let current = pad_meters.get(&shrink).copied().unwrap_or(MIN_DRAWN_LANE_LENGTH_METERS);
                if current <= 0.0 {
                    continue;
                }
                let best = max_safe_pad(&zones[shrink], lanes, successors, current, &polygons[opposing])?;
                if best < current {
                    polygons[shrink] = zone_polygon(&zones[shrink], lanes, successors, best)?;
                    pad_meters.insert(shrink, best);
                    progressed = true;
                }
            }
        }

        // Every cut this round needs, computed against `polygons` exactly
        // as this round found it (before any of them are applied — see
        // this function's own docs on why a cut derived from an
        // already-cut shape isn't the same cut any more), collected per
        // zone so one contested by several neighbours at once gets every
        // one of its own overlaps subtracted together rather than one at a
        // time against a moving target. The cut itself is just the real
        // overlap — see this function's own docs for why that's enough on
        // its own, with no direction to compute or extent to size.
        let mut cuts: HashMap<usize, Vec<GeoPolygon<f64>>> = HashMap::new();
        for (i, j) in still_overlapping(polygons, &candidates) {
            let overlap = polygons[i].intersection(&polygons[j]);
            cuts.entry(i).or_default().extend(overlap.0.iter().cloned());
            cuts.entry(j).or_default().extend(overlap.0);
            progressed = true;
        }
        for (zone, overlaps) in cuts {
            // Unioned first, not handed to `difference` as a raw, possibly
            // self-overlapping list: two different neighbours' own overlap
            // regions can themselves touch or overlap (adjacent contested
            // ground at a busy corner), and a subtrahend `difference` never
            // otherwise gets to clean up itself can carry that same
            // near-degenerate geometry straight into the harder, two-input
            // operation.
            let subtrahend = overlaps
                .into_iter()
                .map(|part| MultiPolygon::new(vec![part]))
                .reduce(|acc, part| acc.union(&part))
                .unwrap_or_else(|| MultiPolygon::new(Vec::new()));
            // Cut (and `keep_part_near`-filtered — see its own docs) *per
            // part*, not the zone's own whole `MultiPolygon` in one go: a
            // zone can legitimately already have more than one disjoint
            // part before this round ever touches it (the core plus each
            // extended-ancestor chain, none of which necessarily overlap
            // each other — `zone_polygon`'s own docs), and a neighbour's
            // own overlap this round is essentially never with *every* one
            // of those parts at once. Cutting the whole `MultiPolygon`
            // as one `difference` call already leaves an untouched part
            // untouched (`difference` only ever removes what's actually in
            // the subtrahend) — the bug was applying `keep_part_near`
            // *after* that, over the combined result: confirmed on real
            // Barcelona data, a zone with several genuinely separate,
            // untouched chain parts had every one of them but the single
            // closest-to-the-stop-line discarded the moment *any* one part
            // needed cutting, not just whichever fragment the cut itself
            // stranded. Splitting the loop by part keeps that distinction:
            // an untouched part is pushed straight through, only a part
            // that actually intersects `subtrahend` is cut and filtered.
            let mut new_parts = Vec::with_capacity(polygons[zone].0.len());
            for part in &polygons[zone].0 {
                let part_polygon = MultiPolygon::new(vec![part.clone()]);
                if overlap_area_m2(&part_polygon, &subtrahend) <= OVERLAP_AREA_THRESHOLD_M2 {
                    new_parts.push(part.clone());
                    continue;
                }
                let cut = part_polygon.difference(&subtrahend);
                // Deliberately no `simplify` here, unlike [`zone_polygon`]'s
                // own build: Douglas-Peucker approximates each side of a cut
                // independently, and two zones' own polygons — both cut
                // along the exact same shared boundary this round — can
                // drift centimetres apart from each other once each one's
                // own simplification picks a different nearby point to
                // keep. That reopens a real, sizeable "overlap" (or gap)
                // for the *next* round to rediscover: confirmed on real
                // Barcelona data, simplifying every round left *more*
                // pairs unresolved after 8 rounds, not fewer. Left
                // unsimplified here, a cut zone's own polygon can carry a
                // few more vertices than [`zone_polygon`]'s own output
                // normally would — a minor cost against never disturbing
                // the exact non-overlap this round's own cut just
                // established.
                //
                // Also deliberately no `snap_coords`: that was
                // load-bearing against the former quad-union
                // `buffer_shape`'s own float noise (see its own docs), but
                // with a real stroke offset behind every polygon this
                // function starts from, rounding every vertex onto a
                // shared grid on top of that no longer measurably helps
                // convergence and net loses ground on the
                // self-intersection coherence check (confirmed by trying
                // both ways on real Barcelona data) — [`geo::MapCoords`]'s
                // own lack of a simplicity guarantee (see [`snap_coords`]'s
                // own docs) is a real cost with nothing left here to buy
                // back.
                let cleaned =
                    keep_part_near(drop_slivers(cut), stop_points[zone]);
                new_parts.extend(cleaned.0);
            }
            polygons[zone] = MultiPolygon::new(new_parts);
        }

        if !progressed {
            break; // nothing left that another round could resolve
        }
    }

    for (i, j) in still_overlapping(polygons, &candidates) {
        eprintln!(
            "{ERROR}error:{ERROR:#} zones {:?} and {:?} still overlap after \
             {MAX_RESOLUTION_ROUNDS} resolution rounds — left as-is",
            zones[i].id, zones[j].id,
        );
    }
    Ok(())
}

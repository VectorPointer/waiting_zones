use anstream::eprintln;
use anstyle::{AnsiColor, Style};
use anyhow::{Context, Result};
use geo::{Coord, LineString, MultiPolygon};
use geojson::{Feature, FeatureCollection, Geometry, JsonObject, Position};
use std::{collections::HashMap, path::Path};
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{Lane, Network, Point};
use crate::geojson_output::geometry::{centroid, despike, single_successors, zone_modes, zone_polygon};
use crate::geojson_output::overlaps::{
    distance_to_polygon, drop_grazing_vertices_everywhere, drop_slivers, keep_part_near,
    resolve_overlaps, split_self_intersections,
};
use crate::geojson_output::reprojection::{distance_from_start, point_and_tangent_at, Reprojector, MIN_DRAWN_LANE_LENGTH_METERS};

/// Styles the "warning:" prefix the same way `zone_generator`'s own does —
/// see that module's own docs on why (`cargo`/`rustc` convention, stripped
/// automatically when stderr isn't a terminal).
const WARNING: Style = AnsiColor::Yellow.on_default().bold();

/// Where each of `zone`'s own exit gates puts its stop line — one point
/// per controlled lane, in network coordinates.
pub fn stop_line_points(zone: &E3Detector, lanes: &HashMap<&str, &Lane>) -> Result<Vec<Point>> {
    // A pedestrian zone's own controlled "lane" is a walkingarea, whose
    // `shape` is a closed *outline* of a patch of pavement rather than a
    // centreline to travel along (see
    // `geometry::pedestrian_lane_polygon`). Measuring a distance along it
    // — what `point_and_tangent_at` does, correctly, for every real lane —
    // therefore lands at an arbitrary spot on that outline's perimeter,
    // unrelated to where anyone actually waits: the gate's own position is
    // the walkingarea's `length` (a representative crossing distance,
    // ~2.4m), while its perimeter is several times that, so the two
    // measure different things entirely. Confirmed on real Barcelona data,
    // where this published stop lines up to 3.6m outside their own zone.
    // The centre of the patch is both a real point of it and a truthful
    // answer to "where is someone waiting here".
    let is_pedestrian = !zone.detect_persons.is_empty();

    let mut stop_points = Vec::with_capacity(zone.exits.len());
    for exit in &zone.exits {
        let lane = lanes.get(exit.lane.0.as_str()).copied().with_context(|| {
            format!("zone {:?} references lane {:?}, which isn't in the network", zone.id, exit.lane)
        })?;
        stop_points.push(if is_pedestrian {
            centroid(&lane.shape.0)
        } else {
            let exit_distance = distance_from_start(exit.position, lane.length);
            point_and_tangent_at(&lane.shape, exit_distance).0
        });
    }
    Ok(stop_points)
}

pub fn stop_line_point(zone: &E3Detector, lanes: &HashMap<&str, &Lane>) -> Result<Point> {
    Ok(centroid(&stop_line_points(zone, lanes)?))
}

/// The stop line to publish for `zone`: whichever of its own candidates
/// actually lands on the polygon being published for it.
///
/// The centroid of every controlled lane's own stop point is the right
/// answer whenever those lanes sit side by side, which is nearly always,
/// and it stays the answer here — it's tried first and wins every tie. It
/// stops being one as soon as the lanes *don't*: for a zone merged from
/// two fork siblings landing on different junctions
/// (`zone_generator::merge_fork_sibling_groups`), the midpoint between the
/// two branches falls in the gap between them rather than on either, and
/// `-27642114#8+115826833#0+1218906995_straight` published a stop line
/// 6.5m outside its own zone that way. A client told to expect the stop
/// line at a point the zone it belongs to doesn't contain has been handed
/// a contradiction; falling back to a real lane's own stop point keeps the
/// property meaning what it says.
fn published_stop_line(candidates: &[Point], polygon: &MultiPolygon<f64>) -> Point {
    let centroid = centroid(candidates);
    let distance = |p: &Point| distance_to_polygon(Coord { x: p.x, y: p.y }, polygon);

    std::iter::once(&centroid)
        .chain(candidates)
        .min_by(|a, b| distance(a).total_cmp(&distance(b)))
        .copied()
        .unwrap_or(centroid)
}

pub fn build_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    lane_to_junction: &HashMap<&str, &str>,
    polygon: &MultiPolygon<f64>,
    reproject: &Reprojector,
) -> Result<Feature> {
    let stop_line = reproject.to_lon_lat(published_stop_line(&stop_line_points(zone, lanes)?, polygon))?;

    let ring = |line: &LineString<f64>| -> Result<Vec<Position>> {
        line.coords().map(|c| reproject.to_lon_lat(Point { x: c.x, y: c.y, z: 0.0 }).map(Position::from)).collect()
    };
    // A waiting zone is one contiguous area, and a client's own point-in-
    // polygon check (`resolver::catalogue`) only ever needs a single
    // exterior ring plus holes — GeoJSON's `Polygon`, not `MultiPolygon`'s
    // multiple independent exteriors. `zone_polygon`'s own pipeline is
    // built to always converge on exactly one part per zone (see
    // `to_feature_collection`'s own despike/split/`keep_part_near` pass);
    // more than one surviving here means that guarantee broke somewhere
    // upstream, which a client silently receiving only the first part of a
    // real zone would be a far worse failure mode than an error at
    // generation time.
    let [part] = polygon.0.as_slice() else {
        anyhow::bail!(
            "zone {:?} resolved to {} polygon part(s), expected exactly 1 — geojson_output only emits a Polygon, not a MultiPolygon",
            zone.id,
            polygon.0.len()
        );
    };
    // `part.interiors()` is always empty here: `zone_polygon`'s own
    // `drop_interior_rings` strips every interior ring before this
    // function ever sees the polygon (see that function's own docs on why
    // a waiting zone's own real ground never legitimately has a hole).
    // Debug-only, not a silent truncation: this asserts the guarantee
    // holds rather than quietly re-dropping a hole a future change to
    // `zone_polygon` reintroduces.
    debug_assert!(part.interiors().is_empty(), "zone {:?} has an interior ring `zone_polygon` should have dropped", zone.id);
    let rings = vec![ring(part.exterior())?];

    // The junction the zone's own controlled lane (its exit — always the
    // group's own controlled lane, never guaranteed of an entry, extended
    // or otherwise; see `zone_generator`'s own docs) feeds into. Not a
    // field on `E3Detector` itself: `E3Detector` is a literal mirror of
    // SUMO's own `e3Detector` schema (see `zone_generator`'s module docs
    // on why no project-specific type sits between the two), which has no
    // concept of "intersection" at all — this is derived straight from the
    // network here instead, the same way `territory::zones::Zone::edge` is
    // derived from a zone's own exits rather than carried as a field.
    let intersection_id =
        zone.exits.first().and_then(|exit| lane_to_junction.get(exit.lane.0.as_str()).copied());

    let mut properties = JsonObject::new();
    properties.insert("waiting_zone_id".to_string(), zone.id.0.clone().into());
    if let Some(intersection_id) = intersection_id {
        properties.insert("intersection_id".to_string(), intersection_id.into());
    }
    properties.insert("stop_line".to_string(), serde_json::json!(stop_line));
    properties.insert("modes".to_string(), serde_json::json!(zone_modes(zone, lanes)));

    let mut feature = Feature::from(Geometry::new_polygon(rings));
    feature.properties = Some(properties);
    Ok(feature)
}

#[cfg(test)]
pub fn zone_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    lane_to_junction: &HashMap<&str, &str>,
    successors: &HashMap<&str, (&str, Option<&str>)>,
    reproject: &Reprojector,
    pad_meters: f64,
) -> Result<Feature> {
    build_feature(zone, lanes, lane_to_junction, &zone_polygon(zone, lanes, successors, pad_meters)?, reproject)
}

pub fn to_feature_collection(network: &Network, zones: &[E3Detector]) -> Result<FeatureCollection> {
    let reproject = Reprojector::new(&network.location)?;
    let lanes: HashMap<&str, &Lane> = network
        .edges
        .iter()
        .flat_map(|edge| &edge.lanes)
        .map(|lane| (lane.id.0.as_str(), lane))
        .collect();
    // Which junction lists a lane among its own `incLanes` — used to report
    // a zone's `intersection_id` (see `build_feature`'s own docs on why
    // this is derived here rather than stored on `E3Detector`). Not
    // `edge.to`: real `.net.xml` output never sets `from`/`to` on a
    // walkingarea edge (it's already "at" a junction, not a stretch of
    // road connecting two of them) — confirmed on real Barcelona data,
    // every pedestrian zone's own exit lane resolving to no junction at
    // all through that path — but SUMO still lists a walkingarea lane
    // among the junction's own `incLanes`, right alongside the vehicle
    // lanes it shares the junction with (see `zone_generator`'s own module
    // docs), so this covers both zone kinds uniformly.
    let lane_to_junction: HashMap<&str, &str> = network
        .junctions
        .iter()
        .flat_map(|junction| junction.incoming_lanes.iter().map(move |lane| (lane.0.as_str(), junction.id.0.as_str())))
        .collect();
    let successors = single_successors(network);

    let mut polygons = zones
        .iter()
        .map(|zone| zone_polygon(zone, &lanes, &successors, MIN_DRAWN_LANE_LENGTH_METERS))
        .collect::<Result<Vec<_>>>()?;
    // `resolve_overlaps`'s own reference point per zone — see
    // `keep_part_near`'s own docs for what it's for.
    let stop_points = zones
        .iter()
        .map(|zone| stop_line_point(zone, &lanes).map(|p| Coord { x: p.x, y: p.y }))
        .collect::<Result<Vec<_>>>()?;

    resolve_overlaps(zones, &lanes, &successors, &stop_points, &mut polygons)?;
    // `resolve_overlaps`'s own `difference` cuts leave the same kind of
    // spike/near-duplicate seam `zone_polygon`'s own `union` does (see
    // `despike`'s own docs) — confirmed on real Barcelona data
    // (`207322883#0_straight`, `207322888#0_straight`,
    // `-37442552#5_straight`, `47373956#0_straight`, all clean before
    // `resolve_overlaps` runs and carrying the defect only afterward).
    // Despiking *inside* `resolve_overlaps`'s own per-round loop was tried
    // first and made things worse: nudging a cut polygon's boundary
    // mid-loop feeds back into the next round's own overlap check, the
    // exact round-to-round drift this function's own docs already warn
    // `simplify`/`snap_coords` cause there. Run once here instead, on the
    // finished, final polygons no further round will ever re-examine —
    // there's nothing left for a one-shot cleanup at this point to
    // destabilize.
    // Skipped for a pedestrian zone's own polygon, same as `zone_polygon`'s
    // own `simplify` -- confirmed on real Barcelona data
    // (`5588597076_w0_straight_ped`, `6119951203_w0_straight_ped`): a
    // walkingarea's own compact outline has its real corners close enough
    // together that repositioning even one nearby vertex measurably
    // sharpens which chord spans a real corner, exactly the effect
    // `zone_polygon`'s own docs already describe for `simplify` there.
    for ((zone, polygon), &stop_point) in zones.iter().zip(&mut polygons).zip(&stop_points) {
        if zone.detect_persons.is_empty() {
            let taken = std::mem::replace(polygon, MultiPolygon::new(Vec::new()));
            // Despiked, and then checked, *per original part* -- not by
            // reducing the whole `MultiPolygon` down to one part right
            // here, which would conflate this loop's own narrow concern
            // (a `despike`-introduced self-crossing's own far half) with
            // the separate, zone-wide "keep only the part nearest the
            // stop line" reduction the caller already applies once, after
            // this loop, to every zone regardless of kind (see that
            // reduction's own docs for why it belongs there and not here).
            // `despike` removes a vertex pair by local proximity alone,
            // with no check that doing so leaves the rest of *that part's
            // own* ring simple — confirmed on real Barcelona data
            // (`207322888#0_straight`): the raw cut it ran on was already
            // simple, but the shortcut `despike` took past a spike it
            // removed crossed back over a real, untouched edge elsewhere
            // in the same ring. `split_self_intersections` (already how
            // `resolve_overlaps`'s own cut handles a self-crossing result)
            // plus `keep_part_near` fixes that *within* a part that
            // actually needed it, leaving every other, already-simple
            // part untouched at this stage.
            let mut new_parts = Vec::with_capacity(taken.0.len());
            for part in taken.0 {
                let despiked = despike(MultiPolygon::new(vec![part]));
                let split = split_self_intersections(despiked);
                if split.0.len() > 1 {
                    let kept = keep_part_near(split, stop_point);
                    new_parts.extend(kept.0);
                } else {
                    new_parts.extend(split.0);
                }
            }
            *polygon = MultiPolygon::new(new_parts);
        }
    }
    // A waiting zone is one contiguous area anchored at its own stop line,
    // never several disconnected islands — whether the extra parts come
    // from a cut against a neighbouring zone, or (`merged_core_polygon`'s
    // own non-contiguous-lane-group case) from the zone's own shape never
    // touching itself to begin with: either way, the zone ends where the
    // gap is, rather than continuing on the far side of it.
    // `keep_part_near` picks exactly the part a client walking backward
    // from the stop line would actually reach (the one containing it, or
    // else the largest), applied here regardless of zone kind or of
    // whether the extra part predates `resolve_overlaps`.
    for (polygon, &stop_point) in polygons.iter_mut().zip(&stop_points) {
        if polygon.0.len() > 1 {
            *polygon = keep_part_near(std::mem::replace(polygon, MultiPolygon::new(Vec::new())), stop_point);
        }
    }
    // Last geometric step before reprojection, and the only one that runs
    // for *every* zone regardless of kind: drop any vertex left grazing an
    // edge it's ring-adjacent to. Every other pass above works in this
    // crate's own local metre coordinates and leaves a ring that is simple
    // *there*; `drop_grazing_vertices_everywhere`'s own docs cover why a
    // residual nanometre-scale coincidence is nonetheless the one thing
    // `Reprojector::to_lon_lat` can — and, measured across the whole real
    // Barcelona network, reliably does — turn back into a real crossing in
    // the lon/lat output a client actually consumes. Running it here, on
    // the finished polygons no later round will re-examine, is the same
    // reasoning the despike pass above is placed here rather than inside
    // `resolve_overlaps`' own loop.
    // A zone's ring has to be simple in the lon/lat a client actually
    // consumes, whatever kind of zone it is. The despike pass above is
    // deliberately vehicle-only (it repositions vertices, which measurably
    // sharpens a walkingarea's own tightly-spaced real corners), but
    // neither step here moves a vertex anywhere: one *drops* a vertex whose
    // own contribution is sub-square-millimetre, the other cuts a ring at a
    // point it already revisits. Both are therefore safe to run for a
    // pedestrian zone too — and needed there, since nothing else did:
    // `5588593504_w0_straight_ped`'s own outline crosses itself by a real
    // 13.5mm, well past anything `drop_grazing_vertices_everywhere`'s own
    // margin is meant to see, and `netconvert` drew it that way.
    for (polygon, &stop_point) in polygons.iter_mut().zip(&stop_points) {
        let taken = std::mem::replace(polygon, MultiPolygon::new(Vec::new()));
        let cleaned = drop_slivers(split_self_intersections(drop_grazing_vertices_everywhere(taken)));
        // `drop_slivers` unconditionally, not only inside this branch: a
        // zone with exactly one surviving part never used to be checked
        // against `MIN_KEPT_PART_AREA_M2` at all, since the sliver filter
        // only ran here when there was more than one part to choose
        // between. Confirmed on real Eixample data, 15 zones whose own
        // sole remaining part had collapsed to a near-zero-area triangle
        // (as small as 0.000061m²) shipped anyway — `keep_part_near`'s own
        // "more than one part" check never even looked at whether the one
        // part it *did* have was worth keeping.
        *polygon = if cleaned.0.len() > 1 { keep_part_near(cleaned, stop_point) } else { cleaned };
    }

    let features = zones
        .iter()
        .zip(&polygons)
        .filter_map(|(zone, polygon)| {
            if polygon.0.is_empty() {
                // Cut away entirely by overlap resolution — no ground left
                // for this zone to claim, so there's nothing to draw.
                // Reported, not silently dropped: it's rare enough on real
                // data that it's worth a human noticing, but not a reason
                // to fail the whole run the way `build_feature` finding
                // *more* than one surviving part would be (an actual
                // pipeline bug, not an empty-but-valid outcome).
                eprintln!("{WARNING}warning:{WARNING:#} zone {:?} was fully cut away by overlap resolution; omitted from the GeoJSON output", zone.id);
                return None;
            }
            Some(build_feature(zone, &lanes, &lane_to_junction, polygon, &reproject))
        })
        .collect::<Result<Vec<_>>>()?;

    Ok(FeatureCollection {
        bbox: None,
        features,
        foreign_members: None,
    })
}

/// `path`, with `suffix` inserted right before the extension —
/// `zones.geojson` with `"vehicles"` becomes `zones.vehicles.geojson`. Falls
/// back to appending `.{suffix}` when `path` has no extension to insert
/// before (e.g. a bare `"zones"`), so the two output paths this module's
/// own `write` derives are still always distinct from each other and from
/// `path` itself.
fn suffixed(path: &Path, suffix: &str) -> std::path::PathBuf {
    match path.extension().and_then(|ext| ext.to_str()) {
        Some(extension) => path.with_extension(format!("{suffix}.{extension}")),
        None => path.with_extension(suffix),
    }
}

fn write_collection(path: &Path, collection: &FeatureCollection) -> Result<()> {
    let json =
        serde_json::to_string_pretty(collection).context("could not serialize waiting zones as GeoJSON")?;
    std::fs::write(path, json).with_context(|| format!("could not write output file: {path:?}"))
}

/// Writes `zones` out as 2 separate GeoJSON `FeatureCollection`s at paths
/// derived from `path` — `cdn.md`'s catalogue is split by mode so a client
/// resolving, say, a vehicle position never has to fetch (or hold) the
/// pedestrian half of a territory's own zones, and vice versa. Always
/// writes both files, even when one side has no zones at all (a
/// vehicle-only or pedestrian-only network): a fixed pair of endpoints a
/// client can always fetch beats one that sometimes doesn't exist.
pub fn write(path: &Path, network: &Network, zones: &[E3Detector]) -> Result<()> {
    let (pedestrian, vehicle): (Vec<&E3Detector>, Vec<&E3Detector>) =
        zones.iter().partition(|zone| !zone.detect_persons.is_empty());

    let pedestrian_zones: Vec<E3Detector> = pedestrian.into_iter().cloned().collect();
    let vehicle_zones: Vec<E3Detector> = vehicle.into_iter().cloned().collect();

    write_collection(&suffixed(path, "pedestrians"), &to_feature_collection(network, &pedestrian_zones)?)?;
    write_collection(&suffixed(path, "vehicles"), &to_feature_collection(network, &vehicle_zones)?)?;
    Ok(())
}

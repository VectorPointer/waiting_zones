use anstream::eprintln;
use anstyle::{AnsiColor, Style};
use anyhow::{Context, Result};
use geo::{Coord, LineString, MultiPolygon};
use geojson::{Feature, FeatureCollection, Geometry, JsonObject, Position};
use osm_crosswalks::OsmCrosswalk;
use std::{collections::HashMap, path::Path};
use sumo_types::additional::domain::E3Detector;
use sumo_types::domain::{Lane, Network, Point};
use crate::geojson_output::crosswalks::match_crosswalks;
use crate::geojson_output::geometry::{centroid, despike, lane_links, rectangle_if_close, zone_modes, zone_polygon};
use crate::geojson_output::overlaps::{
    distance_to_polygon, drop_grazing_vertices_everywhere, drop_interior_rings, drop_slivers,
    keep_part_near, overlap_area_m2, relax_needle_vertices_everywhere, resolve_overlaps,
    resolve_pedestrian_overlaps, resolve_vehicle_overlaps, split_self_intersections, unsplit_crossings,
    OVERLAP_AREA_THRESHOLD_M2,
};
use crate::geojson_output::reprojection::{distance_from_start, point_and_tangent_at, Reprojector, MIN_DRAWN_LANE_LENGTH_METERS};

/// Styles the "warning:" prefix the same way `zone_generator`'s own does —
/// see that module's own docs on why (`cargo`/`rustc` convention, stripped
/// automatically when stderr isn't a terminal).
const WARNING: Style = AnsiColor::Yellow.on_default().bold();

/// How far a zone's own published stop line may sit outside its finished
/// polygon before the final needle-relaxation pass must leave the shape
/// alone, in metres. The same value `tests`' own
/// `STOP_LINE_TOLERANCE_METERS` treats as the real limit (a car stopped at
/// the line is inside the zone at under a metre even with a few metres of
/// GPS error), restated here because the production pass — not the test —
/// is what has to make the decision. Relaxation is cosmetic; coverage of
/// the line the zone exists to detect is not, so this is the tie-breaker.
const MAX_PUBLISHED_STOP_LINE_DRIFT_METERS: f64 = 1.0;

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
    let stop_line = published_stop_line(&stop_line_points(zone, lanes)?, polygon);
    build_feature_at(zone, lanes, lane_to_junction, polygon, reproject, stop_line)
}

/// [`build_feature`], with the stop line to publish already decided.
pub fn build_feature_at(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    lane_to_junction: &HashMap<&str, &str>,
    polygon: &MultiPolygon<f64>,
    reproject: &Reprojector,
    stop_line: Point,
) -> Result<Feature> {
    // The junction the zone's own controlled lane (its exit) feeds into —
    // derived from the network, since `E3Detector` mirrors SUMO's own
    // `e3Detector` schema, which has no concept of "intersection".
    let intersection_id =
        zone.exits.first().and_then(|exit| lane_to_junction.get(exit.lane.0.as_str()).copied());
    let modes = serde_json::json!(zone_modes(zone, lanes));
    feature_from_parts(&zone.id.0, intersection_id, modes, polygon, reproject, stop_line)
}

/// A zone's GeoJSON feature from its parts: id, junction, modes, polygon
/// (local metres) and stop line — for a zone with or without an
/// `E3Detector` behind it.
pub fn feature_from_parts(
    id: &str,
    intersection_id: Option<&str>,
    modes: serde_json::Value,
    polygon: &MultiPolygon<f64>,
    reproject: &Reprojector,
    stop_line: Point,
) -> Result<Feature> {
    let stop_line = reproject.to_lon_lat(stop_line)?;
    let ring = |line: &LineString<f64>| -> Result<Vec<Position>> {
        line.coords().map(|c| reproject.to_lon_lat(Point { x: c.x, y: c.y, z: 0.0 }).map(Position::from)).collect()
    };
    // A zone with more than one part is emitted as a GeoJSON `MultiPolygon`,
    // which the client's own point-in-polygon check, the panel and this
    // crate's own fixtures all already accept.
    if polygon.0.is_empty() {
        anyhow::bail!("zone {id:?} has no polygon part to emit");
    }
    let polygons: Vec<Vec<Vec<Position>>> = polygon
        .0
        .iter()
        .map(|part| {
            debug_assert!(part.interiors().is_empty(), "zone {id:?} has an interior ring that should have been dropped");
            ring(part.exterior()).map(|ring| vec![ring])
        })
        .collect::<Result<_>>()?;

    let mut properties = JsonObject::new();
    properties.insert("waiting_zone_id".to_string(), id.into());
    if let Some(intersection_id) = intersection_id {
        properties.insert("intersection_id".to_string(), intersection_id.into());
    }
    properties.insert("stop_line".to_string(), serde_json::json!(stop_line));
    properties.insert("modes".to_string(), modes);

    let geometry = if polygons.len() == 1 {
        Geometry::new_polygon(polygons.into_iter().next().expect("checked non-empty above"))
    } else {
        Geometry::new_multi_polygon(polygons)
    };
    let mut feature = Feature::from(geometry);
    feature.properties = Some(properties);
    Ok(feature)
}

#[cfg(test)]
pub fn zone_feature(
    zone: &E3Detector,
    lanes: &HashMap<&str, &Lane>,
    lane_to_junction: &HashMap<&str, &str>,
    links: &crate::geojson_output::geometry::LaneLinks<'_>,
    reproject: &Reprojector,
    pad_meters: f64,
) -> Result<Feature> {
    build_feature(zone, lanes, lane_to_junction, &zone_polygon(zone, lanes, links, pad_meters)?, reproject)
}

pub fn to_feature_collection(network: &Network, zones: &[E3Detector]) -> Result<FeatureCollection> {
    to_feature_collection_with_crosswalks(network, zones, &[])
}

/// [`to_feature_collection`], additionally widening a pedestrian zone's own
/// crossing to fully contain whichever real OSM `crosswalks` entry matches
/// it (see [`crate::geojson_output::crosswalks::match_crosswalks`]). An
/// empty `crosswalks` behaves exactly like `to_feature_collection` — no
/// match is ever made, no footprint ever grows.
pub fn to_feature_collection_with_crosswalks(
    network: &Network,
    zones: &[E3Detector],
    crosswalks: &[OsmCrosswalk],
) -> Result<FeatureCollection> {
    // Pedestrian zones are built on their own (see `pedestrian`'s own docs):
    // the crosswalk plus a sidewalk offset, nothing of the vehicle pipeline.
    let is_pedestrian = |zone: &E3Detector| !zone.detect_persons.is_empty();
    if zones.iter().any(is_pedestrian) {
        let (pedestrian, vehicle): (Vec<E3Detector>, Vec<E3Detector>) =
            zones.iter().cloned().partition(|zone| is_pedestrian(zone));
        let mut features = crate::geojson_output::pedestrian::feature_collection(network, &pedestrian, crosswalks)?.features;
        if !vehicle.is_empty() {
            features.extend(vehicle_feature_collection(network, &vehicle)?.features);
        }
        // Back in the zones' own order.
        let position = |feature: &Feature| {
            let id = feature.property("waiting_zone_id").and_then(|v| v.as_str()).unwrap_or_default();
            zones.iter().position(|zone| zone.id.0 == id).unwrap_or(usize::MAX)
        };
        features.sort_by_key(position);
        return Ok(FeatureCollection { bbox: None, features, foreign_members: None });
    }
    vehicle_feature_collection(network, zones)
}

/// Which junction lists a lane among its own `incLanes` — used to report
/// a zone's `intersection_id` (see `build_feature`'s own docs on why this
/// is derived here rather than stored on `E3Detector`). Not `edge.to`:
/// real `.net.xml` output never sets `from`/`to` on a walkingarea edge
/// (it's already "at" a junction, not a stretch of road connecting two of
/// them), but SUMO still lists a walkingarea lane among the junction's own
/// `incLanes`, right alongside the vehicle lanes it shares the junction
/// with, so this covers both zone kinds uniformly.
pub fn lane_to_junction(network: &Network) -> HashMap<&str, &str> {
    network
        .junctions
        .iter()
        .flat_map(|junction| junction.incoming_lanes.iter().map(move |lane| (lane.0.as_str(), junction.id.0.as_str())))
        .collect()
}

fn vehicle_feature_collection(network: &Network, zones: &[E3Detector]) -> Result<FeatureCollection> {
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
    let lane_to_junction = lane_to_junction(network);
    let links = lane_links(network);

    let mut polygons = zones
        .iter()
        .map(|zone| zone_polygon(zone, &lanes, &links, MIN_DRAWN_LANE_LENGTH_METERS))
        .collect::<Result<Vec<_>>>()?;
    // `resolve_overlaps`'s own reference point per zone — see
    // `keep_part_near`'s own docs for what it's for.
    let stop_points = zones
        .iter()
        .map(|zone| stop_line_point(zone, &lanes).map(|p| Coord { x: p.x, y: p.y }))
        .collect::<Result<Vec<_>>>()?;
    // Every zone's own stop-line candidates, kept for the final
    // needle-relaxation coverage guard — see the loop's own comment.
    let stop_line_candidates: Vec<Vec<Point>> = zones
        .iter()
        .map(|zone| stop_line_points(zone, &lanes))
        .collect::<Result<Vec<_>>>()?;


    resolve_overlaps(zones, &lanes, &links, &stop_points, &mut polygons)?;
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
    // A *vehicle* waiting zone is one contiguous area anchored at its own
    // stop line, never several disconnected islands — whether the extra
    // parts come from a cut against a neighbouring zone, or
    // (`merged_core_polygon`'s own non-contiguous-lane-group case) from the
    // zone's own shape never touching itself to begin with: either way, the
    // zone ends where the gap is, rather than continuing on the far side of
    // it. `keep_part_near` picks exactly the part a client walking backward
    // from the stop line would actually reach (the one containing it, or
    // else the largest).
    //
    // A pedestrian zone is deliberately exempt: it spans both banks of one
    // crossing, two genuinely disjoint rectangles with the road between
    // them, and both are the zone (see `zone_generator::pedestrian_zones`).
    // Collapsing to the near bank here would silently drop the far one.
    for (index, (polygon, &stop_point)) in polygons.iter_mut().zip(&stop_points).enumerate() {
        if polygon.0.len() > 1 && zones[index].detect_persons.is_empty() {
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
    // consumes, whatever kind of zone it is. Most of what runs here never
    // repositions a vertex — `drop_grazing_vertices_everywhere` *drops* one
    // whose own contribution is sub-square-millimetre and
    // `split_self_intersections` cuts a ring at a point it already revisits
    // — so both are safe for a pedestrian zone too, and needed there, since
    // nothing else did: `5588593504_w0_straight_ped`'s own outline crosses
    // itself by a real 13.5mm, well past anything
    // `drop_grazing_vertices_everywhere`'s own margin is meant to see, and
    // `netconvert` drew it that way.
    //
    // `relax_needle_vertices_everywhere` is the one pass that *does* move a
    // vertex, and it is held to a stricter standard for it — see the loop's
    // own comment on why its result is only ever accepted when it keeps the
    // zone's own published stop line covered.
    for (index, (polygon, &stop_point)) in polygons.iter_mut().zip(&stop_points).enumerate() {
        let taken = std::mem::replace(polygon, MultiPolygon::new(Vec::new()));

        // `drop_slivers` unconditionally, not only inside the "more than one
        // part" branch: a zone with exactly one surviving part never used to
        // be checked against `MIN_KEPT_PART_AREA_M2` at all, since the
        // sliver filter only ran here when there was more than one part to
        // choose between. Confirmed on real Eixample data, 15 zones whose
        // own sole remaining part had collapsed to a near-zero-area triangle
        // (as small as 0.000061m²) shipped anyway — `keep_part_near`'s own
        // "more than one part" check never even looked at whether the one
        // part it *did* have was worth keeping.
        let is_pedestrian = !zones[index].detect_persons.is_empty();
        let finish = |polygon: MultiPolygon<f64>| {
            let cleaned = drop_interior_rings(drop_slivers(split_self_intersections(
                drop_grazing_vertices_everywhere(polygon),
            )));
            if cleaned.0.len() > 1 && !is_pedestrian {
                keep_part_near(cleaned, stop_point)
            } else {
                cleaned
            }
        };

        let cleaned = finish(taken);
        // Smooth each remaining spike by moving its tip rather than deleting
        // it (see `relax_needle_vertices_everywhere`'s own docs), then keep
        // the result only when it leaves the *published* stop line's own
        // coverage exactly as the cleaned shape had it. A spike's tip is
        // sometimes precisely the ground a walkingarea's centroid stop line
        // sits on; a zone that stops covering the line it exists to detect
        // is a worse defect than any cosmetic spike, so coverage wins. Both
        // sides are fully finished (including `keep_part_near`) before the
        // comparison, because it is the final polygon that has to cover the
        // line, and relaxing can change which part survives.
        let relaxed = finish(relax_needle_vertices_everywhere(cleaned.clone()));
        // How far the zone's own *published* stop line sits outside its
        // polygon — `0.0` for the overwhelming majority, and a small,
        // already-known debt only where `resolve_overlaps` had to cut the
        // zone back past its own line (see
        // `tests::ZONES_CUT_BACK_PAST_THEIR_OWN_STOP_LINE`).
        let stop_line_drift = |polygon: &MultiPolygon<f64>| {
            let stop = published_stop_line(&stop_line_candidates[index], polygon);
            distance_to_polygon(Coord { x: stop.x, y: stop.y }, polygon)
        };
        let cleaned_drift = stop_line_drift(&cleaned);
        let relaxed_drift = stop_line_drift(&relaxed);
        // Cosmetic win only when it doesn't cost stop-line coverage: the
        // relaxed shape is used only if the cleaned one already covered its
        // line (never *fixing* a known cut-back zone, which would just make
        // that tracked debt lie) and relaxing does not move the line
        // further out. Anything else keeps the cleaned shape untouched.
        let use_relaxed = cleaned_drift <= MAX_PUBLISHED_STOP_LINE_DRIFT_METERS
            && relaxed_drift <= cleaned_drift;
        *polygon = if use_relaxed { relaxed } else { cleaned };
    }

    // Every pedestrian-pedestrian pair whose shared ground is a clean
    // quadrilateral — two crossings meeting at one corner, the common case
    // — gets partitioned along its own diagonal rather than left as a
    // double-claimed patch; see `resolve_pedestrian_overlaps`'s own docs
    // for why that, not `resolve_overlaps`'s own cut-from-both fix, is the
    // right one for two pedestrians sharing ground. Run on the zones'
    // finished, per-zone-cleaned polygons (the loop just above), so the
    // quad it looks for is the real, settled shape, not a still-noisy
    // intermediate one. A pair whose shared ground isn't a clean quad is
    // left exactly as before, still reported as an overlap.
    resolve_pedestrian_overlaps(zones, &stop_points, &mut polygons);
    // Every real crossing lane wholly inside its own structural owner's
    // zone, whatever split it between two in the first place — see that
    // function's own docs for why this has to run last and can't just
    // trust the bisector cut just above to have gotten it right. Widened,
    // where a real OSM crosswalk was matched to it, to also fully contain
    // that crosswalk (see `crosswalks::match_crosswalks`'s own docs) —
    // empty when `crosswalks` is, so this is a no-op for every existing
    // caller that doesn't pass any.
    let osm_matches = match_crosswalks(network, zones, &reproject, &[]);
    let osm_footprints: HashMap<&str, MultiPolygon<f64>> =
        osm_matches.footprints.iter().map(|(lane_id, polygon)| (lane_id.as_str(), polygon.clone())).collect();
    unsplit_crossings(zones, &lanes, &stop_points, &mut polygons, &osm_footprints);

    // The simplest possible footprint for the common case, tried last and
    // only once, on every zone's own finished, cross-zone-resolved polygon
    // — see `rectangle_if_close`'s own docs for why earlier (feeding a
    // rectangle into `resolve_overlaps`'s own iterative cuts) is actively
    // unsafe. A rectangle always contains every point of the shape it
    // replaces, so accepting one can never lose stop-line coverage the loop
    // above already settled — but it *can* newly overlap a neighbour
    // `resolve_overlaps` had already cut this zone clear of (confirmed on
    // real Barcelona data: swapping unconditionally reintroduced 17
    // same-mode pairs the resolver had left disjoint). Kept in the zones'
    // own iteration order rather than computed independently for every zone
    // at once, so a later zone's own overlap check sees an earlier zone's
    // already-accepted rectangle, not its pre-swap shape — accepting both
    // could reintroduce exactly the overlap checking each one individually
    // against the original shapes alone would have missed. Skipped for a
    // pedestrian zone, same as `zone_polygon`'s own `simplify`: its shape is
    // already the union of two rectangular banks and a stripe, not one
    // footprint to square off further. Any pedestrian-pedestrian overlap
    // `resolve_pedestrian_overlaps` (just above) couldn't partition is left
    // deliberately alone, not squared off out from under it — see that
    // function's own docs.
    for index in 0..polygons.len() {
        if !zones[index].detect_persons.is_empty() {
            continue;
        }
        let candidate = rectangle_if_close(polygons[index].clone());
        let fits = polygons
            .iter()
            .enumerate()
            .filter(|&(other, _)| other != index && zones[other].detect_persons.is_empty())
            .all(|(_, other_polygon)| overlap_area_m2(&candidate, other_polygon) <= OVERLAP_AREA_THRESHOLD_M2);
        if fits {
            polygons[index] = candidate;
        }
    }
    // Run only now, after squaring: `rectangle_if_close`'s own candidate can
    // legitimately grow a zone back out past a cut `resolve_vehicle_overlaps`
    // made earlier (its own `fits` check above only ever compares against
    // the *other* zone's polygon at that point in the loop, not against a
    // still-to-come sibling's own later squaring) — running this pass
    // before squaring let the very overlap it just resolved reopen once
    // squaring ran afterward. Moved here, it sees the real, final shapes
    // squaring settles on, so nothing downstream can undo its own cut.
    resolve_vehicle_overlaps(zones, &stop_points, &mut polygons);

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
pub fn write(path: &Path, network: &Network, zones: &[E3Detector], crosswalks: &[OsmCrosswalk]) -> Result<()> {
    let (pedestrian, vehicle): (Vec<&E3Detector>, Vec<&E3Detector>) =
        zones.iter().partition(|zone| !zone.detect_persons.is_empty());

    let pedestrian_zones: Vec<E3Detector> = pedestrian.into_iter().cloned().collect();
    let vehicle_zones: Vec<E3Detector> = vehicle.into_iter().cloned().collect();

    write_collection(
        &suffixed(path, "pedestrians"),
        &to_feature_collection_with_crosswalks(network, &pedestrian_zones, crosswalks)?,
    )?;
    // A vehicle zone never references a crossing lane in its own entries
    // (`unsplit_crossings` only ever looks at pedestrian zones' own
    // `detect_persons`), so there is nothing for `crosswalks` to widen here
    // — passed along anyway rather than duplicating `to_feature_collection`
    // just to omit an argument that's already a no-op for this half.
    write_collection(
        &suffixed(path, "vehicles"),
        &to_feature_collection_with_crosswalks(network, &vehicle_zones, crosswalks)?,
    )?;
    Ok(())
}

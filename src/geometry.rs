//! Polyline and polygon helpers, all in local metres (`[x, y]`).

use geo::{Coord, LineString, MultiPolygon, Polygon};
use i_overlay::mesh::stroke::offset::StrokeOffset;
use i_overlay::mesh::style::{LineCap, LineJoin, StrokeStyle};

pub type Pt = [f64; 2];

pub fn dist(a: Pt, b: Pt) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

pub fn length(points: &[Pt]) -> f64 {
    points.windows(2).map(|w| dist(w[0], w[1])).sum()
}

fn lerp(a: Pt, b: Pt, t: f64) -> Pt {
    [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]
}

/// The part of `points` between `from` and `to` metres along it (clamped
/// to the polyline's own length).
pub fn sub_polyline(points: &[Pt], from: f64, to: f64) -> Vec<Pt> {
    let (from, to) = (from.max(0.0), to.min(length(points)));
    let mut out = Vec::new();
    if to <= from {
        return out;
    }
    let mut travelled = 0.0;
    for w in points.windows(2) {
        let segment = dist(w[0], w[1]);
        let (start, end) = (travelled, travelled + segment);
        travelled = end;
        if segment <= 0.0 || end < from || start > to {
            continue;
        }
        let a = if from > start {
            lerp(w[0], w[1], (from - start) / segment)
        } else {
            w[0]
        };
        let b = if to < end {
            lerp(w[0], w[1], (to - start) / segment)
        } else {
            w[1]
        };
        if out.last() != Some(&a) {
            out.push(a);
        }
        out.push(b);
    }
    out
}

/// Unit direction of travel at the end of `points`.
pub fn end_direction(points: &[Pt]) -> Option<Pt> {
    let n = points.len();
    let (a, b) = (points.get(n.checked_sub(2)?)?, points.get(n - 1)?);
    unit([b[0] - a[0], b[1] - a[1]])
}

pub fn unit(v: Pt) -> Option<Pt> {
    let n = v[0].hypot(v[1]);
    (n > 1e-9).then(|| [v[0] / n, v[1] / n])
}

/// The right-hand normal of direction `d` (right of travel).
pub fn right_of(d: Pt) -> Pt {
    [d[1], -d[0]]
}

/// `points` shifted `offset` metres to the right of travel (negative: left),
/// with each interior vertex moved along the bisector of its two segments'
/// own normals so a gentle bend stays a bend.
pub fn offset(points: &[Pt], offset: f64) -> Vec<Pt> {
    offset_each(points, &vec![offset; points.len()])
}

/// `points` with each vertex shifted its own `offsets[i]` metres to the
/// right of travel, along the bisector of its two segments' normals.
pub fn offset_each(points: &[Pt], offsets: &[f64]) -> Vec<Pt> {
    let normals: Vec<Pt> = points
        .windows(2)
        .map(|w| {
            unit([w[1][0] - w[0][0], w[1][1] - w[0][1]])
                .map(right_of)
                .unwrap_or([0.0, 0.0])
        })
        .collect();
    if normals.is_empty() {
        return points.to_vec();
    }
    points
        .iter()
        .zip(offsets)
        .enumerate()
        .map(|(i, (p, &offset))| {
            let n = match (
                i.checked_sub(1).and_then(|j| normals.get(j)),
                normals.get(i),
            ) {
                (Some(a), Some(b)) => {
                    let sum = [a[0] + b[0], a[1] + b[1]];
                    let cos_half = unit(sum)
                        .map(|s| s[0] * b[0] + s[1] * b[1])
                        .unwrap_or(1.0)
                        .max(0.5);
                    unit(sum)
                        .map(|s| [s[0] / cos_half, s[1] / cos_half])
                        .unwrap_or(*b)
                }
                (Some(a), None) => *a,
                (None, Some(b)) => *b,
                (None, None) => [0.0, 0.0],
            };
            [p[0] + n[0] * offset, p[1] + n[1] * offset]
        })
        .collect()
}

/// `points` cut into runs at every vertex turning sharper than `degrees`,
/// each run keeping that vertex as its end or start.
pub fn split_at_sharp_turns(points: &[Pt], degrees: f64) -> Vec<Vec<Pt>> {
    let cos_limit = degrees.to_radians().cos();
    let mut runs = vec![Vec::new()];
    for (i, &p) in points.iter().enumerate() {
        runs.last_mut().expect("one run").push(p);
        if i == 0 || i + 1 == points.len() {
            continue;
        }
        let (a, c) = (points[i - 1], points[i + 1]);
        if let (Some(u), Some(v)) = (
            unit([p[0] - a[0], p[1] - a[1]]),
            unit([c[0] - p[0], c[1] - p[1]]),
        ) && u[0] * v[0] + u[1] * v[1] < cos_limit
        {
            runs.push(vec![p]);
        }
    }
    runs.retain(|run| run.len() >= 2);
    runs
}

/// `points` buffered into a band `width` metres wide, flat at both ends.
pub fn stroke(points: &[Pt], width: f64) -> MultiPolygon<f64> {
    if points.len() < 2 || width <= 0.0 {
        return MultiPolygon::new(Vec::new());
    }
    let style = StrokeStyle::new(width)
        // Sharp corners, as a lane's own edge turns: a rounded join
        // leaves a string of vertices at every bend. Only a corner sharper
        // than 30° is cut off.
        .line_join(LineJoin::Miter(std::f64::consts::PI / 6.0))
        .start_cap(LineCap::Butt)
        .end_cap(LineCap::Butt);
    let shapes = points.to_vec().stroke(style, false);
    MultiPolygon::new(
        shapes
            .into_iter()
            .filter_map(|mut contours| {
                if contours.is_empty() {
                    return None;
                }
                let exterior = ring(contours.remove(0));
                Some(Polygon::new(
                    exterior,
                    contours.into_iter().map(ring).collect(),
                ))
            })
            .collect(),
    )
}

/// A `along` × `across` rectangle centred on `centre`, its `along` side
/// parallel to `axis` (a unit vector).
pub fn rectangle(centre: Pt, axis: Pt, along: f64, across: f64) -> MultiPolygon<f64> {
    let normal = right_of(axis);
    let corner = |u: f64, v: f64| {
        [
            centre[0] + axis[0] * u + normal[0] * v,
            centre[1] + axis[1] * u + normal[1] * v,
        ]
    };
    let (u, v) = (along / 2.0, across / 2.0);
    MultiPolygon::new(vec![Polygon::new(
        ring(vec![
            corner(-u, -v),
            corner(u, -v),
            corner(u, v),
            corner(-u, v),
        ]),
        Vec::new(),
    )])
}

/// The polygon with outline `points`.
pub fn polygon(points: Vec<Pt>) -> MultiPolygon<f64> {
    MultiPolygon::new(vec![Polygon::new(ring(points), Vec::new())])
}

fn ring(points: Vec<Pt>) -> LineString<f64> {
    let mut coords: Vec<Coord<f64>> = points.into_iter().map(|[x, y]| Coord { x, y }).collect();
    if coords.first() != coords.last()
        && let Some(&first) = coords.first()
    {
        coords.push(first);
    }
    LineString::new(coords)
}

/// Whether two polylines meet anywhere, touching included: a path running
/// exactly through another's vertex (a one-lane one-way road's centreline
/// through the crossing node on it) meets it as surely as one crossing
/// mid-segment, and treating that as a miss would put both on green.
pub fn polylines_meet(a: &[Pt], b: &[Pt]) -> bool {
    use geo::Intersects;
    let line =
        |points: &[Pt]| LineString::new(points.iter().map(|&[x, y]| Coord { x, y }).collect());
    a.len() >= 2 && b.len() >= 2 && line(a).intersects(&line(b))
}

/// Where the lines through `p` along `d` and through `q` along `e` meet,
/// as the distances along each: `p + d·t = q + e·u`.
pub fn line_intersection(p: Pt, d: Pt, q: Pt, e: Pt) -> Option<(f64, f64)> {
    let det = d[0] * e[1] - d[1] * e[0];
    if det.abs() < 1e-6 {
        return None;
    }
    let (wx, wy) = (q[0] - p[0], q[1] - p[1]);
    Some(((wx * e[1] - wy * e[0]) / det, (wx * d[1] - wy * d[0]) / det))
}

/// `n + 1` points along the quadratic Bézier from `a` to `b` bending
/// towards `control`.
pub fn bezier(a: Pt, control: Pt, b: Pt, n: usize) -> Vec<Pt> {
    (0..=n)
        .map(|i| {
            let t = i as f64 / n as f64;
            let (u, v, w) = ((1.0 - t) * (1.0 - t), 2.0 * t * (1.0 - t), t * t);
            [
                u * a[0] + v * control[0] + w * b[0],
                u * a[1] + v * control[1] + w * b[1],
            ]
        })
        .collect()
}

/// Spacing of the points a line is sampled at to tell which of two sets of
/// lines a point is nearer to.
const NEAREST_SAMPLE_METERS: f64 = 0.25;
/// How far inside each end of a line its first and last samples sit.
const END_INSET_METERS: f64 = 0.01;

/// The part of `area` nearer to some line in `a` than to every line in `b`.
///
/// Each line is sampled every [`NEAREST_SAMPLE_METERS`]; a sample of `a`
/// owns the convex cell of points nearer to it than to any sample of `b`,
/// and the cells' union is `a`'s side. The boundary follows the true
/// equidistant curve to within the sample spacing, and the part of `area`
/// left out is exactly `b`'s side, so two zones cut this way meet with no
/// gap and no overlap.
pub fn nearer_to(a: &[Vec<Pt>], b: &[Vec<Pt>], area: &MultiPolygon<f64>) -> MultiPolygon<f64> {
    use geo::{BooleanOps, BoundingRect};
    let Some(bounds) = area.bounding_rect() else {
        return MultiPolygon::new(Vec::new());
    };
    let (min, max) = (bounds.min(), bounds.max());
    let frame = vec![
        [min.x - 1.0, min.y - 1.0],
        [max.x + 1.0, min.y - 1.0],
        [max.x + 1.0, max.y + 1.0],
        [min.x - 1.0, max.y + 1.0],
    ];
    let (a, b) = (samples(a), samples(b));
    if b.is_empty() {
        return area.clone();
    }
    let mut side = MultiPolygon::new(Vec::new());
    for &p in &a {
        let mut cell = frame.clone();
        for &q in &b {
            // Keep the points x with (x - mid)·(p - q) ≥ 0.
            let normal = [p[0] - q[0], p[1] - q[1]];
            if normal[0].hypot(normal[1]) < 1e-9 {
                continue;
            }
            let mid = [(p[0] + q[0]) / 2.0, (p[1] + q[1]) / 2.0];
            cell = clip_half_plane(&cell, mid, normal);
            if cell.len() < 3 {
                break;
            }
        }
        if cell.len() >= 3 {
            side = side.union(&Polygon::new(ring(cell), Vec::new()));
        }
    }
    area.intersection(&side)
}

/// Points along every line of `lines`, [`NEAREST_SAMPLE_METERS`] apart,
/// starting and ending [`END_INSET_METERS`] inside each line: two
/// crosswalks sharing an end (the corner of an island) then never share a
/// sample, and the ground past that corner splits along the bisector of the
/// angle between them instead of all going to one.
fn samples(lines: &[Vec<Pt>]) -> Vec<Pt> {
    let mut out = Vec::new();
    for line in lines {
        let length = length(line);
        if length <= 2.0 * END_INSET_METERS {
            out.extend(line.first());
            continue;
        }
        let inner = length - 2.0 * END_INSET_METERS;
        let steps = (inner / NEAREST_SAMPLE_METERS).ceil().max(1.0) as usize;
        for k in 0..=steps {
            let at = END_INSET_METERS + inner * k as f64 / steps as f64;
            out.extend(sub_polyline(line, 0.0, at).last());
        }
    }
    out
}

/// The convex polygon `points` cut to the half-plane of points `x` with
/// `(x - origin)·normal ≥ 0`.
fn clip_half_plane(points: &[Pt], origin: Pt, normal: Pt) -> Vec<Pt> {
    let side = |p: Pt| (p[0] - origin[0]) * normal[0] + (p[1] - origin[1]) * normal[1];
    let mut out = Vec::new();
    for i in 0..points.len() {
        let (p, q) = (points[i], points[(i + 1) % points.len()]);
        let (sp, sq) = (side(p), side(q));
        if sp >= 0.0 {
            out.push(p);
        }
        if (sp >= 0.0) != (sq >= 0.0) {
            out.push(lerp(p, q, sp / (sp - sq)));
        }
    }
    out
}

/// `polygon` with every hole filled.
pub fn without_holes(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    use geo::BooleanOps;
    polygon
        .0
        .into_iter()
        .map(|part| MultiPolygon::new(vec![Polygon::new(part.exterior().clone(), Vec::new())]))
        .fold(MultiPolygon::new(Vec::new()), |acc, part| acc.union(&part))
}

/// Vertices closer than this are one point.
pub const WELD_METERS: f64 = 0.05;
/// A ring turning back on itself by more than this at a vertex runs out and
/// straight back: a needle or a slit, never real ground.
const TURN_BACK_DEGREES: f64 = 170.0;
/// A vertex closer than this to the straight line between its neighbours
/// doesn't change the shape.
pub const REDUNDANT_VERTEX_METERS: f64 = 0.01;
/// A ring piece smaller than this left over from cleaning is noise.
const MIN_RING_AREA_M2: f64 = 0.5;

/// `polygon` without the zero-width defects unions and cuts leave behind —
/// OGC-valid when their two sides stay a hair apart, but drawn on a map as a
/// line through the zone: vertices a hair apart welded into one, needles and
/// slits (the ring running out and straight back) removed, and a ring that
/// touches itself at a point split into its two loops.
pub fn clean_rings(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    use geo::{Area, BooleanOps};
    let mut parts = Vec::new();
    let mut holes: Vec<Vec<Pt>> = Vec::new();
    for part in polygon.0 {
        holes.extend(
            part.interiors()
                .iter()
                .flat_map(|ring| clean_ring(ring_points(ring)))
                .filter(|ring| ring.len() >= 3),
        );
        // A hole touching the outline at a point can come back as a loop
        // of the exterior ring itself, pinched there: splitting the pinch
        // yields it as a loop running the other way round, which is a
        // hole, not more ground.
        let points = ring_points(part.exterior());
        let orientation = signed_area(&points).signum();
        for ring in clean_ring(points) {
            if signed_area(&ring).signum() != orientation {
                holes.push(ring);
                continue;
            }
            let candidate = Polygon::new(closed(&ring), Vec::new());
            if candidate.unsigned_area() >= MIN_RING_AREA_M2 {
                parts.push(candidate);
            }
        }
    }
    let mut whole = MultiPolygon::new(parts);
    for hole in holes {
        // Opened a few centimetres wider than itself, so a hole that
        // touched the outline becomes a notch rather than a hole pinned to
        // it at a point.
        let mut outline = hole.clone();
        outline.push(hole[0]);
        let opened = MultiPolygon::new(vec![Polygon::new(closed(&hole), Vec::new())])
            .union(&stroke(&outline, 2.0 * WELD_METERS));
        whole = whole.difference(&opened);
    }
    whole
}

/// Twice the signed area of the ring through `points` (positive
/// counter-clockwise).
fn signed_area(points: &[Pt]) -> f64 {
    (0..points.len())
        .map(|i| {
            let (p, q) = (points[i], points[(i + 1) % points.len()]);
            p[0] * q[1] - q[0] * p[1]
        })
        .sum()
}

/// `polygon` without the vertices that sit on the straight line between
/// their neighbours — only those whose removal shrinks it, so its outline
/// never moves outwards onto ground a neighbour holds.
pub fn straighten_inwards(polygon: MultiPolygon<f64>) -> MultiPolygon<f64> {
    let clean = |ring: &LineString<f64>, outline: bool| {
        let mut points = ring_points(ring);
        let orientation = signed_area(&points).signum();
        loop {
            let n = points.len();
            if n < 4 {
                break;
            }
            let shrinks = |i: usize| {
                let (a, b, c) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
                let turn = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
                // Cutting a convex corner of an outline off shrinks it; for
                // a hole it's the other way round.
                off_chord(a, b, c) <= REDUNDANT_VERTEX_METERS
                    && (turn * orientation >= 0.0) == outline
            };
            match (0..n).find(|&i| shrinks(i) && removable(&points, i)) {
                Some(i) => {
                    points.remove(i);
                }
                None => break,
            }
        }
        closed(&points)
    };
    MultiPolygon::new(
        polygon
            .0
            .iter()
            .map(|part| {
                Polygon::new(
                    clean(part.exterior(), true),
                    part.interiors().iter().map(|h| clean(h, false)).collect(),
                )
            })
            .collect(),
    )
}

/// Whether the two sides of the turn `a → b → c` stay within
/// [`WELD_METERS`] of each other: the nearer end sits that close to the
/// other side's line.
pub fn hairline_apart(a: Pt, b: Pt, c: Pt) -> bool {
    let to_line = |p: Pt, from: Pt, to: Pt| {
        let (dx, dy) = (to[0] - from[0], to[1] - from[1]);
        let n = dx.hypot(dy);
        if n <= 0.0 {
            dist(p, from)
        } else {
            ((p[0] - from[0]) * dy - (p[1] - from[1]) * dx).abs() / n
        }
    };
    to_line(a, b, c).min(to_line(c, a, b)) <= WELD_METERS
}

/// How far `b` lies from the segment `a`–`c`.
pub fn off_chord(a: Pt, b: Pt, c: Pt) -> f64 {
    let (dx, dy) = (c[0] - a[0], c[1] - a[1]);
    let len2 = dx * dx + dy * dy;
    if len2 <= 0.0 {
        return dist(a, b);
    }
    let t = (((b[0] - a[0]) * dx + (b[1] - a[1]) * dy) / len2).clamp(0.0, 1.0);
    dist(b, [a[0] + dx * t, a[1] + dy * t])
}

fn ring_points(ring: &LineString<f64>) -> Vec<Pt> {
    let mut points: Vec<Pt> = ring.coords().map(|c| [c.x, c.y]).collect();
    if points.len() > 1 && points.first() == points.last() {
        points.pop();
    }
    points
}

fn closed(points: &[Pt]) -> LineString<f64> {
    ring(points.to_vec())
}

/// Whether dropping vertex `i` keeps the ring from crossing itself: the
/// new edge joining its neighbours crosses no other edge. Near a neck a
/// hair wide, moving an edge by a centimetre can push it across the other
/// side.
fn removable(points: &[Pt], i: usize) -> bool {
    let n = points.len();
    let (a, c) = (points[(i + n - 1) % n], points[(i + 1) % n]);
    let d = [c[0] - a[0], c[1] - a[1]];
    (0..n).all(|k| {
        // Edges touching the new one at its ends can't cross it.
        let (p, q) = (points[k], points[(k + 1) % n]);
        if [k, (k + 1) % n]
            .iter()
            .any(|&v| v == i || v == (i + n - 1) % n || v == (i + 1) % n)
        {
            return true;
        }
        let e = [q[0] - p[0], q[1] - p[1]];
        match line_intersection(a, d, p, e) {
            // Well inside both: touching near an end is what the next step
            // of a needle folded twice over looks like, and cleaning on
            // removes it.
            Some((t, u)) => {
                let (new_length, other_length) = (dist(a, c), dist(p, q));
                let inside = |x: f64, length: f64| {
                    x * length > WELD_METERS && (1.0 - x) * length > WELD_METERS
                };
                !(inside(t, new_length) && inside(u, other_length))
            }
            None => true,
        }
    })
}

/// One ring, cleaned, as one or more rings (a pinched ring splits in two).
fn clean_ring(mut points: Vec<Pt>) -> Vec<Vec<Pt>> {
    let cos_limit = TURN_BACK_DEGREES.to_radians().cos();
    loop {
        let n = points.len();
        if n < 3 {
            return Vec::new();
        }
        // Weld a vertex onto its neighbour when they're a hair apart.
        if let Some(i) = (0..n).find(|&i| dist(points[i], points[(i + 1) % n]) <= WELD_METERS) {
            points.remove((i + 1) % n);
            continue;
        }
        // A vertex the ring runs out to and straight back from, its two
        // sides a hair apart. A sharp corner whose sides spread apart (a
        // lane band cut obliquely by a neighbour) is real ground and stays.
        let turns_back = |i: usize| {
            let (a, b, c) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
            match (
                unit([b[0] - a[0], b[1] - a[1]]),
                unit([c[0] - b[0], c[1] - b[1]]),
            ) {
                (Some(u), Some(v)) => {
                    u[0] * v[0] + u[1] * v[1] < cos_limit && hairline_apart(a, b, c)
                }
                _ => false,
            }
        };
        if let Some(i) = (0..n).find(|&i| turns_back(i) && removable(&points, i)) {
            points.remove(i);
            continue;
        }
        // A vertex the ring runs straight through adds nothing to its shape.
        let redundant = |i: usize| {
            let (a, b, c) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
            off_chord(a, b, c) <= REDUNDANT_VERTEX_METERS
        };
        if let Some(i) = (0..n).find(|&i| redundant(i) && removable(&points, i)) {
            points.remove(i);
            continue;
        }
        // The ring meeting itself at a point: two loops, cleaned separately.
        for i in 0..n {
            for j in i + 2..n {
                if (i, j) != (0, n - 1) && dist(points[i], points[j]) <= WELD_METERS {
                    let inner: Vec<Pt> = points[i..j].to_vec();
                    let outer: Vec<Pt> = points[j..].iter().chain(&points[..i]).copied().collect();
                    return clean_ring(inner)
                        .into_iter()
                        .chain(clean_ring(outer))
                        .collect();
                }
            }
        }
        return vec![points];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use geo::Area;

    #[test]
    fn sub_polyline_cuts_inside_a_segment() {
        let line = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]];
        assert_eq!(
            sub_polyline(&line, 5.0, 15.0),
            vec![[5.0, 0.0], [10.0, 0.0], [10.0, 5.0]]
        );
    }

    #[test]
    fn offset_moves_to_the_right_of_travel() {
        // Travelling +x, right is -y.
        assert_eq!(
            offset(&[[0.0, 0.0], [10.0, 0.0]], 2.0),
            vec![[0.0, -2.0], [10.0, -2.0]]
        );
    }

    #[test]
    fn polylines_meet_when_crossing_or_touching_at_a_vertex() {
        assert!(polylines_meet(
            &[[0.0, 0.0], [2.0, 2.0]],
            &[[0.0, 2.0], [2.0, 0.0]]
        ));
        // A one-lane one-way road's centreline through the crossing node
        // that is also a vertex of the crosswalk.
        assert!(polylines_meet(
            &[[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]],
            &[[1.0, -1.0], [1.0, 0.0], [1.0, 1.0]]
        ));
        assert!(!polylines_meet(
            &[[0.0, 0.0], [2.0, 0.0]],
            &[[0.0, 1.0], [2.0, 1.0]]
        ));
    }

    #[test]
    fn a_slit_into_a_square_is_cleaned_away() {
        use geo::Area;
        // A 10x10 square with a zero-width slit from the middle of its right
        // side 5m inwards (out and straight back, the two sides 1cm apart).
        let ring = vec![
            [0.0, 0.0],
            [10.0, 0.0],
            [10.0, 5.0],
            [5.0, 5.0],
            [5.0, 5.01],
            [10.0, 5.01],
            [10.0, 10.0],
            [0.0, 10.0],
        ];
        let square = MultiPolygon::new(vec![Polygon::new(super::ring(ring), Vec::new())]);
        let cleaned = clean_rings(square);
        assert_eq!(cleaned.0.len(), 1);
        let points = ring_points(cleaned.0[0].exterior());
        assert!(
            points.iter().all(|p| p[0] == 0.0 || p[0] == 10.0),
            "slit still there: {points:?}"
        );
        assert!((cleaned.unsigned_area() - 100.0).abs() < 0.2);
    }

    #[test]
    fn a_sharp_but_real_corner_is_kept() {
        use geo::Area;
        // A long, thin wedge: 40m sides meeting at 8°, spreading 5.6m apart.
        let wedge = vec![
            [0.0, 0.0],
            [40.0, 0.0],
            [
                40.0 * 8f64.to_radians().cos(),
                40.0 * 8f64.to_radians().sin(),
            ],
        ];
        let polygon = MultiPolygon::new(vec![Polygon::new(super::ring(wedge), Vec::new())]);
        let before = polygon.unsigned_area();
        assert!((clean_rings(polygon).unsigned_area() - before).abs() < 1e-9);
    }

    #[test]
    fn stroke_of_a_straight_line_is_its_rectangle() {
        let area = stroke(&[[0.0, 0.0], [10.0, 0.0]], 3.0).unsigned_area();
        assert!((area - 30.0).abs() < 1e-6, "{area}");
    }

    #[test]
    fn shared_ground_goes_to_the_nearest_line_even_past_a_shared_corner() {
        use geo::{BooleanOps, Contains, Point};
        // Two crosswalks meeting at a right angle at the origin.
        let a = vec![vec![[0.0, 0.0], [10.0, 0.0]]];
        let b = vec![vec![[0.0, 0.0], [0.0, 10.0]]];
        let area = rectangle([0.0, 0.0], [1.0, 0.0], 8.0, 8.0);
        let near_a = nearer_to(&a, &b, &area);
        let near_b = nearer_to(&b, &a, &area);
        assert!(near_a.intersection(&near_b).unsigned_area() < 0.05);
        assert!((near_a.unsigned_area() + near_b.unsigned_area() - 64.0).abs() < 0.05);
        let inside = |p: &MultiPolygon<f64>, x: f64, y: f64| p.contains(&Point::new(x, y));
        // Nearer a's line, and past the corner either side of the bisector.
        assert!(inside(&near_a, 3.0, 1.0) && inside(&near_b, 1.0, 3.0));
        assert!(inside(&near_a, -1.0, -3.0) && inside(&near_b, -3.0, -1.0));
    }

    #[test]
    fn filling_holes_keeps_the_outline() {
        use geo::BooleanOps;
        let ring = rectangle([0.0, 0.0], [1.0, 0.0], 10.0, 10.0).difference(&rectangle(
            [0.0, 0.0],
            [1.0, 0.0],
            2.0,
            2.0,
        ));
        assert_eq!(ring.0[0].interiors().len(), 1);
        let filled = without_holes(ring);
        assert!(filled.0.iter().all(|p| p.interiors().is_empty()));
        assert!((filled.unsigned_area() - 100.0).abs() < 1e-6);
    }
}

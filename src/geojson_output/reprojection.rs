use anyhow::{Context, Result, bail};
use proj4rs::proj::Proj;
use proj4rs::transform::transform;
use sumo_types::additional::domain::LanePosition;
use sumo_types::domain::{Location, Point, Projection, Shape};
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

pub const WGS84_LONLAT: &str = "+proj=longlat +ellps=WGS84 +datum=WGS84 +no_defs";

pub const MIN_DRAWN_LANE_LENGTH_METERS: f64 = 5.0;

pub fn padded_entry(entry: Length, exit: Length, target_length: Length) -> Length {
    let span = exit - entry;
    if span < target_length { entry - (target_length - span) } else { entry }
}

pub struct Reprojector {
    from: Proj,
    to: Proj,
    net_offset: Point,
}

impl Reprojector {
    pub fn new(location: &Location) -> Result<Self> {
        let Projection::Proj4(proj_string) = &location.projection else {
            bail!(
                "network is not georeferenced (location/@projParameter is \"!\"): \
                 GeoJSON needs real-world coordinates, and an unprojected network has none to give"
            );
        };
        let from = Proj::from_proj_string(proj_string).with_context(|| {
            format!("invalid PROJ4 string in .net.xml location: {proj_string:?}")
        })?;
        let to =
            Proj::from_proj_string(WGS84_LONLAT).expect("WGS84_LONLAT is a valid PROJ4 string");
        Ok(Self {
            from,
            to,
            net_offset: location.net_offset,
        })
    }

    pub fn to_lon_lat(&self, point: Point) -> Result<[f64; 2]> {
        let mut coords = (
            point.x - self.net_offset.x,
            point.y - self.net_offset.y,
            0.0,
        );
        transform(&self.from, &self.to, &mut coords)
            .with_context(|| format!("could not reproject network point {point:?} to WGS84"))?;
        Ok([coords.0.to_degrees(), coords.1.to_degrees()])
    }
}

pub fn shape_length(shape: &Shape) -> Length {
    Length::new::<meter>(
        shape.0.windows(2).map(|window| {
            let [a, b] = window else {
                unreachable!("windows(2) always yields length-2 slices")
            };
            (b.x - a.x).hypot(b.y - a.y)
        }).sum(),
    )
}

pub fn point_and_tangent_at(shape: &Shape, distance: Length) -> (Point, (f64, f64)) {
    let target = distance.get::<meter>().max(0.0);
    let mut travelled = 0.0;

    for window in shape.0.windows(2) {
        let [a, b] = window else {
            unreachable!("windows(2) always yields length-2 slices")
        };
        let (dx, dy) = (b.x - a.x, b.y - a.y);
        let segment_length = dx.hypot(dy);
        if segment_length > 0.0 && travelled + segment_length < target {
            travelled += segment_length;
            continue;
        }

        let t = if segment_length > 0.0 {
            (target - travelled) / segment_length
        } else {
            0.0
        };
        let point = Point {
            x: a.x + dx * t,
            y: a.y + dy * t,
            z: a.z + (b.z - a.z) * t,
        };
        let tangent = if segment_length > 0.0 {
            (dx / segment_length, dy / segment_length)
        } else {
            (1.0, 0.0)
        };
        return (point, tangent);
    }

    (shape.0.last().copied().unwrap_or_default(), (1.0, 0.0))
}

pub fn distance_from_start(position: LanePosition, lane_length: Length) -> Length {
    match position {
        LanePosition::FromStart(distance) => distance,
        LanePosition::FromEnd(distance) => lane_length - distance,
    }
}

pub fn trimmed_segments(shape: &Shape, entry: Length, exit: Length) -> Vec<(Point, Point, (f64, f64))> {
    let entry_m = entry.get::<meter>();
    let exit_m = exit.get::<meter>().max(entry_m);

    let mut segments = Vec::new();

    if entry_m < 0.0
        && let [first, second, ..] = shape.0.as_slice()
    {
        let (dx, dy) = (second.x - first.x, second.y - first.y);
        let segment_length = dx.hypot(dy);
        if segment_length > 0.0 {
            let tangent = (dx / segment_length, dy / segment_length);
            let extend_to = exit_m.min(0.0);
            if extend_to > entry_m {
                let point_at = |t: f64| Point {
                    x: first.x + tangent.0 * t,
                    y: first.y + tangent.1 * t,
                    z: first.z,
                };
                segments.push((point_at(entry_m), point_at(extend_to), tangent));
            }
        }
    }

    let mut travelled = 0.0;
    for window in shape.0.windows(2) {
        let [a, b] = window else {
            unreachable!("windows(2) always yields length-2 slices")
        };
        let segment_length = (b.x - a.x).hypot(b.y - a.y);
        if segment_length <= 0.0 {
            continue;
        }
        let (segment_start, segment_end) = (travelled, travelled + segment_length);
        travelled = segment_end;
        if segment_end <= entry_m || segment_start >= exit_m {
            continue;
        }

        let tangent = ((b.x - a.x) / segment_length, (b.y - a.y) / segment_length);
        let lerp = |t: f64| Point {
            x: a.x + (b.x - a.x) * t,
            y: a.y + (b.y - a.y) * t,
            z: a.z + (b.z - a.z) * t,
        };
        let t0 = (entry_m.max(segment_start) - segment_start) / segment_length;
        let t1 = (exit_m.min(segment_end) - segment_start) / segment_length;
        segments.push((lerp(t0), lerp(t1), tangent));

        if travelled >= exit_m {
            break;
        }
    }
    segments
}

pub fn trimmed_path_points(shape: &Shape, entry: Length, exit: Length) -> Vec<Point> {
    let segments = trimmed_segments(shape, entry, exit);
    let mut points = Vec::with_capacity(segments.len() + 1);
    for (i, &(start, end, _)) in segments.iter().enumerate() {
        if i == 0 {
            points.push(start);
        }
        points.push(end);
    }
    points
}

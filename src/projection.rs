//! Lon/lat ↔ local metres, via an equirectangular projection centred on the
//! extract. At city scale (a few km) its distortion stays well under 0.1%,
//! far below anything a waiting zone's geometry is sensitive to.

const EARTH_RADIUS_METERS: f64 = 6_371_008.8;

#[derive(Clone, Copy)]
pub struct Projection {
    lon0: f64,
    lat0: f64,
    meters_per_degree_lon: f64,
    meters_per_degree_lat: f64,
}

impl Projection {
    pub fn centred_on(lon_lats: impl Iterator<Item = [f64; 2]>) -> Self {
        let (mut min, mut max) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
        for [lon, lat] in lon_lats {
            min = [min[0].min(lon), min[1].min(lat)];
            max = [max[0].max(lon), max[1].max(lat)];
        }
        let (lon0, lat0) = ((min[0] + max[0]) / 2.0, (min[1] + max[1]) / 2.0);
        let meters_per_degree_lat = EARTH_RADIUS_METERS.to_radians();
        Self {
            lon0,
            lat0,
            meters_per_degree_lon: meters_per_degree_lat * lat0.to_radians().cos(),
            meters_per_degree_lat,
        }
    }

    pub fn to_local(&self, [lon, lat]: [f64; 2]) -> [f64; 2] {
        [
            (lon - self.lon0) * self.meters_per_degree_lon,
            (lat - self.lat0) * self.meters_per_degree_lat,
        ]
    }

    pub fn to_lon_lat(&self, [x, y]: [f64; 2]) -> [f64; 2] {
        [
            x / self.meters_per_degree_lon + self.lon0,
            y / self.meters_per_degree_lat + self.lat0,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trips_and_measures_real_distances() {
        let projection = Projection::centred_on([[2.2, 41.4], [2.3, 41.5]].into_iter());
        let a = [2.25, 41.45];
        let back = projection.to_lon_lat(projection.to_local(a));
        assert!((back[0] - a[0]).abs() < 1e-12 && (back[1] - a[1]).abs() < 1e-12);
        // One thousandth of a degree of latitude is ~111m anywhere.
        let [_, y0] = projection.to_local([2.25, 41.45]);
        let [_, y1] = projection.to_local([2.25, 41.451]);
        assert!(((y1 - y0) - 111.2).abs() < 0.5);
    }
}

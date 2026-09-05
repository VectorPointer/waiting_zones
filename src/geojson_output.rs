mod feature;
mod geometry;
mod overlaps;
mod reprojection;
#[cfg(test)]
mod tests;

pub use feature::{to_feature_collection, write};
pub use overlaps::{overlapping_zone_ids, overlapping_zone_ids_larger_than};
#[cfg(test)]
pub(crate) use overlaps::distance_to_polygon;

#[cfg(test)]
pub(crate) use feature::zone_feature;
#[cfg(test)]
pub(crate) use geometry::single_successors;
#[cfg(test)]
pub(crate) use overlaps::{feature_rings, find_near_touch, weld_near_touch_and_split};
#[cfg(test)]
pub(crate) use reprojection::{MIN_DRAWN_LANE_LENGTH_METERS, Reprojector};

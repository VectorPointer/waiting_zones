mod feature;
mod geometry;
mod overlaps;
mod reprojection;
#[cfg(test)]
mod tests;

pub use feature::{to_feature_collection, write};
pub use overlaps::overlapping_zone_ids;

#[cfg(test)]
pub(crate) use feature::{build_feature, stop_line_point, zone_feature};
#[cfg(test)]
pub(crate) use geometry::{single_successors, zone_modes, zone_polygon};
#[cfg(test)]
pub(crate) use overlaps::{feature_rings, resolve_overlaps};
#[cfg(test)]
pub(crate) use reprojection::{MIN_DRAWN_LANE_LENGTH_METERS, Reprojector};

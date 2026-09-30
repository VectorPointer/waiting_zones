/// Matches a real OSM crosswalk to the SUMO crossing lane it belongs to —
/// see that module's own docs.
mod crosswalks;
mod feature;
mod geometry;
mod overlaps;
/// Pedestrian zones: the crosswalk plus a sidewalk offset.
mod pedestrian;
mod reprojection;
#[cfg(test)]
mod tests;

pub use crosswalks::{match_crosswalks, CrosswalkMatches};
pub use feature::{to_feature_collection, to_feature_collection_with_crosswalks, write};
pub use overlaps::{overlapping_zone_ids, overlapping_zone_ids_larger_than};
// Genuinely `pub`, not `#[cfg(test)] pub(crate)` like the rest of this
// file's own internal test-only re-exports below: those are invisible to
// an integration test under `tests/` (which links the library's own
// normal, non-`cfg(test)` build, not the one `cargo test`'s unit-test
// harness compiles), and `distance_to_polygon`/`feature_rings`/
// `Reprojector` are exactly what a real-data integration test needs to
// rebuild a shipped zone's own polygon from its GeoJSON and measure a
// point against it the same way this crate's own pipeline does — see
// `tests/pedestrian_zones_dont_split_a_crossing.rs`.
pub use overlaps::{distance_to_polygon, feature_rings};
pub use reprojection::Reprojector;

#[cfg(test)]
pub(crate) use feature::zone_feature;
#[cfg(test)]
pub(crate) use geometry::lane_links;
#[cfg(test)]
pub(crate) use overlaps::{find_near_touch, relax_needle_vertices_everywhere, resolve_pedestrian_overlaps, weld_near_touch_and_split};
#[cfg(test)]
pub(crate) use reprojection::MIN_DRAWN_LANE_LENGTH_METERS;

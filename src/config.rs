use clap::Parser;
use std::path::PathBuf;
use sumo_types::uom::si::f64::Length;
use sumo_types::uom::si::length::meter;

const NET_XML_SUFFIX: &str = ".net.xml";

#[derive(Parser)]
struct Cli {
    input: PathBuf,

    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Caps how far a waiting zone's entry extends from the stop line /
    /// crossing (in meters). Without it, a zone spans the full length of
    /// its lane; with it, the entry moves to `length - max`, keeping the
    /// exit anchored at the stop line / crossing.
    #[arg(long, value_name = "METERS")]
    max_zone_length: Option<f64>,

    /// Also write the waiting zones as 2 GeoJSON `FeatureCollection`s (one
    /// per mode — see `geojson_output::write`'s own docs) at paths derived
    /// from this one, reprojected to WGS84 lon/lat. Requires the input
    /// network to be georeferenced (`location/@projParameter` other than
    /// `"!"`).
    #[arg(long, value_name = "PATH")]
    geojson: Option<PathBuf>,

    /// Stops a vehicle zone's backward extension the moment it would reach
    /// a junction that's part of a `joinTLS`-merged traffic light program
    /// spanning more than one junction — a real, physically complex
    /// intersection SUMO modeled as a cluster of closely-spaced nodes
    /// linked by near-zero-length edges (see
    /// `zone_generator::extended_entry_lanes`'s own docs). Without this,
    /// nothing inside that cluster individually looks like a fork or an
    /// existing signal, so extension walks straight through the whole
    /// thing, unioning dozens of tiny, oddly-angled lane buffers into one
    /// zone polygon — confirmed on real Barcelona data
    /// (`203480266#0_straight`) to produce a self-intersecting shape from
    /// this. Off by default: it trades away ever reaching a genuinely
    /// upstream signal beyond the cluster for guaranteed-simple geometry
    /// through it, and that trade isn't free everywhere it'd apply — most
    /// zones' own signal only ever names one junction, so most zones are
    /// unaffected either way.
    #[arg(long)]
    stop_at_complex_intersections: bool,
}

pub struct Config {
    pub input: PathBuf,
    pub output: PathBuf,
    pub max_zone_length: Option<Length>,
    pub geojson_output: Option<PathBuf>,
    pub stop_at_complex_intersections: bool,
}

impl Config {
    pub fn build() -> Self {
        let raw = Cli::parse();
        let output = raw.output.unwrap_or_else(|| {
            let file_name = raw
                .input
                .file_name()
                .and_then(|name| name.to_str())
                .unwrap_or("output");

            let stem = file_name.strip_suffix(NET_XML_SUFFIX).unwrap_or_else(|| {
                raw.input
                    .file_stem()
                    .and_then(|s| s.to_str())
                    .unwrap_or(file_name)
            });

            raw.input
                .with_file_name(format!("{stem}.waiting-zones.add.xml"))
        });

        Config {
            input: raw.input,
            output,
            max_zone_length: raw.max_zone_length.map(Length::new::<meter>),
            geojson_output: raw.geojson,
            stop_at_complex_intersections: raw.stop_at_complex_intersections,
        }
    }
}

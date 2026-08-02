use clap::Parser;
use std::path::PathBuf;
use uom::si::f64::Length;
use uom::si::length::meter;

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
}

pub struct Config {
    pub input: PathBuf,
    pub output: PathBuf,
    pub max_zone_length: Option<Length>,
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
        }
    }
}

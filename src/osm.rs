//! A shallow OSM XML reader: nodes (with id, position and tags) and ways
//! (with id, node refs and tags). Relations are ignored.

use std::collections::HashMap;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;

use anyhow::{Context, Result};
use quick_xml::Reader;
use quick_xml::XmlVersion;
use quick_xml::events::{BytesStart, Event};

pub type NodeId = i64;
pub type WayId = i64;
pub type Tags = HashMap<String, String>;

pub struct Node {
    pub id: NodeId,
    /// `[lon, lat]`.
    pub lon_lat: [f64; 2],
    pub tags: Tags,
}

pub struct Way {
    pub id: WayId,
    pub refs: Vec<NodeId>,
    pub tags: Tags,
}

impl Way {
    pub fn tag(&self, key: &str) -> Option<&str> {
        self.tags.get(key).map(String::as_str)
    }
}

impl Node {
    pub fn tag(&self, key: &str) -> Option<&str> {
        self.tags.get(key).map(String::as_str)
    }
}

/// A turn restriction (`type=restriction` relation): from one way, through
/// a node, onto another way.
pub struct Restriction {
    pub from: WayId,
    pub via: NodeId,
    pub to: WayId,
    /// `true` for `only_*` (every other turn is forbidden), `false` for
    /// `no_*` (this turn is).
    pub only: bool,
    /// What follows `no_`/`only_`: `left_turn`, `right_turn`,
    /// `straight_on`, `u_turn`, …
    pub turn: String,
}

pub struct Osm {
    pub nodes: HashMap<NodeId, Node>,
    pub ways: Vec<Way>,
    pub restrictions: Vec<Restriction>,
}

/// A relation while it's being read: its members `(type, ref, role)` and
/// tags.
#[derive(Default)]
struct RelationDraft {
    members: Vec<(String, i64, String)>,
    tags: Tags,
}

impl RelationDraft {
    fn restriction(&self) -> Option<Restriction> {
        if self.tags.get("type").map(String::as_str) != Some("restriction") {
            return None;
        }
        let kind = self.tags.get("restriction")?;
        let (only, turn) = match (kind.strip_prefix("only_"), kind.strip_prefix("no_")) {
            (Some(turn), _) => (true, turn),
            (_, Some(turn)) => (false, turn),
            _ => return None,
        };
        let member = |kind: &str, role: &str| {
            self.members
                .iter()
                .find(|(k, _, r)| k == kind && r == role)
                .map(|(_, id, _)| *id)
        };
        Some(Restriction {
            from: member("way", "from")?,
            via: member("node", "via")?,
            to: member("way", "to")?,
            only,
            turn: turn.to_string(),
        })
    }
}

pub fn read(path: &Path) -> Result<Osm> {
    let file = File::open(path).with_context(|| format!("opening {}", path.display()))?;
    read_from(BufReader::new(file)).with_context(|| format!("reading {}", path.display()))
}

pub fn read_from(source: impl BufRead) -> Result<Osm> {
    let mut reader = Reader::from_reader(source);
    reader.config_mut().trim_text(true);

    let mut nodes = HashMap::new();
    let mut ways = Vec::new();
    let mut current_node: Option<Node> = None;
    let mut current_way: Option<Way> = None;
    let mut current_relation: Option<RelationDraft> = None;
    let mut restrictions = Vec::new();
    let mut buf = Vec::new();

    loop {
        match reader.read_event_into(&mut buf)? {
            Event::Start(element) if element.name().as_ref() == "node" => {
                current_node = parse_node(&element);
            }
            Event::Empty(element) if element.name().as_ref() == "node" => {
                if let Some(node) = parse_node(&element) {
                    nodes.insert(node.id, node);
                }
            }
            Event::Start(element) if element.name().as_ref() == "way" => {
                current_way = attr(&element, "id")
                    .and_then(|id| id.parse().ok())
                    .map(|id| Way {
                        id,
                        refs: Vec::new(),
                        tags: Tags::new(),
                    });
            }
            Event::Empty(element) | Event::Start(element) if element.name().as_ref() == "nd" => {
                if let Some(way) = current_way.as_mut()
                    && let Some(reference) = attr(&element, "ref").and_then(|v| v.parse().ok())
                {
                    way.refs.push(reference);
                }
            }
            Event::Start(element) if element.name().as_ref() == "relation" => {
                current_relation = Some(RelationDraft::default());
            }
            Event::Empty(element) | Event::Start(element)
                if element.name().as_ref() == "member" =>
            {
                if let Some(relation) = current_relation.as_mut()
                    && let (Some(kind), Some(reference), Some(role)) = (
                        attr(&element, "type"),
                        attr(&element, "ref").and_then(|v| v.parse().ok()),
                        attr(&element, "role"),
                    )
                {
                    relation.members.push((kind, reference, role));
                }
            }
            Event::Empty(element) | Event::Start(element) if element.name().as_ref() == "tag" => {
                if let (Some(key), Some(value)) = (attr(&element, "k"), attr(&element, "v")) {
                    if let Some(node) = current_node.as_mut() {
                        node.tags.insert(key, value);
                    } else if let Some(way) = current_way.as_mut() {
                        way.tags.insert(key, value);
                    } else if let Some(relation) = current_relation.as_mut() {
                        relation.tags.insert(key, value);
                    }
                }
            }
            Event::End(element) if element.name().as_ref() == "relation" => {
                if let Some(restriction) = current_relation.take().and_then(|r| r.restriction()) {
                    restrictions.push(restriction);
                }
            }
            Event::End(element) if element.name().as_ref() == "node" => {
                if let Some(node) = current_node.take() {
                    nodes.insert(node.id, node);
                }
            }
            Event::End(element) if element.name().as_ref() == "way" => {
                if let Some(way) = current_way.take() {
                    ways.push(way);
                }
            }
            Event::Eof => break,
            _ => {}
        }
        buf.clear();
    }

    Ok(Osm {
        nodes,
        ways,
        restrictions,
    })
}

fn parse_node(element: &BytesStart<'_>) -> Option<Node> {
    let id = attr(element, "id")?.parse().ok()?;
    let lat = attr(element, "lat")?.parse().ok()?;
    let lon = attr(element, "lon")?.parse().ok()?;
    Some(Node {
        id,
        lon_lat: [lon, lat],
        tags: Tags::new(),
    })
}

fn attr(element: &BytesStart<'_>, key: &str) -> Option<String> {
    element
        .attributes()
        .flatten()
        .find(|attribute| attribute.key.as_ref() == key)
        .and_then(|attribute| attribute.normalized_value(XmlVersion::Implicit1_0).ok())
        .map(|value| value.into_owned())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reads_nodes_ways_and_their_tags() {
        let osm = read_from(
            r#"<osm>
                <node id="1" lat="41.0" lon="2.0"><tag k="highway" v="traffic_signals"/></node>
                <node id="2" lat="41.001" lon="2.001"/>
                <way id="10"><nd ref="1"/><nd ref="2"/><tag k="highway" v="primary"/></way>
            </osm>"#
                .as_bytes(),
        )
        .unwrap();
        assert_eq!(osm.nodes.len(), 2);
        assert_eq!(osm.nodes[&1].tag("highway"), Some("traffic_signals"));
        assert_eq!(osm.nodes[&2].lon_lat, [2.001, 41.001]);
        assert_eq!(osm.ways[0].refs, vec![1, 2]);
        assert_eq!(osm.ways[0].tag("highway"), Some("primary"));
    }

    #[test]
    fn reads_turn_restrictions() {
        let osm = read_from(
            r#"<osm>
                <relation id="5">
                    <member type="way" ref="10" role="from"/>
                    <member type="node" ref="2" role="via"/>
                    <member type="way" ref="11" role="to"/>
                    <tag k="type" v="restriction"/><tag k="restriction" v="no_left_turn"/>
                </relation>
                <relation id="6"><member type="way" ref="10" role="outer"/><tag k="type" v="multipolygon"/></relation>
            </osm>"#
                .as_bytes(),
        )
        .unwrap();
        assert_eq!(osm.restrictions.len(), 1);
        let r = &osm.restrictions[0];
        assert_eq!(
            (r.from, r.via, r.to, r.only, r.turn.as_str()),
            (10, 2, 11, false, "left_turn")
        );
    }
}

use anyhow::{Context, Result};
use inflector::Inflector;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{atomic::AtomicUsize, Arc};
use xsd_parser::{
    config::Schema,
    generate,
    models::{format_ident, format_unknown_variant, make_type_name, meta::MetaType, NameBuilder as DefaultNameBuilder},
    traits::{NameBuilder, Naming},
    Config, Ident2, Name, TypeIdent,
};

const ORIGINAL_XSD_DIR: &str = "xsd";
const MAIN_SCHEMA_FILE: &str = "net_file.xsd";
const OUTPUT_FILE_NAME: &str = "generated_schema.rs";

/// Some SUMO xsd files (types/base.xsd) declare DTD entity constants in
/// their own <!DOCTYPE ...> (e.g. <!ENTITY FloatPattern "[-+]?...">) and use
/// them inside xsd:pattern patterns (&FloatPattern;). The XML parser used by
/// xsd-parser does not resolve the DOCTYPE's internal subset, so those
/// references make parsing fail if left as-is.
///
/// Unlike neutralizing them (replacing them with a wildcard), here we
/// resolve them to their real value: since they're declared and used within
/// the same file, the substitution is textual and doesn't lose precision
/// from the original pattern. Standard XML entities (&amp; &lt; &gt; &apos;
/// &quot;) are left untouched.
fn resolve_dtd_entities(content: &str) -> String {
    let entities = parse_dtd_entities(content);
    if entities.is_empty() {
        return content.to_string();
    }

    let mut segments = content.split('&');
    let first = segments.next().unwrap_or_default().to_string();

    segments.fold(first, |mut resolved, segment| {
        resolved.push_str(&resolve_entity_reference(segment, &entities));
        resolved
    })
}

/// Extracts the `(name, value)` pairs from the `<!ENTITY ...>` declarations
/// present in `content`'s DOCTYPE internal subset.
fn parse_dtd_entities(content: &str) -> Vec<(&str, &str)> {
    content
        .lines()
        .filter_map(|line| {
            let rest = line.trim().strip_prefix("<!ENTITY ")?.strip_suffix('>')?;
            let (name, value) = rest.split_once(char::is_whitespace)?;
            let value = value.trim().strip_prefix('"')?.strip_suffix('"')?;
            Some((name.trim(), value))
        })
        .collect()
}

/// Resolves the piece of text that follows an `&` (the result of splitting
/// the original content on that character): if it starts with a known
/// entity reference, it's replaced with its value; otherwise, the `&` is
/// restored.
fn resolve_entity_reference(segment: &str, entities: &[(&str, &str)]) -> String {
    let Some((name, rest)) = segment.split_once(';') else {
        return format!("&{segment}");
    };

    let value = match name {
        "amp" | "lt" | "gt" | "apos" | "quot" => return format!("&{name};{rest}"),
        _ => entities.iter().find_map(|(n, v)| (*n == name).then_some(*v)),
    };

    match value {
        Some(value) => format!("{value}{rest}"),
        None => format!("&{segment}"),
    }
}

/// xsd-parser derives Rust type names from XSD primitive types
/// (xsd:float -> `FloatType`, xsd:time -> `TimeType`, xsd:ID -> `IdType`, ...).
/// SUMO defines its own simpleType "floatType", "timeType" and "idType" in
/// base.xsd, which, once capitalized, collide exactly with those generated
/// primitive names, and xsd-parser doesn't disambiguate them (E0428: name
/// defined twice). We rename them so they don't collide; this only affects
/// the generated `schema` layer, not our own code.
const RENAMED_TYPES: &[(&str, &str)] = &[
    (r#""idType""#, r#""sumoIdType""#),
    (r#""floatType""#, r#""sumoFloatType""#),
    (r#""timeType""#, r#""sumoTimeType""#),
];

fn rename_colliding_types(content: &str) -> String {
    RENAMED_TYPES
        .iter()
        .fold(content.to_string(), |content, (from, to)| {
            content.replace(from, to)
        })
}

/// Composition of the text patches applied to each `.xsd` before handing it
/// to xsd-parser.
fn patch_xsd(content: &str) -> String {
    rename_colliding_types(&resolve_dtd_entities(content))
}

/// Recursively copies `src` into `dst`, patching (see [`patch_xsd`]) any
/// `.xsd` file it finds along the way.
fn copy_and_patch_schemas(src: &Path, dst: &Path) -> Result<()> {
    fs::create_dir_all(dst)?;

    fs::read_dir(src)?.try_for_each(|entry| {
        let entry = entry?;
        let path = entry.path();
        let dest_path = dst.join(entry.file_name());

        if path.is_dir() {
            return copy_and_patch_schemas(&path, &dest_path);
        }

        if path.extension().is_some_and(|ext| ext == "xsd") {
            let content = fs::read_to_string(&path)?;
            fs::write(&dest_path, patch_xsd(&content))?;
        } else {
            fs::copy(&path, &dest_path)?;
        }

        Ok(())
    })
}

/// Naming strategy passed to xsd-parser (`Config::with_naming`) for
/// types/modules/fields/constants: identical to the default one (unifies to
/// PascalCase/snake_case/SCREAMING_SNAKE_CASE), but with
/// [`format_variant_name`] replaced by a version that doesn't lose
/// information (see its documentation).
///
/// The `unify` logic is duplicated here because xsd-parser doesn't expose it
/// publicly (`unify_string` is private to the crate); this is a faithful
/// copy of its default implementation.
#[derive(Debug, Clone, Default)]
struct SumoNaming(Arc<AtomicUsize>);

impl Naming for SumoNaming {
    fn clone_boxed(&self) -> Box<dyn Naming> {
        Box::new(self.clone())
    }

    fn builder(&self) -> Box<dyn NameBuilder> {
        Box::new(DefaultNameBuilder::new(self.0.clone(), Box::new(self.clone())))
    }

    fn unify(&self, s: &str) -> String {
        unify(s)
    }

    fn make_type_name(&self, postfixes: &[String], ty: &MetaType, ident: &TypeIdent) -> Name {
        make_type_name(self, postfixes, ty, ident)
    }

    fn make_unknown_variant(&self, id: usize) -> Ident2 {
        format_unknown_variant(id)
    }

    fn format_module_name(&self, s: &str) -> String {
        format_ident(self.unify(s).to_snake_case())
    }

    fn format_type_name(&self, s: &str) -> String {
        format_ident(self.unify(s))
    }

    fn format_field_name(&self, s: &str) -> String {
        format_ident(self.unify(s).to_snake_case())
    }

    fn format_variant_name(&self, s: &str) -> String {
        format_variant_name(s)
    }

    fn format_constant_name(&self, s: &str) -> String {
        format_ident(self.unify(s).to_screaming_snake_case())
    }
}

/// Faithful copy of the PascalCase normalization used by xsd-parser's
/// default `Naming` for types/modules/fields/constants (its real
/// implementation, `unify_string`, is not public).
fn unify(s: &str) -> String {
    let mut done = true;
    let unified = s.replace(
        |c: char| {
            let replace = !c.is_alphanumeric();
            if c != '_' && !replace {
                done = false;
            }
            c != '_' && replace
        },
        "_",
    );

    if done {
        unified
    } else {
        unified.to_screaming_snake_case().to_pascal_case()
    }
}

/// Builds an enum variant name from the XSD value `s`.
///
/// xsd-parser's default `Naming` unifies everything to PascalCase, which is
/// case-insensitive: XSD values that only differ by case (e.g. the
/// single-character codes net_file.xsd uses for `state="M"/"m"` or
/// `dir="s"/"t"/"T"`, or "true"/"True" in `boolType`) collapse into the same
/// Rust identifier, which xsd-parser neither detects nor disambiguates
/// (duplicate enum variants -> compile error).
///
/// Here, instead, we preserve all the information from the original value:
/// alphanumeric characters are kept as-is (with their original case), and
/// each non-alphanumeric symbol is translated to a distinct word (instead of
/// collapsing them all to "_", which would also produce the reserved `_`
/// identifier for single-symbol values like "-").
fn format_variant_name(s: &str) -> String {
    let sanitized: String = s
        .chars()
        .map(|c| {
            if c.is_alphanumeric() {
                c.to_string()
            } else {
                describe_symbol(c)
            }
        })
        .collect();

    format_ident(if sanitized.is_empty() {
        "Empty".to_string()
    } else {
        sanitized
    })
}

/// Gives a readable, distinct name to a non-alphanumeric character, so that
/// two different symbols (e.g. "-" and "=" in `connectionType/@state`) never
/// collapse into the same identifier.
fn describe_symbol(c: char) -> String {
    match c {
        '-' => "Dash".to_string(),
        '=' => "Eq".to_string(),
        '_' => "_".to_string(),
        other => format!("U{:x}", other as u32),
    }
}

fn main() -> Result<()> {
    println!("cargo:rerun-if-changed={ORIGINAL_XSD_DIR}");

    let out_dir =
        PathBuf::from(env::var("OUT_DIR").context("OUT_DIR environment variable is not set")?);
    let patched_xsd_dir = out_dir.join("patched_xsd");

    copy_and_patch_schemas(Path::new(ORIGINAL_XSD_DIR), &patched_xsd_dir)
        .context("Failed to patch and copy XSD schemas")?;

    let mut config = Config::default()
        .with_naming(SumoNaming::default())
        .with_quick_xml_deserialize();
    config.parser.schemas = vec![Schema::File(patched_xsd_dir.join(MAIN_SCHEMA_FILE))];

    let code = generate(config)
        .map_err(|e| anyhow::anyhow!("Error generating code from XSD schema: {e:?}"))?;

    let dest_path = out_dir.join(OUTPUT_FILE_NAME);
    fs::write(&dest_path, code.to_string())
        .with_context(|| format!("Failed to write generated file to {}", dest_path.display()))?;

    Ok(())
}

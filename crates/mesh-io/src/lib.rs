use std::{
    collections::HashMap,
    fs,
    path::{Path, PathBuf},
    str::{self, FromStr},
};

use serde::{Deserialize, Serialize};
use thiserror::Error;

pub type Point = [f32; 3];
pub type Face = [u32; 3];

/// Minimal mesh container used by downstream crates.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct Mesh {
    pub vertices: Vec<Point>,
    pub faces: Vec<Face>,
}

impl Default for Mesh {
    fn default() -> Self {
        Self {
            vertices: Vec::new(),
            faces: Vec::new(),
        }
    }
}

impl Mesh {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn face_count(&self) -> usize {
        self.faces.len()
    }

    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty() && self.faces.is_empty()
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum MeshFormat {
    Stl,
    Obj,
    Ply,
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum Units {
    Millimeters,
    Centimeters,
    Meters,
    Inches,
    Unspecified,
}

impl Default for Units {
    fn default() -> Self {
        Units::Unspecified
    }
}

#[derive(Debug, Default, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct MeshLoadReport {
    pub dropped_faces: usize,
    pub dropped_vertices: usize,
}

#[derive(Debug, Error)]
pub enum MeshError {
    #[error("mesh contains faces but less than three vertices ({0})")]
    TooFewVertices(usize),
}

#[derive(Debug, Error)]
pub enum MeshIoError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),
    #[error("utf8 error: {0}")]
    Utf8(#[from] str::Utf8Error),
    #[error("invalid mesh: {0}")]
    Invalid(String),
    #[error("unsupported format: {0}")]
    Unsupported(String),
    #[error(transparent)]
    Mesh(#[from] MeshError),
}

#[derive(Debug, Clone)]
pub struct LoadedMesh {
    pub mesh: Mesh,
    pub report: MeshLoadReport,
    pub format: MeshFormat,
    pub units: Units,
    pub path: PathBuf,
}

/// Performs a lightweight validation pass so downstream stages can assume basic sanity.
pub fn sanitize(mesh: &mut Mesh) -> Result<MeshLoadReport, MeshError> {
    if mesh.face_count() > 0 && mesh.vertex_count() < 3 {
        return Err(MeshError::TooFewVertices(mesh.vertex_count()));
    }

    let mut dropped_faces = 0usize;
    mesh.faces.retain(|face| {
        let valid = face.iter().all(|&idx| (idx as usize) < mesh.vertices.len());
        if !valid {
            dropped_faces += 1;
        }
        valid
    });

    Ok(MeshLoadReport {
        dropped_faces,
        dropped_vertices: 0,
    })
}

pub fn load_mesh(path: impl AsRef<Path>) -> Result<LoadedMesh, MeshIoError> {
    let path = path.as_ref().to_path_buf();
    let lower_ext = path
        .extension()
        .and_then(|ext| ext.to_str())
        .map(|s| s.to_lowercase())
        .unwrap_or_default();

    match lower_ext.as_str() {
        "stl" => load_stl(&path),
        "obj" => load_obj(&path),
        "ply" => load_ply(&path),
        _ => Err(MeshIoError::Unsupported(
            path.extension()
                .and_then(|ext| ext.to_str())
                .unwrap_or("unknown")
                .to_string(),
        )),
    }
}

pub fn load_stl(path: impl AsRef<Path>) -> Result<LoadedMesh, MeshIoError> {
    let bytes = fs::read(&path)?;
    let mut mesh = parse_stl(&bytes)?;
    let report = sanitize(&mut mesh)?;
    Ok(LoadedMesh {
        mesh,
        report,
        format: MeshFormat::Stl,
        units: Units::Unspecified,
        path: path.as_ref().to_path_buf(),
    })
}

pub fn load_obj(path: impl AsRef<Path>) -> Result<LoadedMesh, MeshIoError> {
    let text = fs::read_to_string(&path)?;
    let (mesh, units) = parse_obj(&text)?;
    let mut mesh = mesh;
    let report = sanitize(&mut mesh)?;
    Ok(LoadedMesh {
        mesh,
        report,
        format: MeshFormat::Obj,
        units,
        path: path.as_ref().to_path_buf(),
    })
}

pub fn load_ply(path: impl AsRef<Path>) -> Result<LoadedMesh, MeshIoError> {
    let text = fs::read_to_string(&path)?;
    let (mesh, units) = parse_ply(&text)?;
    let mut mesh = mesh;
    let report = sanitize(&mut mesh)?;
    Ok(LoadedMesh {
        mesh,
        report,
        format: MeshFormat::Ply,
        units,
        path: path.as_ref().to_path_buf(),
    })
}

fn parse_stl(bytes: &[u8]) -> Result<Mesh, MeshIoError> {
    if is_probably_ascii(bytes) {
        parse_ascii_stl(bytes)
    } else {
        parse_binary_stl(bytes)
    }
}

fn parse_ascii_stl(bytes: &[u8]) -> Result<Mesh, MeshIoError> {
    let source = str::from_utf8(bytes)?;
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let mut vertex_lookup = HashMap::new();
    let mut current = Vec::with_capacity(3);

    for line in source.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() || trimmed.starts_with("solid") || trimmed.starts_with("endsolid") {
            continue;
        }

        let mut tokens = trimmed.split_whitespace();
        let keyword = tokens
            .next()
            .ok_or_else(|| MeshIoError::Invalid("unexpected empty line".into()))?;

        match keyword {
            "facet" => {
                current.clear();
            }
            "vertex" => {
                let coords = parse_vertex(tokens.collect::<Vec<_>>().as_slice())?;
                let idx = upsert_vertex(coords, &mut vertex_lookup, &mut vertices);
                current.push(idx);
            }
            "endfacet" => {
                if current.len() == 3 {
                    faces.push([current[0], current[1], current[2]]);
                } else {
                    return Err(MeshIoError::Invalid("facet missing three vertices".into()));
                }
            }
            _ => {}
        }
    }

    if faces.is_empty() {
        return Err(MeshIoError::Invalid("no faces parsed from STL".into()));
    }

    Ok(Mesh { vertices, faces })
}

fn parse_binary_stl(bytes: &[u8]) -> Result<Mesh, MeshIoError> {
    if bytes.len() < 84 {
        return Err(MeshIoError::Invalid("binary STL too small".into()));
    }

    let tri_count = u32::from_le_bytes(bytes[80..84].try_into().unwrap());
    let mut offset = 84usize;
    let mut vertices = Vec::with_capacity(tri_count as usize * 3);
    let mut faces = Vec::with_capacity(tri_count as usize);
    let mut vertex_lookup = HashMap::new();

    for _ in 0..tri_count {
        if offset + 50 > bytes.len() {
            return Err(MeshIoError::Invalid("binary STL truncated".into()));
        }
        offset += 12; // skip normal.
        let mut indices = [0u32; 3];
        for idx in 0..3 {
            let start = offset + idx * 12;
            let point = [
                f32::from_le_bytes(bytes[start..start + 4].try_into().unwrap()),
                f32::from_le_bytes(bytes[start + 4..start + 8].try_into().unwrap()),
                f32::from_le_bytes(bytes[start + 8..start + 12].try_into().unwrap()),
            ];
            indices[idx] = upsert_vertex(point, &mut vertex_lookup, &mut vertices);
        }
        offset += 36;
        faces.push(indices);
        offset += 2; // attribute byte count.
    }

    Ok(Mesh { vertices, faces })
}

fn parse_vertex(parts: &[&str]) -> Result<Point, MeshIoError> {
    if parts.len() < 3 {
        return Err(MeshIoError::Invalid("vertex missing coordinates".into()));
    }

    let parse =
        |s: &str| f32::from_str(s).map_err(|_| MeshIoError::Invalid("invalid float".into()));
    Ok([parse(parts[0])?, parse(parts[1])?, parse(parts[2])?])
}

fn upsert_vertex(
    vertex: Point,
    lookup: &mut HashMap<[u32; 3], u32>,
    vertices: &mut Vec<Point>,
) -> u32 {
    let key = vertex.map(|f| f.to_bits());
    if let Some(&index) = lookup.get(&key) {
        index
    } else {
        let index = vertices.len() as u32;
        vertices.push(vertex);
        lookup.insert(key, index);
        index
    }
}

fn is_probably_ascii(bytes: &[u8]) -> bool {
    bytes.starts_with(b"solid")
        && bytes.contains(&b'\n')
        && bytes.windows(5).any(|w| w.eq(b"facet"))
}

fn parse_obj(source: &str) -> Result<(Mesh, Units), MeshIoError> {
    let mut vertices = Vec::new();
    let mut faces = Vec::new();
    let mut units = Units::Unspecified;

    for line in source.lines() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        if let Some(stripped) = trimmed.strip_prefix('#') {
            if let Some(found) = parse_units_hint(stripped.trim()) {
                units = found;
            }
            continue;
        }

        let mut parts = trimmed.split_whitespace();
        match parts.next() {
            Some("v") => {
                let coords = parse_vertex(parts.collect::<Vec<_>>().as_slice())?;
                vertices.push(coords);
            }
            Some("f") => {
                let face_indices = parts
                    .map(|token| {
                        token
                            .split('/')
                            .next()
                            .ok_or_else(|| MeshIoError::Invalid("invalid face index".into()))
                            .and_then(|idx| {
                                let value = i32::from_str(idx).map_err(|_| {
                                    MeshIoError::Invalid("invalid face index".into())
                                })?;
                                if value <= 0 {
                                    Err(MeshIoError::Invalid(
                                        "negative face indices unsupported".into(),
                                    ))
                                } else {
                                    Ok((value - 1) as u32)
                                }
                            })
                    })
                    .collect::<Result<Vec<_>, _>>()?;

                triangulate_face(&face_indices, &mut faces)?;
            }
            _ => continue,
        }
    }

    if vertices.is_empty() || faces.is_empty() {
        return Err(MeshIoError::Invalid("OBJ missing vertices or faces".into()));
    }

    Ok((Mesh { vertices, faces }, units))
}

fn parse_ply(source: &str) -> Result<(Mesh, Units), MeshIoError> {
    let mut lines = source.lines();
    let first = lines
        .next()
        .ok_or_else(|| MeshIoError::Invalid("empty PLY".into()))?;
    if first.trim() != "ply" {
        return Err(MeshIoError::Invalid("missing ply header".into()));
    }

    let mut vertex_count = 0usize;
    let mut face_count = 0usize;
    let mut reading_header = true;
    let mut units = Units::Unspecified;
    let mut header_lines = Vec::new();

    while let Some(line) = lines.next() {
        let trimmed = line.trim();
        if reading_header {
            if trimmed.starts_with("comment") {
                if let Some(found) = parse_units_hint(trimmed.trim_start_matches("comment").trim())
                {
                    units = found;
                }
            }
            if trimmed.starts_with("element vertex") {
                vertex_count = trimmed
                    .split_whitespace()
                    .last()
                    .and_then(|count| count.parse::<usize>().ok())
                    .ok_or_else(|| MeshIoError::Invalid("bad vertex count".into()))?;
            } else if trimmed.starts_with("element face") {
                face_count = trimmed
                    .split_whitespace()
                    .last()
                    .and_then(|count| count.parse::<usize>().ok())
                    .ok_or_else(|| MeshIoError::Invalid("bad face count".into()))?;
            } else if trimmed == "end_header" {
                reading_header = false;
                break;
            }
            header_lines.push(trimmed.to_string());
        }
    }

    if reading_header {
        return Err(MeshIoError::Invalid("unterminated header".into()));
    }

    let mut vertices = Vec::with_capacity(vertex_count);
    for _ in 0..vertex_count {
        let line = lines
            .next()
            .ok_or_else(|| MeshIoError::Invalid("unexpected EOF in vertices".into()))?;
        let parts: Vec<_> = line.split_whitespace().collect();
        vertices.push(parse_vertex(&parts)?);
    }

    let mut faces = Vec::with_capacity(face_count);
    for _ in 0..face_count {
        let line = lines
            .next()
            .ok_or_else(|| MeshIoError::Invalid("unexpected EOF in faces".into()))?;
        let mut parts = line.split_whitespace();
        let count: usize = parts
            .next()
            .ok_or_else(|| MeshIoError::Invalid("missing face vertex count".into()))?
            .parse()
            .map_err(|_| MeshIoError::Invalid("invalid face vertex count".into()))?;
        let indices = parts
            .take(count)
            .map(|idx| {
                idx.parse::<u32>()
                    .map_err(|_| MeshIoError::Invalid("invalid face index".into()))
            })
            .collect::<Result<Vec<_>, _>>()?;
        triangulate_face(&indices, &mut faces)?;
    }

    Ok((Mesh { vertices, faces }, units))
}

fn triangulate_face(indices: &[u32], faces: &mut Vec<Face>) -> Result<(), MeshIoError> {
    if indices.len() < 3 {
        return Err(MeshIoError::Invalid("face must have >=3 vertices".into()));
    }
    if indices.len() == 3 {
        faces.push([indices[0], indices[1], indices[2]]);
        return Ok(());
    }

    for i in 1..indices.len() - 1 {
        faces.push([indices[0], indices[i], indices[i + 1]]);
    }
    Ok(())
}

fn parse_units_hint(hint: &str) -> Option<Units> {
    let lower = hint.to_lowercase();
    if lower.contains("mm") || lower.contains("millimeter") {
        Some(Units::Millimeters)
    } else if lower.contains("cm") || lower.contains("centimeter") {
        Some(Units::Centimeters)
    } else if lower.contains("inch") || lower.contains("in ") {
        Some(Units::Inches)
    } else if lower.contains("meter") || lower.contains("metre") || lower.contains(" m ") {
        Some(Units::Meters)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn empty_mesh_is_valid_but_empty() {
        let mesh = Mesh::new();
        assert!(mesh.is_empty());
        assert_eq!(mesh.vertex_count(), 0);
        assert_eq!(mesh.face_count(), 0);
    }

    #[test]
    fn sanitize_drops_faces_with_out_of_bounds_indices() {
        let mut mesh = Mesh {
            vertices: vec![[0.0, 0.0, 0.0]; 3],
            faces: vec![[0, 1, 2], [0, 4, 2]],
        };
        let report = sanitize(&mut mesh).expect("sanitization should succeed");
        assert_eq!(mesh.face_count(), 1);
        assert_eq!(report.dropped_faces, 1);
    }

    #[test]
    fn sanitize_rejects_faces_without_enough_vertices() {
        let mut mesh = Mesh {
            vertices: vec![[0.0, 0.0, 0.0]; 2],
            faces: vec![[0, 1, 1]],
        };
        let err = sanitize(&mut mesh).expect_err("should fail due to few vertices");
        match err {
            MeshError::TooFewVertices(count) => assert_eq!(count, 2),
        }
    }

    #[test]
    fn parses_ascii_stl() {
        let stl = b"solid ascii\nfacet normal 0 0 1\n outer loop\n  vertex 0 0 0\n  vertex 1 0 0\n  vertex 0 1 0\n endloop\nendfacet\nendsolid\n";
        let mesh = parse_ascii_stl(stl).expect("parse ascii");
        assert_eq!(mesh.face_count(), 1);
        assert_eq!(mesh.vertex_count(), 3);
    }

    #[test]
    fn parses_binary_stl() {
        let bytes = make_binary_tetra();
        let mesh = parse_binary_stl(&bytes).expect("binary");
        assert_eq!(mesh.face_count(), 4);
    }

    #[test]
    fn load_stl_reads_from_disk() {
        let mut file = tempfile::NamedTempFile::new().expect("temp");
        let data = make_binary_tetra();
        file.write_all(&data).unwrap();
        let loaded = load_stl(file.path()).expect("load");
        assert_eq!(loaded.mesh.face_count(), 4);
    }

    #[test]
    fn parse_obj_file() {
        let obj = r#"
# units: mm
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 1
f 1 2 3
f 1 2 4
f 1 3 4
f 2 3 4
"#;
        let (mesh, units) = parse_obj(obj).expect("obj parsed");
        assert_eq!(mesh.face_count(), 4);
        assert_eq!(units, Units::Millimeters);
    }

    #[test]
    fn parse_ply_file() {
        let ply = r#"ply
format ascii 1.0
comment units mm
element vertex 4
property float x
property float y
property float z
element face 4
property list uchar int vertex_indices
end_header
0 0 0
1 0 0
0 1 0
0 0 1
3 0 1 2
3 0 1 3
3 0 2 3
3 1 2 3
"#;
        let (mesh, units) = parse_ply(ply).expect("ply parsed");
        assert_eq!(mesh.face_count(), 4);
        assert_eq!(units, Units::Millimeters);
    }

    fn make_binary_tetra() -> Vec<u8> {
        let mut data = vec![0u8; 84];
        let tri_count = 4u32;
        data[80..84].copy_from_slice(&tri_count.to_le_bytes());
        let mut tris = Vec::new();
        let vertices = [
            [0.0f32, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ];
        let faces = [[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]];
        for face in faces {
            tris.extend_from_slice(&[0u8; 12]); // normal zeros
            for &vid in face.iter() {
                let v = vertices[vid];
                tris.extend_from_slice(&v[0].to_le_bytes());
                tris.extend_from_slice(&v[1].to_le_bytes());
                tris.extend_from_slice(&v[2].to_le_bytes());
            }
            tris.extend_from_slice(&0u16.to_le_bytes());
        }
        data.extend_from_slice(&tris);
        data
    }
}

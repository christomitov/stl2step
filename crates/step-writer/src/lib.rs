use std::{
    collections::HashMap,
    fs::File,
    io::{self, Write},
    path::Path,
};

use brep::Tolerance;
use mesh_io::{Face, Mesh};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct StepMetadata {
    pub product_name: String,
    pub mode: StepMode,
    pub tolerance: Tolerance,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub enum StepMode {
    Faceted,
    Editable,
}

#[derive(Debug, Error)]
pub enum StepWriterError {
    #[error("mesh is empty; cannot emit STEP")]
    EmptyMesh,
    #[error("io error: {0}")]
    Io(#[from] io::Error),
}

#[derive(Debug, Clone)]
pub struct PlanarPatchInput {
    pub vertex_loop: Vec<u32>,
    pub inner_loops: Vec<Vec<u32>>,
    pub normal: [f32; 3],
}

pub fn build_metadata(
    name: impl Into<String>,
    mode: StepMode,
    tolerance: Tolerance,
) -> StepMetadata {
    StepMetadata {
        product_name: name.into(),
        mode,
        tolerance,
    }
}

pub fn format_product_label(meta: &StepMetadata) -> String {
    format!(
        "{} ({:?}, lin={:.4}mm)",
        meta.product_name, meta.mode, meta.tolerance.linear
    )
}

pub fn write_faceted_step(
    path: &Path,
    metadata: &StepMetadata,
    mesh: &Mesh,
    patches: Option<&[PlanarPatchInput]>,
) -> Result<(), StepWriterError> {
    if mesh.face_count() == 0 || mesh.vertex_count() == 0 {
        return Err(StepWriterError::EmptyMesh);
    }

    let mut builder = StepBuilder::new();
    let product_context = builder.add("PRODUCT_CONTEXT('default','mechanical','design')");
    let sanitized_name = metadata.product_name.replace('\'', "");

    let product_id = builder.add(format!(
        "PRODUCT('{name}','{name}','',(#{}))",
        product_context,
        name = sanitized_name
    ));
    let formation = builder.add(format!(
        "PRODUCT_DEFINITION_FORMATION('','',#{})",
        product_id
    ));
    let pd_context = builder.add("PRODUCT_DEFINITION_CONTEXT('design','PRESENTATION')");
    let product_definition = builder.add(format!(
        "PRODUCT_DEFINITION('','',#{},#{})",
        formation, pd_context
    ));
    let pds = builder.add(format!(
        "PRODUCT_DEFINITION_SHAPE('','',#{})",
        product_definition
    ));

    let geom_ctx = builder.add("GEOMETRIC_REPRESENTATION_CONTEXT(3)");
    let si_unit = builder.add("SI_UNIT(.MILLI.,.METRE.)");
    let global_ctx = builder.add(format!(
        "GLOBAL_UNIT_ASSIGNED_CONTEXT((#{},#{},#{}))",
        si_unit, si_unit, si_unit
    ));
    let mech_ctx = builder.add(format!(
        "MECHANICAL_WITH_MATERIAL_CONTEXT('',#{},#{})",
        geom_ctx, global_ctx
    ));

    let point_ids: Vec<usize> = mesh
        .vertices
        .iter()
        .map(|v| {
            builder.add(format!(
                "CARTESIAN_POINT('',({:.6},{:.6},{:.6}))",
                v[0], v[1], v[2]
            ))
        })
        .collect();

    let vertex_ids: Vec<usize> = point_ids
        .iter()
        .map(|point_id| builder.add(format!("VERTEX_POINT('',#{})", point_id)))
        .collect();

    let shape_rep = if let Some(patch_data) = patches {
        if !patch_data.is_empty() {
            build_planar_brep(
                &sanitized_name,
                mesh,
                patch_data,
                &vertex_ids,
                &point_ids,
                mech_ctx,
                &mut builder,
            )
        } else {
            build_tessellated_representation(
                &sanitized_name,
                mesh.faces.iter(),
                &point_ids,
                mech_ctx,
                &mut builder,
            )
        }
    } else {
        build_tessellated_representation(
            &sanitized_name,
            mesh.faces.iter(),
            &point_ids,
            mech_ctx,
            &mut builder,
        )
    };
    builder.add(format!(
        "SHAPE_DEFINITION_REPRESENTATION(#{},#{})",
        pds, shape_rep
    ));

    let mut file = File::create(path)?;
    write_header(&mut file, metadata)?;
    write_data(&mut file, &builder)?;
    Ok(())
}

fn build_planar_brep(
    name: &str,
    mesh: &Mesh,
    patches: &[PlanarPatchInput],
    vertex_ids: &[usize],
    point_ids: &[usize],
    mech_ctx: usize,
    builder: &mut StepBuilder,
) -> usize {
    let mut edge_builder = EdgeBuilder::new(mesh, vertex_ids, point_ids);
    let mut faces = Vec::new();
    for patch in patches {
        if patch.vertex_loop.len() < 3 {
            continue;
        }
        let Some(outer_loop) = edge_builder.build_edge_loop(&patch.vertex_loop, builder) else {
            continue;
        };
        let mut bounds = Vec::new();
        let outer_bound = builder.add(format!("FACE_OUTER_BOUND('',#{},.T.)", outer_loop));
        bounds.push(outer_bound);

        for hole in &patch.inner_loops {
            if let Some(loop_id) = edge_builder.build_edge_loop(hole, builder) {
                let bound = builder.add(format!("FACE_BOUND('',#{},.F.)", loop_id));
                bounds.push(bound);
            }
        }
        let bounds_refs = bounds
            .iter()
            .map(|id| format!("#{id}"))
            .collect::<Vec<_>>()
            .join(",");
        let plane = build_plane(
            patch,
            point_ids.get(patch.vertex_loop[0] as usize).copied(),
            builder,
        );
        let face = builder.add(format!(
            "ADVANCED_FACE('',({}) ,#{},.T.)",
            bounds_refs, plane
        ));
        faces.push(face);
    }

    if faces.is_empty() {
        return build_tessellated_representation(
            name,
            mesh.faces.iter(),
            point_ids,
            mech_ctx,
            builder,
        );
    }

    let shell = builder.add(format!(
        "CLOSED_SHELL('',({}))",
        faces
            .iter()
            .map(|id| format!("#{id}"))
            .collect::<Vec<_>>()
            .join(",")
    ));
    builder.add(format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('{}',(#{}) ,#{})",
        name, shell, mech_ctx
    ))
}

fn build_plane(
    patch: &PlanarPatchInput,
    point_id: Option<usize>,
    builder: &mut StepBuilder,
) -> usize {
    let origin_id = point_id.unwrap_or_else(|| builder.add("CARTESIAN_POINT('',(0.0,0.0,0.0))"));
    let normal = normalize_vector(patch.normal);
    let tangent = tangent_from_normal(normal);
    let normal_dir = builder.add(format!(
        "DIRECTION('',({:.6},{:.6},{:.6}))",
        normal[0], normal[1], normal[2]
    ));
    let tangent_dir = builder.add(format!(
        "DIRECTION('',({:.6},{:.6},{:.6}))",
        tangent[0], tangent[1], tangent[2]
    ));
    let axis = builder.add(format!(
        "AXIS2_PLACEMENT_3D('',#{},#{},#{})",
        origin_id, normal_dir, tangent_dir
    ));
    builder.add(format!("PLANE('',#{})", axis))
}

fn normalize_vector(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt();
    if len == 0.0 {
        [0.0, 0.0, 1.0]
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn tangent_from_normal(normal: [f32; 3]) -> [f32; 3] {
    let up = if normal[2].abs() < 0.9 {
        [0.0, 0.0, 1.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    normalize_vector([
        up[1] * normal[2] - up[2] * normal[1],
        up[2] * normal[0] - up[0] * normal[2],
        up[0] * normal[1] - up[1] * normal[0],
    ])
}

struct EdgeBuilder<'a> {
    mesh: &'a Mesh,
    vertex_ids: &'a [usize],
    point_ids: &'a [usize],
    cache: HashMap<(u32, u32), usize>,
}

impl<'a> EdgeBuilder<'a> {
    fn new(mesh: &'a Mesh, vertex_ids: &'a [usize], point_ids: &'a [usize]) -> Self {
        Self {
            mesh,
            vertex_ids,
            point_ids,
            cache: HashMap::new(),
        }
    }

    fn build_edge_loop(
        &mut self,
        loop_vertices: &[u32],
        builder: &mut StepBuilder,
    ) -> Option<usize> {
        if loop_vertices.len() < 2 {
            return None;
        }
        let mut vertices = loop_vertices.to_vec();
        if vertices.first() == vertices.last() && vertices.len() > 2 {
            vertices.pop();
        }
        if vertices.len() < 2 {
            return None;
        }
        let mut oriented_edges = Vec::new();
        for window in vertices.windows(2) {
            oriented_edges.push(self.oriented_edge(window[0], window[1], builder));
        }
        oriented_edges.push(self.oriented_edge(*vertices.last().unwrap(), vertices[0], builder));
        if oriented_edges.len() < 3 {
            return None;
        }
        Some(builder.add(format!(
            "EDGE_LOOP('',({}))",
            oriented_edges
                .iter()
                .map(|id| format!("#{id}"))
                .collect::<Vec<_>>()
                .join(",")
        )))
    }

    fn oriented_edge(&mut self, start: u32, end: u32, builder: &mut StepBuilder) -> usize {
        let (edge_id, forward) = self.edge_curve(start, end, builder);
        builder.add(format!(
            "ORIENTED_EDGE('',*,*,#{},{})",
            edge_id,
            if forward { ".T." } else { ".F." }
        ))
    }

    fn edge_curve(&mut self, start: u32, end: u32, builder: &mut StepBuilder) -> (usize, bool) {
        let (min, max, forward) = if start <= end {
            (start, end, true)
        } else {
            (end, start, false)
        };
        let key = (min, max);
        if let Some(&edge_id) = self.cache.get(&key) {
            return (edge_id, forward);
        }
        let start_vertex = self.vertex_ids[min as usize];
        let end_vertex = self.vertex_ids[max as usize];
        let line = self.line_for_edge(min, max, builder);
        let edge_curve = builder.add(format!(
            "EDGE_CURVE('',#{},#{},#{},.T.)",
            start_vertex, end_vertex, line
        ));
        self.cache.insert(key, edge_curve);
        (edge_curve, forward)
    }

    fn line_for_edge(&mut self, start: u32, end: u32, builder: &mut StepBuilder) -> usize {
        let start_point = self.point_ids[start as usize];
        let start_coord = self.mesh.vertices[start as usize];
        let end_coord = self.mesh.vertices[end as usize];
        let raw_dir = [
            end_coord[0] - start_coord[0],
            end_coord[1] - start_coord[1],
            end_coord[2] - start_coord[2],
        ];
        let direction = {
            let len = (raw_dir[0].powi(2) + raw_dir[1].powi(2) + raw_dir[2].powi(2)).sqrt();
            if len < 1e-6 {
                [1.0, 0.0, 0.0]
            } else {
                [raw_dir[0] / len, raw_dir[1] / len, raw_dir[2] / len]
            }
        };
        let dir_id = builder.add(format!(
            "DIRECTION('',({:.6},{:.6},{:.6}))",
            direction[0], direction[1], direction[2]
        ));
        builder.add(format!("LINE('',#{},#{})", start_point, dir_id))
    }
}

fn build_tessellated_representation<'a, I>(
    name: &str,
    faces: I,
    point_ids: &[usize],
    mech_ctx: usize,
    builder: &mut StepBuilder,
) -> usize
where
    I: IntoIterator<Item = &'a Face>,
{
    let point_refs = point_ids
        .iter()
        .map(|id| format!("#{id}"))
        .collect::<Vec<_>>()
        .join(",");
    let tessellated = build_tessellated_face_set(faces, &point_refs, builder);
    builder.add(format!(
        "SHAPE_REPRESENTATION('{}',(#{}) ,#{})",
        name, tessellated, mech_ctx
    ))
}

fn write_header(file: &mut File, metadata: &StepMetadata) -> io::Result<()> {
    writeln!(file, "ISO-10303-21;")?;
    writeln!(file, "HEADER;")?;
    writeln!(
        file,
        "FILE_DESCRIPTION(('Mesh2Brep {mode:?}'),'3;1');",
        mode = metadata.mode
    )?;
    writeln!(
        file,
        "FILE_NAME('{name}','{timestamp}',('Mesh2Brep'),('Mesh2Brep'),'Mesh2Brep','Mesh2Brep','');",
        name = metadata.product_name.replace('\'', ""),
        timestamp = "2025-01-01T00:00:00"
    )?;
    writeln!(file, "FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));")?;
    writeln!(file, "ENDSEC;")?;
    Ok(())
}

fn write_data(file: &mut File, builder: &StepBuilder) -> io::Result<()> {
    writeln!(file, "DATA;")?;
    for line in &builder.lines {
        writeln!(file, "{line}")?;
    }
    writeln!(file, "ENDSEC;")?;
    writeln!(file, "END-ISO-10303-21;")?;
    Ok(())
}

fn build_tessellated_face_set<'a, I>(faces: I, point_refs: &str, builder: &mut StepBuilder) -> usize
where
    I: IntoIterator<Item = &'a Face>,
{
    let face_refs = faces
        .into_iter()
        .map(|f| format!("({},{},{})", f[0] + 1, f[1] + 1, f[2] + 1))
        .collect::<Vec<_>>()
        .join(",");
    builder.add(format!(
        "TRIANGULATED_FACE_SET(({}),$,({}))",
        point_refs, face_refs
    ))
}

struct StepBuilder {
    next_id: usize,
    lines: Vec<String>,
}

impl StepBuilder {
    fn new() -> Self {
        Self {
            next_id: 1,
            lines: Vec::new(),
        }
    }

    fn add(&mut self, body: impl Into<String>) -> usize {
        let id = self.next_id;
        self.lines.push(format!("#{id}={};", body.into()));
        self.next_id += 1;
        id
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn label_includes_mode_and_tolerance() {
        let meta = build_metadata(
            "widget",
            StepMode::Faceted,
            Tolerance {
                linear: 0.05,
                angular_deg: 1.0,
            },
        );
        assert!(format_product_label(&meta).contains("widget"));
        assert!(format_product_label(&meta).contains("Faceted"));
    }

    #[test]
    fn writes_faceted_step_file() {
        let meta = build_metadata(
            "sample",
            StepMode::Faceted,
            Tolerance {
                linear: 0.1,
                angular_deg: 1.0,
            },
        );
        let mesh = Mesh {
            vertices: vec![[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
            faces: vec![[0, 1, 2]],
        };
        let tmp = tempfile::NamedTempFile::new().unwrap();
        write_faceted_step(tmp.path(), &meta, &mesh, None).expect("write step");
        let content = fs::read_to_string(tmp.path()).unwrap();
        assert!(content.contains("ISO-10303-21"));
        assert!(content.contains("TRIANGULATED_FACE_SET"));
    }
}

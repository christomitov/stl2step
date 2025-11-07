use std::{
    fs::File,
    path::{Path, PathBuf},
};

use anyhow::Result;
use brep::{derived_tolerance, Tolerance};
use clap::{Parser, ValueEnum};
use fit::{
    ensure_positive_tolerance, make_ransac_config, recognize_primitives, segment_mesh,
    PrimitiveFitConfig, PrimitiveKind, PrimitiveReport, SegmentationConfig,
};
use mesh_io::{load_mesh, LoadedMesh, MeshFormat, Units};
use mode_a::{
    analyze_planarity, analyze_topology, extract_planar_patches, merge_planar_clusters, PlanarShell,
};
use serde::Serialize;
use step_writer::{build_metadata, StepMode};

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq, Serialize)]
enum Mode {
    Faceted,
    Editable,
}

#[derive(Parser, Debug)]
#[command(
    name = "stl2step",
    author,
    version,
    about = "Convert STL meshes into STEP shells/solids"
)]
struct Cli {
    /// Path to mesh input (currently STL).
    input: PathBuf,

    #[arg(long, value_enum, default_value = "faceted")]
    mode: Mode,

    #[arg(long = "tol", default_value = "0.1")]
    tolerance_mm: f64,

    #[arg(long = "angle-merge", default_value = "2.0")]
    angle_merge_deg: f32,

    #[arg(long = "prim-tol", default_value = "0.1")]
    primitive_tolerance_mm: f32,

    #[arg(short, long, value_name = "STEP")]
    output: Option<PathBuf>,

    #[arg(long)]
    report: Option<PathBuf>,

    #[arg(long = "patches")]
    patches_path: Option<PathBuf>,

    #[arg(long = "features")]
    features_path: Option<PathBuf>,
}

#[derive(Debug, PartialEq)]
struct JobSummary {
    mode: Mode,
    faces: usize,
    tolerance: Tolerance,
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let summary = run(&cli)?;
    println!(
        "Prepared {:?} job with {} faces (tol {:.4} mm)",
        summary.mode, summary.faces, summary.tolerance.linear
    );
    Ok(())
}

fn run(cli: &Cli) -> Result<JobSummary> {
    ensure_positive_tolerance(cli.tolerance_mm as f32)?;
    ensure_positive_tolerance(cli.primitive_tolerance_mm)?;

    let LoadedMesh {
        mut mesh,
        report,
        format,
        units,
        ..
    } = load_input(cli)?;
    let tol_f32 = cli.tolerance_mm as f32;
    let heal = mode_a::heal_vertices(&mut mesh, tol_f32)?;
    let stats = geom_core::estimate_stats(&mesh)?;
    let _config = make_ransac_config(&mesh)?;
    let tolerance = derived_tolerance(&stats, Some(cli.tolerance_mm));
    let product_name = cli
        .input
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("mesh");
    let meta = build_metadata(
        product_name,
        match cli.mode {
            Mode::Faceted => StepMode::Faceted,
            Mode::Editable => StepMode::Editable,
        },
        tolerance,
    );

    let planar = analyze_planarity(&mesh, cli.angle_merge_deg)?;
    let planar_shell = extract_planar_patches(&mesh, &planar, tol_f32);
    let merged_mesh = merge_planar_clusters(&mesh, &planar, tol_f32);
    let segments = segment_mesh(&mesh, &SegmentationConfig::default())?;
    let prim_config = PrimitiveFitConfig::with_rms_threshold(cli.primitive_tolerance_mm)?;
    let primitive_report = recognize_primitives(&mesh, &segments.segments, &prim_config);
    let patch_inputs: Vec<_> = planar_shell
        .patches
        .iter()
        .filter(|patch| patch.outer_boundary.len() >= 3)
        .map(|patch| step_writer::PlanarPatchInput {
            vertex_loop: patch.outer_boundary.clone(),
            inner_loops: patch.inner_boundaries.clone(),
            normal: patch.normal,
        })
        .collect();
    let topology = analyze_topology(&merged_mesh);
    let planar_cluster_count = planar.cluster_count();
    let planar_largest = planar.largest_cluster_size();
    let qa = QaReport {
        mode: cli.mode,
        vertices: mesh.vertex_count(),
        faces: mesh.face_count(),
        merged_faces: merged_mesh.face_count(),
        healed_vertices: heal.merged_vertices,
        dropped_faces: report.dropped_faces,
        format,
        units,
        tolerance_mm: meta.tolerance.linear,
        watertight: topology.is_watertight,
        boundary_edges: topology.boundary_edges,
        non_manifold_edges: topology.non_manifold_edges,
        planar_clusters: planar_cluster_count,
        planar_largest: planar_largest,
        planar_patches: planar_shell.patches.len(),
        primitives: PrimitiveQaSummary {
            total: primitive_report.primitives.len(),
            planes: primitive_report.count_kind(PrimitiveKind::Plane),
            cylinders: primitive_report.count_kind(PrimitiveKind::Cylinder),
            spheres: primitive_report.count_kind(PrimitiveKind::Sphere),
        },
    };

    if let Some(path) = &cli.report {
        write_report(path, &qa)?;
    }
    if let Some(path) = &cli.patches_path {
        write_patches(path, &planar_shell)?;
    }
    if let Some(path) = &cli.features_path {
        write_features(path, &primitive_report)?;
    }

    let output_path = resolve_output_path(cli);
    let patch_slice = if patch_inputs.is_empty() {
        None
    } else {
        Some(patch_inputs.as_slice())
    };
    step_writer::write_faceted_step(&output_path, &meta, &merged_mesh, patch_slice)?;

    Ok(JobSummary {
        mode: cli.mode,
        faces: mesh.face_count(),
        tolerance: meta.tolerance,
    })
}

fn load_input(cli: &Cli) -> Result<LoadedMesh> {
    load_mesh_inner(&cli.input).map_err(|err| err.into())
}

fn load_mesh_inner(path: &Path) -> Result<LoadedMesh, mesh_io::MeshIoError> {
    load_mesh(path)
}

fn write_report(path: &Path, report: &QaReport) -> Result<()> {
    let file = File::create(path)?;
    serde_json::to_writer_pretty(file, report)?;
    Ok(())
}

fn write_patches(path: &Path, shell: &PlanarShell) -> Result<()> {
    let file = File::create(path)?;
    serde_json::to_writer_pretty(file, shell)?;
    Ok(())
}

fn write_features(path: &Path, report: &PrimitiveReport) -> Result<()> {
    let file = File::create(path)?;
    serde_json::to_writer_pretty(file, report)?;
    Ok(())
}

#[derive(Debug, Serialize)]
struct QaReport {
    mode: Mode,
    vertices: usize,
    faces: usize,
    merged_faces: usize,
    healed_vertices: usize,
    dropped_faces: usize,
    #[serde(rename = "source_format")]
    format: MeshFormat,
    units: Units,
    tolerance_mm: f64,
    watertight: bool,
    boundary_edges: usize,
    non_manifold_edges: usize,
    planar_clusters: usize,
    planar_largest: usize,
    planar_patches: usize,
    primitives: PrimitiveQaSummary,
}

#[derive(Debug, Serialize)]
struct PrimitiveQaSummary {
    total: usize,
    planes: usize,
    cylinders: usize,
    spheres: usize,
}

fn resolve_output_path(cli: &Cli) -> PathBuf {
    if let Some(path) = &cli.output {
        return path.clone();
    }

    let mut path = cli.input.clone();
    path.set_extension("step");
    path
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_io::{MeshFormat, Units};
    use std::fs;

    #[test]
    fn cli_parses_defaults() {
        let asset = sample_asset();
        let cli = Cli::parse_from(["stl2step", asset.to_str().unwrap()]);
        assert_eq!(cli.mode, Mode::Faceted);
        assert_eq!(cli.tolerance_mm, 0.1);
        assert_eq!(cli.angle_merge_deg, 2.0);
        assert_eq!(cli.primitive_tolerance_mm, 0.1);
        assert!(cli.output.is_none());
        assert!(cli.patches_path.is_none());
        assert!(cli.features_path.is_none());
        let resolved = resolve_output_path(&cli);
        assert_eq!(
            resolved,
            asset.with_extension("step"),
            "default output should be input with .step extension"
        );
    }

    #[test]
    fn run_returns_summary() {
        let output = tempfile::NamedTempFile::new().unwrap();
        let cli = Cli {
            input: sample_asset(),
            mode: Mode::Editable,
            tolerance_mm: 0.05,
            primitive_tolerance_mm: 0.05,
            output: Some(output.path().to_path_buf()),
            angle_merge_deg: 2.0,
            report: None,
            patches_path: None,
            features_path: None,
        };
        let summary = run(&cli).expect("stub run succeeds");
        assert_eq!(summary.mode, Mode::Editable);
        assert!(summary.faces >= 4);
        assert!(summary.tolerance.linear > 0.0);
        drop(output);
    }

    #[test]
    fn report_writes_json() {
        let temp = tempfile::NamedTempFile::new().unwrap();
        let report = QaReport {
            mode: Mode::Faceted,
            vertices: 4,
            faces: 4,
            merged_faces: 1,
            healed_vertices: 0,
            dropped_faces: 0,
            format: MeshFormat::Stl,
            units: Units::Millimeters,
            tolerance_mm: 0.1,
            watertight: true,
            boundary_edges: 0,
            non_manifold_edges: 0,
            planar_clusters: 4,
            planar_largest: 1,
            planar_patches: 4,
            primitives: PrimitiveQaSummary {
                total: 1,
                planes: 1,
                cylinders: 0,
                spheres: 0,
            },
        };
        write_report(temp.path(), &report).unwrap();
        let json = fs::read_to_string(temp.path()).unwrap();
        assert!(json.contains("\"faces\""));
    }

    fn sample_asset() -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("assets")
            .join("samples")
            .join("tetrahedron.stl")
    }
}

use std::{
    env,
    fs::{self, File},
    path::{Path, PathBuf},
};

use anyhow::Result;
use brep::{derived_tolerance, Tolerance};
use clap::{Parser, ValueEnum};
use fit::{
    ensure_positive_tolerance, fit_freeform_patches, make_ransac_config, recognize_primitives,
    segment_mesh, FreeformFitConfig, FreeformPatchFit, FreeformPatchInput, PrimitiveFitConfig,
    PrimitiveKind, PrimitiveReport, SegmentationConfig,
};
use mesh_io::{load_mesh, LoadedMesh, MeshFormat, Units};
use mode_a::{
    analyze_planarity, analyze_topology, extract_planar_patches, merge_planar_clusters, PlanarShell,
};
use occt_bridge;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use step_writer::{build_metadata, StepMode};
use toml::Value as TomlValue;

const CONFIG_DIR: &str = ".m2b";
const CONFIG_FILE: &str = "config.toml";

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

    #[arg(long = "surf-tol", default_value = "0.25")]
    surf_tolerance_mm: f32,

    #[arg(long = "max-degree", default_value = "3")]
    max_degree: u32,

    #[arg(long = "max-patch-cp", default_value = "12x12")]
    max_patch_cp: String,

    #[arg(short, long, value_name = "STEP")]
    output: Option<PathBuf>,

    #[arg(long)]
    report: Option<PathBuf>,

    #[arg(long = "patches")]
    patches_path: Option<PathBuf>,

    #[arg(long = "features")]
    features_path: Option<PathBuf>,

    #[arg(long = "preview")]
    preview_path: Option<PathBuf>,

    #[arg(long = "freeform")]
    freeform_path: Option<PathBuf>,

    #[arg(long = "pro-sew")]
    pro_sew: bool,

    #[arg(long = "telemetry", value_enum, default_value = "auto")]
    telemetry: TelemetryPreference,
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

    let cfg = load_user_config();
    let telemetry_opt_in = resolve_telemetry(cli.telemetry, cfg.telemetry_opt_in);
    if cli.pro_sew {
        occt_bridge::ensure_enabled()?;
    }

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
    let control_grid = parse_control_grid(&cli.max_patch_cp).unwrap_or((12, 12));
    let freeform_config = FreeformFitConfig {
        surf_tolerance_mm: cli.surf_tolerance_mm.max(1e-3),
        max_degree: cli.max_degree.max(1),
        max_control: control_grid,
    };
    let freeform_inputs: Vec<_> = planar_shell
        .patches
        .iter()
        .map(|patch| FreeformPatchInput {
            cluster_index: patch.cluster_index,
            outer_loop: patch.outer_boundary.clone(),
            inner_loops: patch.inner_boundaries.clone(),
        })
        .collect();
    let freeform_report = fit_freeform_patches(&mesh, &freeform_inputs, &freeform_config);
    let freeform_lookup: HashMap<usize, &FreeformPatchFit> = freeform_report
        .iter()
        .map(|fit| (fit.cluster_index, fit))
        .collect();
    let planar_error_lookup: HashMap<usize, f32> = planar_shell
        .patches
        .iter()
        .map(|patch| (patch.cluster_index, patch_deviation(&mesh, patch)))
        .collect();
    let patch_inputs: Vec<_> = planar_shell
        .patches
        .iter()
        .filter(|patch| patch.outer_boundary.len() >= 3)
        .map(|patch| step_writer::PlanarPatchInput {
            vertex_loop: patch.outer_boundary.clone(),
            inner_loops: patch.inner_boundaries.clone(),
            normal: patch.normal,
            max_error_mm: planar_error_lookup.get(&patch.cluster_index).copied(),
            freeform_error_mm: freeform_lookup
                .get(&patch.cluster_index)
                .map(|fit| fit.max_error_mm),
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
            cones: primitive_report.count_kind(PrimitiveKind::Cone),
            tori: primitive_report.count_kind(PrimitiveKind::Torus),
        },
        freeform: summarize_freeform(&freeform_report),
        telemetry: telemetry_opt_in,
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
    if let Some(path) = &cli.preview_path {
        let preview = build_preview_bundle(&mesh, &planar_shell, cli.mode, &freeform_report);
        write_preview(path, &preview)?;
    }
    if let Some(path) = &cli.freeform_path {
        write_freeform(path, &freeform_report)?;
    }

    maybe_store_telemetry(cli.telemetry);

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

fn write_preview(path: &Path, preview: &PreviewBundle) -> Result<()> {
    let file = File::create(path)?;
    serde_json::to_writer_pretty(file, preview)?;
    Ok(())
}

fn write_freeform(path: &Path, fits: &[FreeformPatchFit]) -> Result<()> {
    let file = File::create(path)?;
    serde_json::to_writer_pretty(file, fits)?;
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
    freeform: FreeformQaSummary,
    telemetry: bool,
}

#[derive(Debug, Serialize)]
struct PrimitiveQaSummary {
    total: usize,
    planes: usize,
    cylinders: usize,
    spheres: usize,
    cones: usize,
    tori: usize,
}

#[derive(Debug, Serialize)]
struct FreeformQaSummary {
    patches: usize,
    within_tolerance: usize,
    max_error_mm: f32,
}

#[derive(Debug, Default, Serialize, Deserialize)]
struct UserConfig {
    telemetry_opt_in: Option<bool>,
}

fn summarize_freeform(freeform: &[FreeformPatchFit]) -> FreeformQaSummary {
    if freeform.is_empty() {
        return FreeformQaSummary {
            patches: 0,
            within_tolerance: 0,
            max_error_mm: 0.0,
        };
    }
    let patches = freeform.len();
    let within = freeform.iter().filter(|fit| fit.within_tolerance).count();
    let max_error = freeform
        .iter()
        .map(|fit| fit.max_error_mm)
        .fold(0.0f32, |acc, e| acc.max(e));
    FreeformQaSummary {
        patches,
        within_tolerance: within,
        max_error_mm: max_error,
    }
}

fn build_preview_bundle(
    mesh: &mesh_io::Mesh,
    shell: &PlanarShell,
    mode: Mode,
    freeform: &[FreeformPatchFit],
) -> PreviewBundle {
    let bbox = compute_bbox(mesh);
    let mut patches = Vec::new();
    let mut max_error = 0.0f32;
    let freeform_map: HashMap<usize, &FreeformPatchFit> = freeform
        .iter()
        .map(|fit| (fit.cluster_index, fit))
        .collect();
    for patch in &shell.patches {
        let mut error = patch_deviation(mesh, patch);
        let mut freeform_err = 0.0f32;
        if let Some(fit) = freeform_map.get(&patch.cluster_index) {
            freeform_err = fit.max_error_mm;
            error = error.max(freeform_err);
        }
        if error > max_error {
            max_error = error;
        }
        patches.push(PreviewPatch {
            cluster_index: patch.cluster_index,
            vertex_count: patch.outer_boundary.len(),
            inner_loops: patch.inner_boundaries.len(),
            max_error_mm: error,
            freeform_error_mm: freeform_err,
        });
    }
    PreviewBundle {
        mode,
        bbox,
        max_point_error_mm: max_error,
        patches,
    }
}

fn compute_bbox(mesh: &mesh_io::Mesh) -> BoundingBox {
    let mut min = [f32::INFINITY; 3];
    let mut max = [f32::NEG_INFINITY; 3];
    for v in &mesh.vertices {
        for i in 0..3 {
            if v[i] < min[i] {
                min[i] = v[i];
            }
            if v[i] > max[i] {
                max[i] = v[i];
            }
        }
    }
    if mesh.vertices.is_empty() {
        min = [0.0; 3];
        max = [0.0; 3];
    }
    BoundingBox { min, max }
}

fn patch_deviation(mesh: &mesh_io::Mesh, patch: &mode_a::PlanarPatch) -> f32 {
    if patch.outer_boundary.len() < 3 {
        return 0.0;
    }
    let normal = normalize(patch.normal);
    let origin = mesh.vertices[*patch.outer_boundary.first().unwrap() as usize];
    let mut max_error = 0.0f32;
    for &idx in patch
        .outer_boundary
        .iter()
        .chain(patch.inner_boundaries.iter().flatten())
    {
        if let Some(point) = mesh.vertices.get(idx as usize) {
            let delta = sub(*point, origin);
            let proj = dot(delta, normal).abs();
            if proj > max_error {
                max_error = proj;
            }
        }
    }
    max_error
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt();
    if len == 0.0 {
        v
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn load_user_config() -> UserConfig {
    let path = default_config_path();
    if let Some(path) = path {
        if let Ok(text) = std::fs::read_to_string(&path) {
            if let Ok(value) = text.parse::<TomlValue>() {
                if let Ok(cfg) = value.try_into() {
                    return cfg;
                }
            }
        }
    }
    UserConfig::default()
}

fn default_config_path() -> Option<PathBuf> {
    if let Ok(dir) = env::var("M2B_CONFIG_HOME") {
        return Some(PathBuf::from(dir).join(CONFIG_FILE));
    }
    dirs::home_dir().map(|home| home.join(CONFIG_DIR).join(CONFIG_FILE))
}

fn resolve_telemetry(preference: TelemetryPreference, config_opt_in: Option<bool>) -> bool {
    match preference {
        TelemetryPreference::On => true,
        TelemetryPreference::Off => false,
        TelemetryPreference::Auto => config_opt_in.unwrap_or(false),
    }
}

#[derive(Debug, Serialize)]
struct PreviewBundle {
    mode: Mode,
    bbox: BoundingBox,
    max_point_error_mm: f32,
    patches: Vec<PreviewPatch>,
}

#[derive(Debug, Serialize)]
struct BoundingBox {
    min: [f32; 3],
    max: [f32; 3],
}

#[derive(Debug, Serialize)]
struct PreviewPatch {
    cluster_index: usize,
    vertex_count: usize,
    inner_loops: usize,
    max_error_mm: f32,
    freeform_error_mm: f32,
}

#[derive(Debug, Clone, Copy, ValueEnum, PartialEq, Eq, Serialize)]
enum TelemetryPreference {
    Auto,
    On,
    Off,
}

fn resolve_output_path(cli: &Cli) -> PathBuf {
    if let Some(path) = &cli.output {
        return path.clone();
    }

    let mut path = cli.input.clone();
    path.set_extension("step");
    path
}

fn parse_control_grid(spec: &str) -> Option<(u32, u32)> {
    let parts: Vec<_> = spec.split('x').collect();
    if parts.len() != 2 {
        return None;
    }
    let u = parts[0].trim().parse::<u32>().ok()?;
    let v = parts[1].trim().parse::<u32>().ok()?;
    Some((u.max(1), v.max(1)))
}

fn maybe_store_telemetry(preference: TelemetryPreference) {
    if matches!(preference, TelemetryPreference::Auto) {
        return;
    }
    let mut cfg = load_user_config();
    cfg.telemetry_opt_in = Some(matches!(preference, TelemetryPreference::On));
    let _ = store_user_config(&cfg);
}

fn store_user_config(cfg: &UserConfig) -> std::io::Result<()> {
    if let Some(path) = default_config_path() {
        if let Some(dir) = path.parent() {
            fs::create_dir_all(dir)?;
        }
        let toml = toml::to_string_pretty(cfg).unwrap_or_default();
        if !toml.is_empty() {
            fs::write(path, toml)?;
        }
    }
    Ok(())
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
        assert_eq!(cli.surf_tolerance_mm, 0.25);
        assert_eq!(cli.max_degree, 3);
        assert_eq!(cli.max_patch_cp, "12x12");
        assert!(cli.output.is_none());
        assert!(cli.patches_path.is_none());
        assert!(cli.features_path.is_none());
        assert!(cli.preview_path.is_none());
        assert!(cli.freeform_path.is_none());
        assert!(!cli.pro_sew);
        assert_eq!(cli.telemetry, TelemetryPreference::Auto);
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
        let cfg_dir = tempfile::tempdir().unwrap();
        env::set_var("M2B_CONFIG_HOME", cfg_dir.path());
        let cli = Cli {
            input: sample_asset(),
            mode: Mode::Editable,
            tolerance_mm: 0.05,
            primitive_tolerance_mm: 0.05,
            surf_tolerance_mm: 0.1,
            max_degree: 3,
            max_patch_cp: "8x8".into(),
            output: Some(output.path().to_path_buf()),
            angle_merge_deg: 2.0,
            report: None,
            patches_path: None,
            features_path: None,
            preview_path: None,
            freeform_path: None,
            pro_sew: false,
            telemetry: TelemetryPreference::Auto,
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
                cones: 0,
                tori: 0,
            },
            freeform: FreeformQaSummary {
                patches: 1,
                within_tolerance: 1,
                max_error_mm: 0.0,
            },
            telemetry: false,
        };
        write_report(temp.path(), &report).unwrap();
        let json = fs::read_to_string(temp.path()).unwrap();
        assert!(json.contains("\"faces\""));
    }

    #[test]
    fn preview_bundle_serializes() {
        let mesh = mesh_io::Mesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            faces: vec![[0, 1, 2], [0, 2, 3]],
        };
        let patch = mode_a::PlanarPatch {
            cluster_index: 0,
            outer_boundary: vec![0, 1, 2, 3],
            inner_boundaries: Vec::new(),
            normal: [0.0, 0.0, 1.0],
            area: 1.0,
        };
        let shell = PlanarShell {
            patches: vec![patch],
        };
        let freeform = vec![FreeformPatchFit {
            cluster_index: 0,
            rms_error_mm: 0.0,
            max_error_mm: 0.0,
            within_tolerance: true,
            suggested_degree: 3,
            suggested_control: (12, 12),
            outer_loop: vec![0, 1, 2, 3],
            inner_loops: Vec::new(),
        }];
        let preview = build_preview_bundle(&mesh, &shell, Mode::Faceted, &freeform);
        assert_eq!(preview.patches.len(), 1);
        assert!(preview.max_point_error_mm <= 1e-6);
    }

    #[test]
    fn freeform_dump_succeeds() {
        let temp = tempfile::NamedTempFile::new().unwrap();
        let fits = vec![FreeformPatchFit {
            cluster_index: 0,
            rms_error_mm: 0.05,
            max_error_mm: 0.1,
            within_tolerance: true,
            suggested_degree: 3,
            suggested_control: (12, 12),
            outer_loop: vec![0, 1, 2],
            inner_loops: vec![],
        }];
        write_freeform(temp.path(), &fits).unwrap();
        let json = fs::read_to_string(temp.path()).unwrap();
        assert!(json.contains("cluster_index"));
    }

    #[test]
    fn telemetry_resolution_respects_cli_override() {
        assert!(resolve_telemetry(TelemetryPreference::On, Some(false)));
        assert!(!resolve_telemetry(TelemetryPreference::Off, Some(true)));
        assert!(!resolve_telemetry(TelemetryPreference::Auto, None));
        assert!(resolve_telemetry(TelemetryPreference::Auto, Some(true)));
    }

    #[test]
    fn control_grid_parser_handles_invalid_input() {
        assert_eq!(parse_control_grid("8x10"), Some((8, 10)));
        assert!(parse_control_grid("oops").is_none());
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

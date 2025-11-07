use std::path::PathBuf;

use assert_cmd::cargo::cargo_bin_cmd;
use tempfile::{tempdir, NamedTempFile};

#[test]
fn converts_sample_meshes_in_multiple_formats() -> Result<(), Box<dyn std::error::Error>> {
    for ext in ["stl", "obj", "ply"] {
        run_conversion_for(ext)?;
    }
    Ok(())
}

fn asset_path(name: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("assets")
        .join("samples")
        .join(name)
}

fn run_conversion_for(ext: &str) -> Result<(), Box<dyn std::error::Error>> {
    let asset = asset_path(&format!("tetrahedron.{ext}"));
    let report = NamedTempFile::new()?;
    let patches = NamedTempFile::new()?;
    let features = NamedTempFile::new()?;
    let preview = NamedTempFile::new()?;
    let freeform = NamedTempFile::new()?;
    let working_dir = tempdir()?;
    let config_home = tempdir()?;
    let input_path = working_dir.path().join(format!("part.{ext}"));
    std::fs::copy(asset, &input_path)?;

    let mut cmd = cargo_bin_cmd!("stl2step");
    cmd.arg(input_path.to_str().unwrap())
        .arg("--mode")
        .arg("faceted")
        .arg("--report")
        .arg(report.path().to_str().unwrap())
        .arg("--patches")
        .arg(patches.path().to_str().unwrap())
        .arg("--features")
        .arg(features.path().to_str().unwrap())
        .arg("--preview")
        .arg(preview.path().to_str().unwrap())
        .arg("--freeform")
        .arg(freeform.path().to_str().unwrap())
        .env(
            "M2B_CONFIG_HOME",
            config_home.path().to_str().expect("path utf8"),
        );

    cmd.assert().success();

    let json = std::fs::read_to_string(report.path())?;
    assert!(json.contains("\"faces\""), "report missing faces for {ext}");
    let default_output = input_path.with_extension("step");
    let step_contents = std::fs::read_to_string(default_output)?;
    assert!(
        step_contents.contains("ISO-10303-21"),
        "STEP file missing header for {ext}"
    );
    assert!(
        step_contents.contains("TRIANGULATED_FACE_SET")
            || step_contents.contains("FACETED_BREP")
            || step_contents.contains("ADVANCED_BREP_SHAPE_REPRESENTATION"),
        "STEP file missing tessellation or brep for {ext}"
    );

    let patches_json = std::fs::read_to_string(patches.path())?;
    assert!(
        patches_json.contains("patches"),
        "patches dump missing for {ext}"
    );
    let features_json = std::fs::read_to_string(features.path())?;
    assert!(
        features_json.contains("primitives"),
        "features dump missing for {ext}"
    );
    let preview_json = std::fs::read_to_string(preview.path())?;
    assert!(
        preview_json.contains("max_point_error_mm"),
        "preview dump missing deviations for {ext}"
    );
    let freeform_json = std::fs::read_to_string(freeform.path())?;
    assert!(freeform_json.contains("rms_error_mm"));

    Ok(())
}

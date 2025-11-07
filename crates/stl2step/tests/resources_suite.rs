use assert_cmd::cargo::cargo_bin_cmd;
use serde_json::Value;
use std::path::PathBuf;

#[test]
fn stl_resources_run_cleanly() -> Result<(), Box<dyn std::error::Error>> {
    for name in [
        "simple_poly",
        "hole_poly",
        "multi_poly",
        "multi_hole_multi_poly",
    ] {
        let stl = resource(name, "stl");
        let report = tempfile::NamedTempFile::new()?;
        let preview = tempfile::NamedTempFile::new()?;
        let features = tempfile::NamedTempFile::new()?;
        let config_home = tempfile::tempdir()?;
        let mut cmd = cargo_bin_cmd!("stl2step");
        cmd.arg(stl.to_str().unwrap())
            .arg("--report")
            .arg(report.path().to_str().unwrap())
            .arg("--preview")
            .arg(preview.path().to_str().unwrap())
            .arg("--features")
            .arg(features.path().to_str().unwrap())
            .env(
                "M2B_CONFIG_HOME",
                config_home.path().to_str().expect("path utf8"),
            );
        cmd.assert().success().stderr("");
        let report_contents = std::fs::read_to_string(report.path())?;
        let json: Value = serde_json::from_str(&report_contents)?;
        assert!(
            json["watertight"].as_bool().unwrap_or(false),
            "fixture {name} should sew into a watertight shell"
        );
        assert!(json["primitives"]["total"].as_u64().is_some());
        let preview_json = std::fs::read_to_string(preview.path())?;
        assert!(preview_json.contains("max_point_error_mm"));
        let feature_json = std::fs::read_to_string(features.path())?;
        assert!(feature_json.contains("primitives"));
    }
    Ok(())
}

fn resource(name: &str, ext: &str) -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("..")
        .join("tests")
        .join("resources")
        .join(format!("{name}.{ext}"))
}

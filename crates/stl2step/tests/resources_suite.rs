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
        let mut cmd = cargo_bin_cmd!("stl2step");
        cmd.arg(stl.to_str().unwrap())
            .arg("--report")
            .arg(report.path().to_str().unwrap());
        cmd.assert().success().stderr("");
        let report_contents = std::fs::read_to_string(report.path())?;
        let json: Value = serde_json::from_str(&report_contents)?;
        assert!(
            json["watertight"].as_bool().unwrap_or(false),
            "fixture {name} should sew into a watertight shell"
        );
        assert!(json["primitives"]["total"].as_u64().is_some());
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

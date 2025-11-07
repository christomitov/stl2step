# Packaging & Telemetry

## Installers

| Platform | Artifact | Contents |
| --- | --- | --- |
| macOS | `.dmg` with universal binary | CLI + optional `occt.bundle`; signed and notarized. |
| Windows | `.msi` | CLI, service wrapper, VC++ runtime, optional OCCT DLL pack. |
| Linux | `.deb` + tarball | CLI + systemd unit template; looks for OCCT libs if installed. |

All installers place configs in `$HOME/.m2b/` (override via `M2B_CONFIG`) and respect `M2B_TEMP` for working files.

## Runtime Layout

- `~/.m2b/config.toml`: defaults (units, tolerances, telemetry opt-in, OCCT path).
- `~/.m2b/cache/`: downloaded bundles (viewer shaders, OCCT pack).
- `~/.m2b/jobs/`: job checkpoints (`job.partial`) enabling resume.

## Crash Safety

- Long jobs periodically sync checkpoints; resume via `m2b resume path/to/job.partial`.
- On hard failure, write stack metadata + reproduction args to `crash-YYYYMMDD-HHMMSS.json`.
- CLI suggests running built-in validator before rerunning conversions.

## Telemetry Policy

- Disabled by default; opt-in prompt with clear disclosure.
- When enabled, send anonymized metrics (tri count, runtime per stage, failure codes) over HTTPS; adhere to offline mode when `M2B_OFFLINE=1`.
- Provide `m2b telemetry on|off|status` command for user control.


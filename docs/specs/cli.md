# CLI & UX

## Command Surface (MVP)

```
stl2step INPUT.stl
    --mode faceted|editable
    --units mm|cm|m|in|auto
    --angle-merge <deg>          # Mode A coplanar merge threshold
    --tol <mm>                   # Global sewing tolerance
    --prim-tol <mm>              # Primitive RMS threshold
    --surf-tol <mm>              # Freeform RMS target
    --max-degree 5
    --max-patch-cp 12x12
    --repair                     # Attempt hole filling / small component removal
    --pro-sew                    # Enable OCCT bridge when installed
    --preview preview.json       # Emit preview bundle used by viewer/deviation tooling
    --report qa.json
    --patches patches.json         # optional planar patch dump
    --features primitives.json     # optional primitive metadata dump
    --freeform freeform.json       # optional freeform/trimmed surface diagnostics
    --pro-sew                      # hand sewing to OCCT bridge when available
    --telemetry auto|on|off        # override config at ~/.m2b/config.toml
    --output output.step         # optional; defaults to INPUT with .step extension
```

### Behavioral Notes

- Diagnostics default to JSON-on-stdout plus human-readable summaries unless `--quiet`.
- Non-zero exit codes include structured remediation hints (e.g., suggest `--repair`).
- `stl2step daemon` exposes local REST/gRPC endpoints for job submission; same flags apply per job payload.
- `stl2step preview job.json` (later) launches a minimal viewer for deviation heatmaps.

### UX Guardrails

- Ask for unit confirmation when not provided; remember per-project defaults in `$HOME/.m2b/config.toml`.
- Provide progress bars for long-running stages (ingest, fitting, sewing, export).
- After each completed stage, append/refresh a checkpoint file to support `stl2step resume job.partial`.
- Preview bundles capture bbox + max deviations per patch; configuration lives under `$HOME/.m2b/config.toml` so telemetry and other UX toggles persist between runs.

# Product & Modes

## 1. Goals

- **Primary:** Convert watertight or nearly-watertight meshes (20k–2M triangles) into single-body STEP files that open quickly (Mode A) and, where possible, into analytic/NURBS B-reps suitable for CAD editing (Mode B).
- **Secondary:** Provide QA reports (tolerances, deviations, analytic coverage) and optional previews so users can trust the conversion.
- **Out of scope:** Fully organic scans, perfect design-intent reconstruction, or cloud-only workflows. Offline-first native experience is required.

## 2. Target Platforms & Distribution

| Platform | Binary | Notes |
| --- | --- | --- |
| macOS 12+ (Intel + Apple Silicon) | universal `.app`/CLI | Notarized; ships pure Rust core; optional `occt.bundle`. |
| Windows 10/11 x64 | MSI | Bundles VC++ runtime plus optional OCCT DLL pack. |
| Ubuntu 22.04+ / generic Linux | `.deb` + tarball | Dynamically links OCCT when installed; no root required. |

Single binary modes:
- **CLI:** `m2b convert part.stl ...`
- **Daemon:** `m2b daemon --listen 127.0.0.1:9000` for local/batch queues.
- **Viewer (later milestone):** `m2b preview job.json` for deviation overlays.

## 3. High-Level Modes

| Mode | Output | Use Case | Success Criteria |
| --- | --- | --- | --- |
| **Mode A – Faceted** | Sewn `FacetedBrep` or planar-merged `ManifoldSolidBrep`. | Open STL as a single body without thousands of components. | Single body, face count ≪ triangle count, opens fast in Fusion/SolidWorks/SolidEdge. |
| **Mode B – Editable** | Hybrid analytic/NURBS STEP (AP214/AP242). | Reverse-engineer mechanical parts for editing. | Recognized primitives (planes/cylinders/cones/spheres/tori), trimmed NURBS elsewhere, tolerances met, editable dimensions. |

Fallback: if Mode B fails tolerance or topology validation, return Mode A plus diagnostics and QA artifacts.


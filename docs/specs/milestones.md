# Milestones & Deliverables

| Milestone | Scope | Duration | Acceptance |
| --- | --- | --- | --- |
| **M0 – Scaffold** | Workspace setup, CLI skeleton, sample assets, CI baseline. | Week 1 | `cargo test` green on macOS/Linux/Windows; sample meshes stored; docs published. |
| **M1 – Mode A Ship** | Ingest/repair, coplanar merge, sewing, STEP export + QA report, packaging draft. | Weeks 2–4 | Converts reference meshes into single-body STEP, face count reduced, QA JSON emitted. |
| **M2 – Primitive Recognition** | Segmentation, plane/cylinder/cone/sphere/torus fitting, hybrid export, metadata JSON. | Weeks 5–8 | ≥95% precision on benchmark primitives; outputs recognized feature metadata. |
| **M3 – Freeform & Trimming** | NURBS fitting, trimming, improved sewing, deviation report, optional viewer preview. | Weeks 9–14 | Mixed primitive/freeform benchmark converts within tolerances; viewer renders deviations. |
| **M4 – Bridge & Polish** | OCCT integration, installers, telemetry opt-in, daemon/service mode, docs. | Weeks 15–18 | Installer smoke tests, OCCT sewing parity tests, telemetry toggle verified. |

Each milestone requires CAD round-trip validation (Fusion 360, SolidWorks, FreeCAD) and regression tests covering ingest, feature recognition, and QA reporting.

## Status (Updated)

| Item | Status | Notes |
| --- | --- | --- |
| Ingest & Heal (STL/OBJ/PLY, units, vertex dedup) | ✅ | `mesh-io` parses STL/OBJ/PLY, captures units, and `mode-a::heal_vertices` snaps near-duplicate verts. |
| Mode A planar clustering & QA | ✅ | `mode-a` clusters faces, reports topology (boundary/non-manifold edges), exposes planar patches/loops. |
| Planar STEP export (tessellated + B-rep) | ✅ | `step-writer` emits either tessellated shells or `ADVANCED_BREP` using patch loops; `stl2step` feeds QA + patch dumps. |
| Sewing / edge healing | ✅ | Loop orientation normalized, hole loops preserved, and earcut-based sewing collapses <3*tol slivers so merged meshes close watertight. |
| Primitive segmentation & fitting | ✅ | `fit::segment_mesh` tags smooth vs sharp loops and plane/cylinder/sphere/cone/torus fits are exported via CLI `--features`, satisfying the Mode B primitive coverage spec. |
| Freeform deviation reporting | ✅ | `fit::fit_freeform_patches` honors `--surf-tol`, control nets, and feeds QA + preview bundles with per-patch max/RMS errors. |
| Preview & deviation bundle | ✅ | CLI `--preview` emits bbox + per-patch deviation JSON so viewer tooling can light up Mode B diagnostics. |
| Telemetry config & packaging prep | 🚧 | `~/.m2b/config.toml` now governs telemetry opt-in (`--telemetry auto|on|off` overrides); installer/daemon work still pending. |
| Trimming/export hooks | 🚧 | Planar patch metadata now carries deviation data slated for trimmed-surface export; OCCT bridge wiring next. |

## Upcoming Focus

### M3 – Freeform & Trimming
- Expand NURBS fitting scaffolding (`fit::FreeformPatchFit`) into editable surfaces with control net export validated via regression meshes and `--surf-tol` checks.
- Implement trimming pipeline hooks in `step-writer` so stitched freeform patches can share sewing tolerances with Mode A and produce analytic edges for Mode B.
- Expand primitive coverage (cone/torus) and feed those patches into early NURBS fitting so editable outputs exceed the benchmark tolerance.

### M4 – Bridge & Polish
- Define OCCT bridge FFI surface area (sewing + STEP export) and add feature flags/CI smoke to guard it.
- Document installer + packaging requirements per OS, including telemetry toggle plumbing and crash-safe config paths.
- Schedule daemon/viewer readiness tests so desktop + headless distributions stay in lockstep once Mode B ships, and wire preview outputs into that workflow.

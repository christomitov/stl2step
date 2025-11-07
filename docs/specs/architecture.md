# Architecture & Dependencies

## 1. System Overview

```
┌──────────┐   ┌──────────┐   ┌────────────┐   ┌──────────┐   ┌──────────┐
│ Ingest   │→→│ Segments  │→→│ Fit Prims  │→→│ Stitching │→→│ STEP Out  │
│ + Repair │   │ + Tags    │   │ + NURBS    │   │ + QA      │   │ + Reports │
└──────────┘   └──────────┘   └────────────┘   └──────────┘   └──────────┘
```

- **Language:** Rust 1.77+ (edition 2024) for safety and performance.
- **Concurrency:** `rayon` for data-parallel passes; lock-free queues where necessary.
- **Spatial queries:** `kiddo` or `rstar`.
- **Numeric results:** every kernel returns `Result<T, FitError>` (no silent failures).

## 2. Workspace Layout (`crates/`)

- `mesh-io`: STL/OBJ/PLY parsing, deduplication, normalization.
- `geom-core`: DCEL representation, adjacency graphs, curvature, region growing.
- `fit`: RANSAC + least-squares primitive fitters and NURBS surfaces.
- `brep`: topology operations, tolerances, sewing/healing (pure Rust baseline).
- `step-writer`: deterministic AP214/AP242 writer, inspired by `truck-stepio`.
- `occt-bridge` (feature `occt`): optional OpenCascade sewing/export bridge via C FFI.
- `cli`: Clap-based UX, job orchestration, reporting and daemon/service wiring.
- `viewer` (feature `preview`): wgpu-based preview for deviation heatmaps (later milestone).

## 3. Dependencies & Tooling

- **Rust crates:** `nalgebra`, `rayon`, `rstar`/`kiddo`, `thiserror`, `serde`, `clap`, `approx`, `proptest`.
- **External (optional):** OpenCascade libraries distributed as a separate bundle; linked only when `--pro-sew` or feature `occt` is enabled.
- **Build tooling:** Cargo workspaces with `cargo xtask dist` for installers; GitHub Actions CI across Ubuntu/macOS/Windows running `fmt`, `clippy`, unit/integration tests.
- **Test assets:** `assets/tests/` contains synthetic and scanned meshes (planes, cylinders, noisy parts, pathological samples).

## 4. Reliability Principles

- No `unsafe` in core crates; isolate FFI/SIMD to reviewed modules.
- Deterministic results given identical seeds and tolerance settings.
- Rich telemetry (opt-in) capturing performance counters and failure modes to improve heuristics.


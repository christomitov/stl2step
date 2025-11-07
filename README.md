# Mesh→B-Rep

Mesh→B-Rep is a Rust workspace that converts tessellated meshes (STL/OBJ/PLY) into watertight STEP B-reps. The `stl2step` CLI orchestrates the full pipeline:

- **Mode A – Faceted:** Ingests/repairs meshes, clusters coplanar facets, runs sewing, and exports faceted or planar B-reps along with QA diagnostics.
- **Mode B – Editable:** Builds on Mode A by segmenting smooth vs sharp regions, fitting analytic primitives (planes/cylinders/spheres/cones/tori), and capturing per-patch freeform deviation data that will seed the upcoming NURBS/trimmed-surface exporter in tandem with OCCT-backed healing.

Each conversion can emit STEP files, QA reports, planar patch dumps, primitive metadata, freeform QA summaries, and preview/deviation bundles (via `--preview`). Telemetry/UX defaults persist in `$HOME/.m2b/config.toml` and can be overridden per run (`--telemetry auto|on|off`).

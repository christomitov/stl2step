# Mesh→B-Rep Specification Index

The full product specification now lives in focused Markdown documents so we can tackle each area independently and track progress per milestone. Start from this index when deciding what to build or test next.

| Area | Description |
| --- | --- |
| [Product & Modes](specs/product.md) | Goals, target platforms, distribution, and dual Mode A/Mode B definitions. |
| [Architecture & Dependencies](specs/architecture.md) | Workspace layout, core crates, concurrency model, and external libraries/FFI bridges. |
| [Pipelines](specs/pipelines.md) | Stage-by-stage breakdown covering ingest, Mode A, segmentation, primitive fitting, freeform fitting, stitching, and QA/export outputs. |
| [CLI & UX](specs/cli.md) | Command-line surface, flags, diagnostics, and tooling expectations (daemon/viewer hooks). |
| [Performance & QA](specs/performance.md) | Runtime/memory targets, reporting requirements, and preview/telemetry rules. |
| [Packaging & Telemetry](specs/packaging.md) | Installers per OS, config directories, crash safety, and optional telemetry requirements. |
| [Milestones & Deliverables](specs/milestones.md) | Work phases M0–M4 with acceptance criteria and datasets. |
| [Risks & Mitigations](specs/risks.md) | Known technical/business risks plus mitigations. |
| [Open Questions & Next Steps](specs/open-questions.md) | Outstanding decisions needed to unblock later milestones. |

**Working agreement:** Implement features in spec order, write/expand tests after each step, and ensure the workspace compiles before moving on.


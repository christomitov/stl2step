# Risks & Mitigations

- **Non-manifold or low-quality meshes**
  - *Risk:* Repair fails, blocking conversion.
  - *Mitigation:* Provide `--repair` tools (hole fill, component pruning) and clear remediation guidance; suggest Poisson remesh for severe cases.

- **Thin features and slivers**
  - *Risk:* Numerical instability during sewing/fit causes artifacts.
  - *Mitigation:* Detect and warn when features fall below `3*tol`; allow user to increase tolerance or run simplification.

- **Licensing constraints (CGAL/OCC)**
  - *Risk:* GPL packages force server-only deployment; OCC distribution complicates licensing.
  - *Mitigation:* Prefer permissive algorithms for shipped binaries; distribute OCCT as separate optional bundle with clear licensing.

- **Performance on huge scans**
  - *Risk:* Memory blow-ups >5M triangles.
  - *Mitigation:* Implement streaming chunker, optional decimation, and explicit RAM requirement docs.

- **Inconsistent tolerances**
  - *Risk:* Users confused by tolerance configuration leading to failed fits.
  - *Mitigation:* Provide presets (coarse/medium/fine) plus expert overrides and contextual warnings.


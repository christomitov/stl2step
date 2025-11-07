# Pipelines

Stage-by-stage breakdown that underpins both Mode A (faceted) and Mode B (editable) outputs.

## 1. Ingest & Normalize

- Parse STL/OBJ/PLY (binary + ASCII).
- Deduplicate vertices (quantized hashing) and drop zero-area triangles.
- Flood-fill for consistent orientation; repair tiny holes (≤ tolerance) via ear clipping.
- Detect manifoldness; split disconnected shells/components.
- Units detection via file metadata (OBJ `units`, PLY comments) with CLI override prompts.

## 2. Mode A – Faceted Flow

1. **Coplanar merge:** Region-grow facets whose normals differ ≤ `--angle-merge` (default 2°); fit least-squares plane per region.
2. **Face creation:** Convert merged facets into planar faces; build DCEL loops for boundaries.
3. **Sewing:** Snap coincident edges within `--tol`, enforce orientation, collapse slivers shorter than `3*tol`.
4. **Validation:** Attempt to close shell; classify watertight vs open and record diagnostics.
5. **Export:** Write STEP `FacetedBrep` when merge disabled, otherwise `ManifoldSolidBrep`. Emit QA JSON (face count, bbox, watertight flag, units).

## 3. Segmentation & Feature Tagging (Mode B)

- Compute per-face curvature and dihedral angles.
- Classify edges/faces as `sharp` (crease > α) or `smooth`.
- Region-grow smooth patches using SVD of local normals (windowed).
- Build edge graph capturing creases, borders, ridges.
- Detect circular/elliptical loops that suggest holes, bosses, or revolved features.
- Persist segmentation maps as JSON and optional `.ply` overlays for debugging.

## 4. Primitive Recognition

- Run adaptive RANSAC per segment until probability of missing best model < ε.
- Candidate primitives: plane, cylinder, cone, sphere, torus.
- Refine via Levenberg–Marquardt least squares over inliers.
- Accept when RMS ≤ `--prim-tol`, inlier ratio ≥ threshold, and axes align within `--axis-tol`.
- Snap near-nice radii/angles within relative tolerance (e.g., 1e-3) for tidy outputs.
- Tag edges between primitives (plane-cylinder → line/ellipse) for trimming hints.

## 5. Freeform Surface Fitting

- Parameterize segments via mean-value mapping; fall back to LSCM for complex boundaries. Optional quad-remesh stabilizes ill-conditioned patches.
- Choose knots using curvature-weighted chord length; enforce max degree (`--max-degree`, default 3) and control net limits (`--max-patch-cp`, e.g., 12×12).
- Solve least-squares with Tikhonov smoothing; prune low-influence control points to avoid oscillations.
- Validate RMS ≤ `--surf-tol` and target G1 continuity with neighboring surfaces; escalate warnings otherwise.

## 6. Trimming & Topology Stitching

- Convert boundary polylines to analytic curves where possible; otherwise fit BSpline curves under chord-error bound.
- Build trimmed surfaces/wires; snap coincident vertices within tolerance and drop duplicates.
- Pure Rust sewing ensures closed shells; `--pro-sew` hands surfaces to OpenCascade `BRepBuilderAPI_Sewing` for industrial healing.
- Run self-intersection/watertight checks (OCCT `BRepCheck` when available) and downgrade to shell with reason codes if solidification fails.

## 7. Export & QA

- STEP AP242 writer emits analytic surfaces, BSpline surfaces, and 2D trim curves.
- QA bundle (JSON):
  ```json
  {
    "mode": "editable",
    "faces": 37,
    "analytic_area_ratio": 0.76,
    "max_point_error_mm": 0.09,
    "watertight": true,
    "warnings": ["freeform patch 12 exceeded G1 continuity tolerance"]
  }
  ```
- Optional preview assets: `.ply` or `.glb` with per-face deviation scalars for the viewer.


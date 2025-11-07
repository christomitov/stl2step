use std::cmp::Ordering;
use std::collections::{HashMap, HashSet};

use anyhow::{anyhow, Result};
use geom_core::{estimate_stats, DcelStats};
use mesh_io::{Face, Mesh};
use nalgebra::{linalg::SVD, DMatrix, DVector, SymmetricEigen, Vector3};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct RansacConfig {
    pub max_iterations: u32,
    pub confidence: f32,
    pub distance_threshold: f32,
}

impl Default for RansacConfig {
    fn default() -> Self {
        Self {
            max_iterations: 500,
            confidence: 0.99,
            distance_threshold: 0.05,
        }
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum FitError {
    #[error("target RMS {0} is not positive")]
    NonPositiveTolerance(f32),
    #[error("mesh must contain at least one face")]
    EmptyMesh,
}

pub fn make_ransac_config(mesh: &Mesh) -> Result<RansacConfig> {
    let stats = estimate_stats(mesh)
        .map_err(|err| anyhow!("cannot configure RANSAC without valid mesh: {err}"))?;

    Ok(RansacConfig {
        max_iterations: (stats.faces.max(1) as u32).saturating_mul(10).min(25_000),
        confidence: 0.995,
        distance_threshold: dynamic_threshold(&stats),
    })
}

fn dynamic_threshold(stats: &DcelStats) -> f32 {
    let base = 0.25f32;
    let modifier = (stats.faces as f32).max(1.0).ln().max(1.0);
    (base / modifier).max(0.01)
}

pub fn ensure_positive_tolerance(tol: f32) -> Result<()> {
    if tol <= 0.0 {
        Err(FitError::NonPositiveTolerance(tol).into())
    } else {
        Ok(())
    }
}

// --- Segmentation --------------------------------------------------------

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct SegmentationConfig {
    pub smooth_angle_deg: f32,
    pub sharp_angle_deg: f32,
}

impl Default for SegmentationConfig {
    fn default() -> Self {
        Self {
            smooth_angle_deg: 30.0,
            sharp_angle_deg: 45.0,
        }
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum EdgeKind {
    Smooth,
    Sharp,
    Boundary,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct EdgeTag {
    pub vertices: (u32, u32),
    pub kind: EdgeKind,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SegmentInfo {
    pub id: usize,
    pub face_indices: Vec<usize>,
    pub average_normal: [f32; 3],
    pub face_count: usize,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct SegmentationResult {
    pub segments: Vec<SegmentInfo>,
    pub edges: Vec<EdgeTag>,
}

pub fn segment_mesh(
    mesh: &Mesh,
    config: &SegmentationConfig,
) -> Result<SegmentationResult, FitError> {
    if mesh.face_count() == 0 {
        return Err(FitError::EmptyMesh);
    }

    let normals = compute_face_normals(mesh);
    let (adjacency, edge_faces) = build_face_adjacency(mesh);
    let mut segments = Vec::new();
    let mut visited = vec![false; mesh.face_count()];
    let cos_threshold = config.smooth_angle_deg.to_radians().cos();

    for start in 0..mesh.face_count() {
        if visited[start] {
            continue;
        }
        let mut queue = vec![start];
        let mut faces = Vec::new();
        let mut accum = [0.0f32; 3];
        visited[start] = true;

        while let Some(face_idx) = queue.pop() {
            faces.push(face_idx);
            let n = normals[face_idx];
            accum[0] += n[0];
            accum[1] += n[1];
            accum[2] += n[2];

            for &neighbor in &adjacency[face_idx] {
                if visited[neighbor] {
                    continue;
                }
                let dot = dot(normals[face_idx], normals[neighbor]);
                if dot >= cos_threshold {
                    visited[neighbor] = true;
                    queue.push(neighbor);
                }
            }
        }

        if faces.is_empty() {
            continue;
        }

        let len = (accum[0].powi(2) + accum[1].powi(2) + accum[2].powi(2))
            .sqrt()
            .max(f32::EPSILON);
        segments.push(SegmentInfo {
            id: segments.len(),
            face_indices: faces.clone(),
            average_normal: [accum[0] / len, accum[1] / len, accum[2] / len],
            face_count: faces.len(),
        });
    }

    let sharp_threshold = config.sharp_angle_deg.to_radians();
    let mut edges = Vec::new();
    for (edge, faces) in edge_faces {
        match faces.len() {
            0 => continue,
            1 => edges.push(EdgeTag {
                vertices: edge,
                kind: EdgeKind::Boundary,
            }),
            2 => {
                let n0 = normals[faces[0]];
                let n1 = normals[faces[1]];
                let cos = dot(n0, n1).clamp(-1.0, 1.0);
                let angle = cos.acos();
                let kind = if angle >= sharp_threshold {
                    EdgeKind::Sharp
                } else {
                    EdgeKind::Smooth
                };
                edges.push(EdgeTag {
                    vertices: edge,
                    kind,
                });
            }
            _ => edges.push(EdgeTag {
                vertices: edge,
                kind: EdgeKind::Sharp,
            }),
        }
    }

    Ok(SegmentationResult { segments, edges })
}

fn compute_face_normals(mesh: &Mesh) -> Vec<[f32; 3]> {
    mesh.faces
        .iter()
        .map(|face| face_normal(mesh, face))
        .collect()
}

fn face_normal(mesh: &Mesh, face: &Face) -> [f32; 3] {
    normalize(face_normal_raw(mesh, face))
}

fn face_normal_raw(mesh: &Mesh, face: &Face) -> [f32; 3] {
    let a = mesh.vertices[face[0] as usize];
    let b = mesh.vertices[face[1] as usize];
    let c = mesh.vertices[face[2] as usize];
    cross(sub(b, a), sub(c, a))
}

fn build_face_adjacency(mesh: &Mesh) -> (Vec<Vec<usize>>, HashMap<(u32, u32), Vec<usize>>) {
    let mut adjacency = vec![Vec::new(); mesh.face_count()];
    let mut edge_faces: HashMap<(u32, u32), Vec<usize>> = HashMap::new();

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        for edge in face_edges(face) {
            let key = if edge.0 < edge.1 {
                (edge.0, edge.1)
            } else {
                (edge.1, edge.0)
            };
            edge_faces.entry(key).or_default().push(face_idx);
        }
    }

    for faces in edge_faces.values() {
        if faces.len() == 2 {
            adjacency[faces[0]].push(faces[1]);
            adjacency[faces[1]].push(faces[0]);
        }
    }

    (adjacency, edge_faces)
}

fn face_edges(face: &Face) -> [(u32, u32); 3] {
    [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt();
    if len == 0.0 {
        v
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

// --- Primitive recognition ----------------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PrimitiveFitConfig {
    pub rms_threshold: f32,
    pub min_inlier_ratio: f32,
    pub min_radius: f32,
    pub max_radius: f32,
    pub min_axis_length: f32,
}

impl Default for PrimitiveFitConfig {
    fn default() -> Self {
        Self {
            rms_threshold: 0.1,
            min_inlier_ratio: 0.6,
            min_radius: 0.5,
            max_radius: 5_000.0,
            min_axis_length: 1.0,
        }
    }
}

impl PrimitiveFitConfig {
    pub fn with_rms_threshold(rms: f32) -> Result<Self> {
        ensure_positive_tolerance(rms)?;
        let mut cfg = Self::default();
        cfg.rms_threshold = rms;
        Ok(cfg)
    }
}

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub enum PrimitiveKind {
    Plane,
    Cylinder,
    Sphere,
    Cone,
    Torus,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
#[serde(tag = "type", rename_all = "snake_case")]
pub enum PrimitiveParams {
    Plane {
        normal: [f32; 3],
        d: f32,
    },
    Cylinder {
        axis_origin: [f32; 3],
        axis_dir: [f32; 3],
        radius: f32,
        length: f32,
    },
    Sphere {
        center: [f32; 3],
        radius: f32,
    },
    Cone {
        apex: [f32; 3],
        axis_dir: [f32; 3],
        angle_deg: f32,
    },
    Torus {
        center: [f32; 3],
        axis_dir: [f32; 3],
        major_radius: f32,
        minor_radius: f32,
    },
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct PrimitiveFit {
    pub segment_id: usize,
    pub kind: PrimitiveKind,
    pub rms: f32,
    pub inlier_ratio: f32,
    pub params: PrimitiveParams,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Default)]
pub struct PrimitiveReport {
    pub primitives: Vec<PrimitiveFit>,
}

impl PrimitiveReport {
    pub fn count_kind(&self, kind: PrimitiveKind) -> usize {
        self.primitives.iter().filter(|p| p.kind == kind).count()
    }
}

// --- Freeform fitting (placeholder) -------------------------------------

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FreeformPatchInput {
    pub cluster_index: usize,
    pub outer_loop: Vec<u32>,
    pub inner_loops: Vec<Vec<u32>>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FreeformFitConfig {
    pub surf_tolerance_mm: f32,
    pub max_degree: u32,
    pub max_control: (u32, u32),
}

impl Default for FreeformFitConfig {
    fn default() -> Self {
        Self {
            surf_tolerance_mm: 0.25,
            max_degree: 3,
            max_control: (12, 12),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq)]
pub struct FreeformPatchFit {
    pub cluster_index: usize,
    pub rms_error_mm: f32,
    pub max_error_mm: f32,
    pub within_tolerance: bool,
    pub suggested_degree: u32,
    pub suggested_control: (u32, u32),
    pub outer_loop: Vec<u32>,
    pub inner_loops: Vec<Vec<u32>>,
}

pub fn fit_freeform_patches(
    mesh: &Mesh,
    patches: &[FreeformPatchInput],
    config: &FreeformFitConfig,
) -> Vec<FreeformPatchFit> {
    let mut fits = Vec::with_capacity(patches.len());
    for patch in patches {
        if patch.outer_loop.len() < 3 {
            continue;
        }
        let mut indices = HashSet::new();
        for &idx in &patch.outer_loop {
            indices.insert(idx);
        }
        for loop_vertices in &patch.inner_loops {
            for &idx in loop_vertices {
                indices.insert(idx);
            }
        }
        if indices.len() < 3 {
            continue;
        }
        let points: Vec<Vector3<f64>> = indices
            .into_iter()
            .filter_map(|idx| mesh.vertices.get(idx as usize))
            .map(|p| Vector3::new(p[0] as f64, p[1] as f64, p[2] as f64))
            .collect();
        if points.len() < 3 {
            continue;
        }
        let centroid = points.iter().fold(Vector3::zeros(), |acc, p| acc + p) / points.len() as f64;
        let mut cov = nalgebra::Matrix3::zeros();
        for p in &points {
            let v = p - centroid;
            cov += v * v.transpose();
        }
        let eigen = SymmetricEigen::new(cov);
        let normal_vec = eigen
            .eigenvectors
            .column(smallest_index(&eigen.eigenvalues));
        let normal = if normal_vec.norm() < 1e-6 {
            Vector3::new(0.0, 0.0, 1.0)
        } else {
            normal_vec.normalize()
        };
        let mut sum_sq = 0.0f64;
        let mut max_err = 0.0f64;
        for p in &points {
            let dist = (p - centroid).dot(&normal).abs();
            sum_sq += dist * dist;
            if dist > max_err {
                max_err = dist;
            }
        }
        let rms = (sum_sq / points.len() as f64).sqrt() as f32;
        let max_err = max_err as f32;
        let within = rms <= config.surf_tolerance_mm;
        fits.push(FreeformPatchFit {
            cluster_index: patch.cluster_index,
            rms_error_mm: rms,
            max_error_mm: max_err,
            within_tolerance: within,
            suggested_degree: config.max_degree.clamp(1, 5),
            suggested_control: config.max_control,
            outer_loop: patch.outer_loop.clone(),
            inner_loops: patch.inner_loops.clone(),
        });
    }
    fits
}

pub fn recognize_primitives(
    mesh: &Mesh,
    segments: &[SegmentInfo],
    config: &PrimitiveFitConfig,
) -> PrimitiveReport {
    let mut primitives = Vec::new();
    for segment in segments {
        let points = collect_segment_points(mesh, segment);
        if points.len() < 3 {
            continue;
        }
        let mut best: Option<PrimitiveFit> = None;
        for candidate in [
            fit_plane(segment, &points),
            fit_sphere(segment, &points, config),
            fit_cylinder(segment, &points, config),
            fit_cone(segment, &points, config),
            fit_torus(segment, &points, config),
        ] {
            if let Some(fit) = candidate {
                if fit.rms <= config.rms_threshold && fit.inlier_ratio >= config.min_inlier_ratio {
                    let replace = match &best {
                        None => true,
                        Some(existing) => {
                            let rms_improves = fit.rms + 1e-6 < existing.rms;
                            let same_rms = (fit.rms - existing.rms).abs() <= 1e-6;
                            rms_improves
                                || (same_rms
                                    && primitive_priority(fit.kind)
                                        < primitive_priority(existing.kind))
                        }
                    };
                    if replace {
                        best = Some(fit);
                    }
                }
            }
        }
        if let Some(fit) = best {
            primitives.push(fit);
        }
    }
    PrimitiveReport { primitives }
}

fn collect_segment_points(mesh: &Mesh, segment: &SegmentInfo) -> Vec<Vector3<f64>> {
    let mut points = Vec::new();
    let mut seen = HashSet::new();
    for &face_idx in &segment.face_indices {
        if let Some(face) = mesh.faces.get(face_idx) {
            for &vid in face {
                if seen.insert(vid) {
                    let v = mesh.vertices[vid as usize];
                    points.push(Vector3::new(v[0] as f64, v[1] as f64, v[2] as f64));
                }
            }
        }
    }
    points
}

fn fit_plane(segment: &SegmentInfo, points: &[Vector3<f64>]) -> Option<PrimitiveFit> {
    if points.len() < 3 {
        return None;
    }
    let centroid = points.iter().fold(Vector3::zeros(), |acc, p| acc + p) / points.len() as f64;
    let mut cov = nalgebra::Matrix3::zeros();
    for p in points {
        let v = p - centroid;
        cov += v * v.transpose();
    }
    let eigen = SymmetricEigen::new(cov);
    let idx = smallest_index(&eigen.eigenvalues);
    let normal = eigen.eigenvectors.column(idx);
    if normal.norm() < 1e-6 {
        return None;
    }
    let normal = normal.normalize();
    let mut sum_sq = 0.0;
    for p in points {
        let dist = (p - centroid).dot(&normal);
        sum_sq += dist * dist;
    }
    let rms = (sum_sq / points.len() as f64).sqrt() as f32;
    let d = -(normal.dot(&centroid)) as f32;
    Some(PrimitiveFit {
        segment_id: segment.id,
        kind: PrimitiveKind::Plane,
        rms,
        inlier_ratio: 1.0,
        params: PrimitiveParams::Plane {
            normal: [normal.x as f32, normal.y as f32, normal.z as f32],
            d,
        },
    })
}

fn fit_sphere(
    segment: &SegmentInfo,
    points: &[Vector3<f64>],
    config: &PrimitiveFitConfig,
) -> Option<PrimitiveFit> {
    if points.len() < 4 {
        return None;
    }
    let mut a = DMatrix::zeros(points.len(), 4);
    let mut b = DVector::zeros(points.len());
    for (i, p) in points.iter().enumerate() {
        a[(i, 0)] = -2.0 * p.x;
        a[(i, 1)] = -2.0 * p.y;
        a[(i, 2)] = -2.0 * p.z;
        a[(i, 3)] = 1.0;
        b[i] = -(p.x * p.x + p.y * p.y + p.z * p.z);
    }
    let svd = SVD::new(a, true, true);
    let solution = svd.solve(&b, 1e-9).ok()?;
    let center = Vector3::new(solution[0], solution[1], solution[2]);
    let radius_sq = center.dot(&center) - solution[3];
    if radius_sq <= 0.0 {
        return None;
    }
    let radius = radius_sq.sqrt() as f32;
    if radius < config.min_radius || radius > config.max_radius {
        return None;
    }
    let mut sum_sq = 0.0;
    for p in points {
        let dist = (p - center).norm() as f32;
        sum_sq += (dist - radius).powi(2) as f64;
    }
    let rms = (sum_sq / points.len() as f64).sqrt() as f32;
    Some(PrimitiveFit {
        segment_id: segment.id,
        kind: PrimitiveKind::Sphere,
        rms,
        inlier_ratio: 1.0,
        params: PrimitiveParams::Sphere {
            center: [center.x as f32, center.y as f32, center.z as f32],
            radius,
        },
    })
}

fn fit_cylinder(
    segment: &SegmentInfo,
    points: &[Vector3<f64>],
    config: &PrimitiveFitConfig,
) -> Option<PrimitiveFit> {
    if points.len() < 6 {
        return None;
    }
    let centroid = points.iter().fold(Vector3::zeros(), |acc, p| acc + p) / points.len() as f64;
    let mut cov = nalgebra::Matrix3::zeros();
    for p in points {
        let v = p - centroid;
        cov += v * v.transpose();
    }
    let eigen = SymmetricEigen::new(cov);
    let idx = largest_index(&eigen.eigenvalues);
    let axis = eigen.eigenvectors.column(idx);
    if axis.norm() < 1e-6 {
        return None;
    }
    let axis = axis.normalize();
    let mut radii = Vec::with_capacity(points.len());
    let mut min_t = f64::MAX;
    let mut max_t = f64::MIN;
    for p in points {
        let v = p - centroid;
        let t = v.dot(&axis);
        min_t = min_t.min(t);
        max_t = max_t.max(t);
        let radial = v - axis * t;
        radii.push(radial.norm());
    }
    let axis_length = (max_t - min_t) as f32;
    if axis_length < config.min_axis_length {
        return None;
    }
    let radius = (radii.iter().sum::<f64>() / radii.len() as f64) as f32;
    if radius < config.min_radius || radius > config.max_radius {
        return None;
    }
    let mut sum_sq = 0.0;
    for r in radii {
        let diff = r as f32 - radius;
        sum_sq += (diff * diff) as f64;
    }
    let rms = (sum_sq / points.len() as f64).sqrt() as f32;
    Some(PrimitiveFit {
        segment_id: segment.id,
        kind: PrimitiveKind::Cylinder,
        rms,
        inlier_ratio: 1.0,
        params: PrimitiveParams::Cylinder {
            axis_origin: [centroid.x as f32, centroid.y as f32, centroid.z as f32],
            axis_dir: [axis.x as f32, axis.y as f32, axis.z as f32],
            radius,
            length: axis_length,
        },
    })
}

fn primitive_priority(kind: PrimitiveKind) -> u8 {
    match kind {
        PrimitiveKind::Plane => 0,
        PrimitiveKind::Cone => 1,
        PrimitiveKind::Cylinder => 2,
        PrimitiveKind::Sphere => 3,
        PrimitiveKind::Torus => 4,
    }
}

fn average_sample(slice: &[(f64, f64)]) -> (f64, f64) {
    let mut sum_t = 0.0;
    let mut sum_r = 0.0;
    for (t, r) in slice {
        sum_t += *t;
        sum_r += *r;
    }
    let len = slice.len() as f64;
    (sum_t / len, sum_r / len)
}

fn fit_cone(
    segment: &SegmentInfo,
    points: &[Vector3<f64>],
    config: &PrimitiveFitConfig,
) -> Option<PrimitiveFit> {
    if points.len() < 6 {
        return None;
    }
    let centroid = points.iter().fold(Vector3::zeros(), |acc, p| acc + p) / points.len() as f64;
    let mut cov = nalgebra::Matrix3::zeros();
    for p in points {
        let v = p - centroid;
        cov += v * v.transpose();
    }
    let eigen = SymmetricEigen::new(cov);
    let idx = largest_index(&eigen.eigenvalues);
    let axis = eigen.eigenvectors.column(idx);
    if axis.norm() < 1e-6 {
        return None;
    }
    let axis = axis.normalize();
    let mut samples = Vec::with_capacity(points.len());
    let mut min_t = f64::MAX;
    let mut max_t = f64::MIN;
    for p in points {
        let v = p - centroid;
        let t = v.dot(&axis);
        let radial = v - axis * t;
        let r = radial.norm();
        samples.push((t, r));
        min_t = min_t.min(t);
        max_t = max_t.max(t);
    }
    let axis_length = (max_t - min_t) as f32;
    if axis_length < config.min_axis_length {
        return None;
    }
    samples.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
    let bucket = (samples.len() / 5).max(1);
    let (t_lo, r_lo) = average_sample(&samples[..bucket]);
    let (t_hi, r_hi) = average_sample(&samples[samples.len() - bucket..]);
    if (t_hi - t_lo).abs() < 1e-6 {
        return None;
    }
    let slope = (r_hi - r_lo) / (t_hi - t_lo);
    if slope.abs() < 1e-6 {
        return None;
    }
    let intercept = r_lo - slope * t_lo;
    let apex_t = -intercept / slope;
    let apex = centroid + axis * apex_t;
    let mut sum_sq = 0.0f64;
    for (t, r) in samples {
        let pred = slope * t + intercept;
        if pred <= 1e-6 {
            return None;
        }
        let diff = r - pred;
        sum_sq += diff * diff;
    }
    let rms = (sum_sq / points.len() as f64).sqrt() as f32;
    let angle_rad = slope.abs().atan();
    Some(PrimitiveFit {
        segment_id: segment.id,
        kind: PrimitiveKind::Cone,
        rms,
        inlier_ratio: 1.0,
        params: PrimitiveParams::Cone {
            apex: [apex.x as f32, apex.y as f32, apex.z as f32],
            axis_dir: [axis.x as f32, axis.y as f32, axis.z as f32],
            angle_deg: angle_rad.to_degrees() as f32,
        },
    })
}

fn fit_torus(
    segment: &SegmentInfo,
    points: &[Vector3<f64>],
    config: &PrimitiveFitConfig,
) -> Option<PrimitiveFit> {
    if points.len() < 8 {
        return None;
    }
    let centroid = points.iter().fold(Vector3::zeros(), |acc, p| acc + p) / points.len() as f64;
    let mut cov = nalgebra::Matrix3::zeros();
    for p in points {
        let v = p - centroid;
        cov += v * v.transpose();
    }
    let eigen = SymmetricEigen::new(cov);
    let idx = smallest_index(&eigen.eigenvalues);
    let axis = eigen.eigenvectors.column(idx);
    if axis.norm() < 1e-6 {
        return None;
    }
    let axis = axis.normalize();
    let mut ring_radii = Vec::with_capacity(points.len());
    let mut tube_offsets = Vec::with_capacity(points.len());
    for p in points {
        let v = p - centroid;
        let axial = v.dot(&axis);
        let radial_vec = v - axis * axial;
        let radial = radial_vec.norm();
        ring_radii.push(radial);
        tube_offsets.push(axial);
    }
    let major_radius = ring_radii.iter().sum::<f64>() / ring_radii.len() as f64;
    if major_radius <= 0.0 {
        return None;
    }
    if !(config.min_radius as f64..=config.max_radius as f64).contains(&major_radius) {
        return None;
    }
    let tube_values: Vec<f64> = ring_radii
        .iter()
        .zip(tube_offsets.iter())
        .map(|(radial, axial)| ((radial - major_radius).powi(2) + axial.powi(2)).sqrt())
        .collect();
    let minor_radius = tube_values.iter().sum::<f64>() / tube_values.len() as f64;
    if minor_radius <= 1e-3 || minor_radius > config.max_radius as f64 {
        return None;
    }
    let mut sum_sq = 0.0f64;
    let mut max_dev = 0.0f64;
    for &val in &tube_values {
        let diff = val - minor_radius;
        sum_sq += diff * diff;
        if diff.abs() > max_dev {
            max_dev = diff.abs();
        }
    }
    let rms = (sum_sq / tube_values.len() as f64).sqrt() as f32;
    if rms > config.rms_threshold {
        return None;
    }
    Some(PrimitiveFit {
        segment_id: segment.id,
        kind: PrimitiveKind::Torus,
        rms,
        inlier_ratio: 1.0,
        params: PrimitiveParams::Torus {
            center: [centroid.x as f32, centroid.y as f32, centroid.z as f32],
            axis_dir: [axis.x as f32, axis.y as f32, axis.z as f32],
            major_radius: major_radius as f32,
            minor_radius: minor_radius as f32,
        },
    })
}

fn smallest_index(values: &Vector3<f64>) -> usize {
    let mut idx = 0;
    let mut min = values[0];
    if values[1] < min {
        min = values[1];
        idx = 1;
    }
    if values[2] < min {
        idx = 2;
    }
    idx
}

fn largest_index(values: &Vector3<f64>) -> usize {
    let mut idx = 0;
    let mut max = values[0];
    if values[1] > max {
        max = values[1];
        idx = 1;
    }
    if values[2] > max {
        idx = 2;
    }
    idx
}

// --- Tests ---------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_config_is_sensible() {
        let mesh = Mesh {
            vertices: vec![[0.0, 0.0, 0.0]; 4],
            faces: vec![[0, 1, 2], [0, 2, 3]],
        };
        let config = make_ransac_config(&mesh).expect("config should build");
        assert!(config.max_iterations >= 20);
        assert!(config.confidence > 0.9);
        assert!(config.distance_threshold <= 0.25);
    }

    #[test]
    fn ensure_positive_tolerance_flags_bad_values() {
        let err = ensure_positive_tolerance(0.0).unwrap_err();
        assert!(err.to_string().contains("not positive"));
        ensure_positive_tolerance(0.1).expect("tol > 0 works");
    }

    #[test]
    fn segmentation_detects_cube_faces() {
        let mesh = cube_mesh();
        let result = segment_mesh(&mesh, &SegmentationConfig::default()).expect("segments");
        assert_eq!(result.segments.len(), 6);
        assert!(result
            .edges
            .iter()
            .any(|e| matches!(e.kind, EdgeKind::Sharp)));
    }

    #[test]
    fn plane_segment_is_recognized() {
        let mesh = plane_mesh();
        let segment = SegmentInfo {
            id: 0,
            face_indices: (0..mesh.faces.len()).collect(),
            average_normal: [0.0, 0.0, 1.0],
            face_count: mesh.faces.len(),
        };
        let points = collect_segment_points(&mesh, &segment);
        assert!(points.len() >= 3);
        assert!(fit_plane(&segment, &points).is_some());
        let report = recognize_primitives(&mesh, &[segment], &PrimitiveFitConfig::default());
        assert_eq!(report.count_kind(PrimitiveKind::Plane), 1);
    }

    #[test]
    fn cylinder_segment_is_recognized() {
        let mesh = cylinder_mesh();
        let segment = SegmentInfo {
            id: 0,
            face_indices: (0..mesh.faces.len()).collect(),
            average_normal: [1.0, 0.0, 0.0],
            face_count: mesh.faces.len(),
        };
        let mut config = PrimitiveFitConfig::default();
        config.min_axis_length = 0.5;
        let points = collect_segment_points(&mesh, &segment);
        assert!(fit_cylinder(&segment, &points, &config).is_some());
        let report = recognize_primitives(&mesh, &[segment], &config);
        assert_eq!(report.count_kind(PrimitiveKind::Cylinder), 1);
    }

    #[test]
    fn sphere_segment_is_recognized() {
        let mesh = sphere_mesh();
        let segment = SegmentInfo {
            id: 0,
            face_indices: (0..mesh.faces.len()).collect(),
            average_normal: [0.0, 0.0, 1.0],
            face_count: mesh.faces.len(),
        };
        let report = recognize_primitives(&mesh, &[segment], &PrimitiveFitConfig::default());
        assert_eq!(report.count_kind(PrimitiveKind::Sphere), 1);
    }

    #[test]
    fn cone_segment_is_recognized() {
        let mesh = cone_mesh();
        let segment = SegmentInfo {
            id: 0,
            face_indices: (0..mesh.faces.len()).collect(),
            average_normal: [0.0, 0.0, 1.0],
            face_count: mesh.faces.len(),
        };
        let mut config = PrimitiveFitConfig::default();
        config.min_axis_length = 0.5;
        let report = recognize_primitives(&mesh, &[segment], &config);
        assert_eq!(report.count_kind(PrimitiveKind::Cone), 1);
    }

    #[test]
    fn torus_segment_is_recognized() {
        let mesh = torus_mesh();
        let segment = SegmentInfo {
            id: 0,
            face_indices: (0..mesh.faces.len()).collect(),
            average_normal: [0.0, 0.0, 1.0],
            face_count: mesh.faces.len(),
        };
        let report = recognize_primitives(&mesh, &[segment], &PrimitiveFitConfig::default());
        assert_eq!(report.count_kind(PrimitiveKind::Torus), 1);
    }

    #[test]
    fn freeform_patch_reports_errors() {
        let mesh = plane_mesh();
        let mut warped_mesh = mesh.clone();
        warped_mesh.vertices[3][2] = 0.5; // introduce deviation
        let input = FreeformPatchInput {
            cluster_index: 0,
            outer_loop: vec![0, 1, 2, 3],
            inner_loops: vec![],
        };
        let config = FreeformFitConfig {
            surf_tolerance_mm: 0.1,
            max_degree: 3,
            max_control: (12, 12),
        };
        let fits = fit_freeform_patches(&warped_mesh, &[input], &config);
        assert_eq!(fits.len(), 1);
        let fit = &fits[0];
        assert!(fit.max_error_mm > 0.1);
        assert!(!fit.within_tolerance);
    }

    fn cube_mesh() -> Mesh {
        let vertices = vec![
            [-0.5, -0.5, -0.5],
            [0.5, -0.5, -0.5],
            [0.5, 0.5, -0.5],
            [-0.5, 0.5, -0.5],
            [-0.5, -0.5, 0.5],
            [0.5, -0.5, 0.5],
            [0.5, 0.5, 0.5],
            [-0.5, 0.5, 0.5],
        ];
        let faces = vec![
            [0, 1, 2],
            [0, 2, 3], // bottom
            [4, 5, 6],
            [4, 6, 7], // top
            [0, 4, 5],
            [0, 5, 1], // front
            [1, 5, 6],
            [1, 6, 2], // right
            [2, 6, 7],
            [2, 7, 3], // back
            [3, 7, 4],
            [3, 4, 0], // left
        ];
        Mesh { vertices, faces }
    }

    fn plane_mesh() -> Mesh {
        Mesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [1.0, 1.0, 0.0],
                [0.0, 1.0, 0.0],
            ],
            faces: vec![[0, 1, 2], [0, 2, 3]],
        }
    }

    fn cylinder_mesh() -> Mesh {
        let segments = 12;
        let height = 1.0f32;
        let radius = 0.5f32;
        let mut vertices = Vec::new();
        for i in 0..segments {
            let theta = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let x = radius * theta.cos();
            let y = radius * theta.sin();
            vertices.push([x, y, -height / 2.0]);
            vertices.push([x, y, height / 2.0]);
        }
        let mut faces = Vec::new();
        for i in 0..segments {
            let next = (i + 1) % segments;
            let base = (i * 2) as u32;
            let next_base = (next * 2) as u32;
            faces.push([base, base + 1, next_base + 1]);
            faces.push([base, next_base + 1, next_base]);
        }
        Mesh { vertices, faces }
    }

    fn sphere_mesh() -> Mesh {
        // Create octahedron-based sphere approx
        let vertices = vec![
            [0.0, 0.0, 1.0],
            [0.0, 0.0, -1.0],
            [1.0, 0.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0],
        ];
        let faces = vec![
            [0, 2, 4],
            [0, 4, 3],
            [0, 3, 5],
            [0, 5, 2],
            [1, 4, 2],
            [1, 3, 4],
            [1, 5, 3],
            [1, 2, 5],
        ];
        Mesh { vertices, faces }
    }

    fn cone_mesh() -> Mesh {
        let segments = 16;
        let height = 1.0f32;
        let radius_bottom = 0.8f32;
        let radius_top = 0.2f32;
        let mut vertices = Vec::new();
        for i in 0..segments {
            let theta = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            vertices.push([radius_bottom * cos_t, radius_bottom * sin_t, 0.0]);
        }
        for i in 0..segments {
            let theta = (i as f32 / segments as f32) * std::f32::consts::TAU;
            let cos_t = theta.cos();
            let sin_t = theta.sin();
            vertices.push([radius_top * cos_t, radius_top * sin_t, height]);
        }
        let mut faces = Vec::new();
        for i in 0..segments {
            let next = (i + 1) % segments;
            let bottom_a = i as u32;
            let bottom_b = next as u32;
            let top_a = (i + segments) as u32;
            let top_b = (next + segments) as u32;
            faces.push([bottom_a, top_a, top_b]);
            faces.push([bottom_a, top_b, bottom_b]);
        }
        Mesh { vertices, faces }
    }

    fn torus_mesh() -> Mesh {
        let major = 1.0f32;
        let minor = 0.25f32;
        let rings = 16;
        let sides = 12;
        let mut vertices = Vec::new();
        for i in 0..rings {
            let theta = (i as f32 / rings as f32) * std::f32::consts::TAU;
            let cos_theta = theta.cos();
            let sin_theta = theta.sin();
            for j in 0..sides {
                let phi = (j as f32 / sides as f32) * std::f32::consts::TAU;
                let cos_phi = phi.cos();
                let sin_phi = phi.sin();
                let x = (major + minor * cos_phi) * cos_theta;
                let y = (major + minor * cos_phi) * sin_theta;
                let z = minor * sin_phi;
                vertices.push([x, y, z]);
            }
        }
        let mut faces = Vec::new();
        for i in 0..rings {
            for j in 0..sides {
                let next_i = (i + 1) % rings;
                let next_j = (j + 1) % sides;
                let a = (i * sides + j) as u32;
                let b = (i * sides + next_j) as u32;
                let c = (next_i * sides + next_j) as u32;
                let d = (next_i * sides + j) as u32;
                faces.push([a, b, c]);
                faces.push([a, c, d]);
            }
        }
        Mesh { vertices, faces }
    }
}

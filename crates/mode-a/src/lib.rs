use std::cmp::Ordering;
use std::collections::{HashMap, VecDeque};

use earcutr::earcut;

use mesh_io::{Face, Mesh};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ModeAError {
    #[error("mesh must contain at least one face")]
    EmptyMesh,
    #[error("heal tolerance must be positive")]
    InvalidTolerance,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanarAnalysis {
    pub angle_threshold_deg: f32,
    pub clusters: Vec<PlanarCluster>,
}

impl PlanarAnalysis {
    pub fn cluster_count(&self) -> usize {
        self.clusters.len()
    }

    pub fn largest_cluster_size(&self) -> usize {
        self.clusters
            .iter()
            .map(|cluster| cluster.face_indices.len())
            .max()
            .unwrap_or(0)
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanarCluster {
    pub face_indices: Vec<usize>,
    pub normal: [f32; 3],
    pub area: f32,
}

#[derive(Debug, Clone)]
pub struct ClusterMergeResult {
    pub faces: Vec<Face>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanarPatch {
    pub cluster_index: usize,
    pub outer_boundary: Vec<u32>,
    pub inner_boundaries: Vec<Vec<u32>>,
    pub normal: [f32; 3],
    pub area: f32,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanarShell {
    pub patches: Vec<PlanarPatch>,
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub struct TopologyReport {
    pub boundary_edges: usize,
    pub non_manifold_edges: usize,
    pub is_watertight: bool,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HealReport {
    pub tolerance: f32,
    pub merged_vertices: usize,
}
pub fn heal_vertices(mesh: &mut Mesh, tolerance: f32) -> Result<HealReport, ModeAError> {
    if tolerance <= 0.0 {
        return Err(ModeAError::InvalidTolerance);
    }
    let mut map: HashMap<(i64, i64, i64), u32> = HashMap::new();
    let mut new_vertices = Vec::with_capacity(mesh.vertices.len());
    let mut remap = vec![0u32; mesh.vertices.len()];
    let mut merged = 0usize;
    for (idx, vertex) in mesh.vertices.iter().enumerate() {
        let key = quantize(vertex, tolerance);
        if let Some(&existing) = map.get(&key) {
            remap[idx] = existing;
            merged += 1;
        } else {
            let new_index = new_vertices.len() as u32;
            map.insert(key, new_index);
            new_vertices.push(*vertex);
            remap[idx] = new_index;
        }
    }
    mesh.vertices = new_vertices;
    mesh.faces.retain_mut(|face| {
        face[0] = remap[face[0] as usize];
        face[1] = remap[face[1] as usize];
        face[2] = remap[face[2] as usize];
        face[0] != face[1] && face[1] != face[2] && face[0] != face[2]
    });
    Ok(HealReport {
        tolerance,
        merged_vertices: merged,
    })
}

pub fn analyze_planarity(
    mesh: &Mesh,
    angle_threshold_deg: f32,
) -> Result<PlanarAnalysis, ModeAError> {
    if mesh.face_count() == 0 {
        return Err(ModeAError::EmptyMesh);
    }

    let normals = compute_face_normals(mesh);
    let areas = compute_face_areas(mesh);
    let adjacency = build_face_adjacency(mesh);
    let clusters = cluster_faces(
        &normals,
        &areas,
        &adjacency,
        angle_threshold_deg.to_radians(),
    );

    Ok(PlanarAnalysis {
        angle_threshold_deg,
        clusters,
    })
}

fn compute_face_normals(mesh: &Mesh) -> Vec<[f32; 3]> {
    mesh.faces
        .iter()
        .map(|face| face_normal(mesh, face))
        .collect()
}

fn compute_face_areas(mesh: &Mesh) -> Vec<f32> {
    mesh.faces
        .iter()
        .map(|face| face_area(mesh, face))
        .collect()
}

fn build_face_adjacency(mesh: &Mesh) -> Vec<Vec<usize>> {
    let mut map: HashMap<(u32, u32), usize> = HashMap::new();
    let mut adjacency = vec![Vec::new(); mesh.face_count()];

    for (face_idx, face) in mesh.faces.iter().enumerate() {
        for edge in face_edges(face) {
            let key = if edge.0 < edge.1 {
                (edge.0, edge.1)
            } else {
                (edge.1, edge.0)
            };
            if let Some(&other_idx) = map.get(&key) {
                adjacency[face_idx].push(other_idx);
                adjacency[other_idx].push(face_idx);
            } else {
                map.insert(key, face_idx);
            }
        }
    }

    adjacency
}

fn cluster_faces(
    normals: &[[f32; 3]],
    areas: &[f32],
    adjacency: &[Vec<usize>],
    angle_threshold_rad: f32,
) -> Vec<PlanarCluster> {
    let threshold = angle_threshold_rad.cos();
    let mut visited = vec![false; normals.len()];
    let mut clusters = Vec::new();

    for start in 0..normals.len() {
        if visited[start] {
            continue;
        }
        let mut queue = VecDeque::new();
        let mut faces = Vec::new();
        let mut accumulated_area = 0.0f32;
        let base_normal = normals[start];
        queue.push_back(start);
        visited[start] = true;

        while let Some(face_idx) = queue.pop_front() {
            faces.push(face_idx);
            accumulated_area += areas[face_idx];

            for &neighbor in &adjacency[face_idx] {
                if visited[neighbor] {
                    continue;
                }
                if dot(normals[neighbor], base_normal) >= threshold {
                    visited[neighbor] = true;
                    queue.push_back(neighbor);
                }
            }
        }

        clusters.push(PlanarCluster {
            face_indices: faces,
            normal: normalize(base_normal),
            area: accumulated_area,
        });
    }

    clusters
}

fn face_edges(face: &Face) -> [(u32, u32); 3] {
    [(face[0], face[1]), (face[1], face[2]), (face[2], face[0])]
}

fn face_normal(mesh: &Mesh, face: &Face) -> [f32; 3] {
    normalize(face_normal_raw(mesh, face))
}

fn face_area(mesh: &Mesh, face: &Face) -> f32 {
    let normal = face_normal_raw(mesh, face);
    0.5 * length(normal)
}

fn face_normal_raw(mesh: &Mesh, face: &Face) -> [f32; 3] {
    let a = mesh.vertices[face[0] as usize];
    let b = mesh.vertices[face[1] as usize];
    let c = mesh.vertices[face[2] as usize];
    cross(sub(b, a), sub(c, a))
}

fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn length(v: [f32; 3]) -> f32 {
    (v[0].powi(2) + v[1].powi(2) + v[2].powi(2)).sqrt()
}

fn normalize(v: [f32; 3]) -> [f32; 3] {
    let len = length(v);
    if len == 0.0 {
        v
    } else {
        [v[0] / len, v[1] / len, v[2] / len]
    }
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

pub fn merge_planar_clusters(mesh: &Mesh, analysis: &PlanarAnalysis, tolerance: f32) -> Mesh {
    let mut faces = Vec::with_capacity(
        analysis
            .cluster_count()
            .max(mesh.face_count())
            .saturating_mul(2),
    );
    for (idx, cluster) in analysis.clusters.iter().enumerate() {
        let patch = build_planar_patch(mesh, cluster, idx, tolerance);
        if patch.outer_boundary.len() >= 3 {
            if let Some(mut sewn) = triangulate_patch(mesh, &patch) {
                faces.append(&mut sewn);
                continue;
            }
        }
        faces.extend(merge_cluster(mesh, cluster));
    }
    Mesh {
        vertices: mesh.vertices.clone(),
        faces,
    }
}

pub fn extract_planar_patches(
    mesh: &Mesh,
    analysis: &PlanarAnalysis,
    tolerance: f32,
) -> PlanarShell {
    let patches = analysis
        .clusters
        .iter()
        .enumerate()
        .map(|(idx, cluster)| build_planar_patch(mesh, cluster, idx, tolerance))
        .collect();
    PlanarShell { patches }
}

fn build_planar_patch(
    mesh: &Mesh,
    cluster: &PlanarCluster,
    cluster_index: usize,
    tolerance: f32,
) -> PlanarPatch {
    let loops = boundary_loops(mesh, cluster);
    let (outer_boundary, inner_boundaries) = classify_loops(mesh, cluster, loops, tolerance);
    PlanarPatch {
        cluster_index,
        outer_boundary,
        inner_boundaries,
        normal: cluster.normal,
        area: cluster.area,
    }
}

fn merge_cluster(mesh: &Mesh, cluster: &PlanarCluster) -> Vec<Face> {
    let mut unique = collect_unique_vertices(mesh, cluster);
    if unique.len() < 3 {
        return cluster
            .face_indices
            .iter()
            .map(|&idx| mesh.faces[idx])
            .collect();
    }
    unique.sort_unstable();
    unique.dedup();
    if unique.len() == 3 {
        return vec![[unique[0], unique[1], unique[2]]];
    }

    let normal = normalize(cluster.normal);
    let (tangent, bitangent) = plane_basis(normal);
    let origin = mesh.vertices[unique[0] as usize];
    let mut projected = Vec::with_capacity(unique.len());
    for &idx in &unique {
        let point = mesh.vertices[idx as usize];
        let delta = sub(point, origin);
        let u = dot(delta, tangent) as f64;
        let v = dot(delta, bitangent) as f64;
        projected.push((idx, u, v));
    }

    let hull = convex_hull(&projected);
    if hull.len() < 3 {
        cluster
            .face_indices
            .iter()
            .map(|&idx| mesh.faces[idx])
            .collect()
    } else {
        fan_triangulate(&hull)
    }
}

fn triangulate_patch(mesh: &Mesh, patch: &PlanarPatch) -> Option<Vec<Face>> {
    if patch.outer_boundary.len() < 3 {
        return None;
    }
    let normal = normalize(patch.normal);
    let (tangent, bitangent) = plane_basis(normal);
    let origin = mesh.vertices[*patch.outer_boundary.first()? as usize];
    let mut coords: Vec<[f64; 2]> = Vec::new();
    let mut index_lut: Vec<u32> = Vec::new();

    let mut outer = patch.outer_boundary.clone();
    if polygon_area(mesh, &outer, origin, tangent, bitangent) < 0.0 {
        outer.reverse();
    }
    append_loop_vertices(
        mesh,
        &outer,
        origin,
        tangent,
        bitangent,
        &mut coords,
        &mut index_lut,
    )?;

    let mut hole_indices = Vec::new();
    for hole in &patch.inner_boundaries {
        if hole.len() < 3 {
            continue;
        }
        let mut loop_vertices = hole.clone();
        if polygon_area(mesh, &loop_vertices, origin, tangent, bitangent) > 0.0 {
            loop_vertices.reverse();
        }
        let start = index_lut.len();
        hole_indices.push(start);
        if append_loop_vertices(
            mesh,
            &loop_vertices,
            origin,
            tangent,
            bitangent,
            &mut coords,
            &mut index_lut,
        )
        .is_none()
        {
            hole_indices.pop();
            coords.truncate(start);
            index_lut.truncate(start);
        }
    }

    if coords.len() < 3 {
        return None;
    }

    let mut flattened = Vec::with_capacity(coords.len() * 2);
    for pair in &coords {
        flattened.push(pair[0]);
        flattened.push(pair[1]);
    }
    let indices = match earcut(&flattened, &hole_indices, 2) {
        Ok(ix) => ix,
        Err(_) => return None,
    };
    if indices.len() < 3 {
        return None;
    }
    let mut faces = Vec::with_capacity(indices.len() / 3);
    for tri in indices.chunks(3) {
        if tri.len() < 3 {
            continue;
        }
        let a = index_lut[tri[0]];
        let b = index_lut[tri[1]];
        let c = index_lut[tri[2]];
        if a == b || b == c || c == a {
            continue;
        }
        faces.push([a, b, c]);
    }
    if faces.is_empty() {
        None
    } else {
        Some(faces)
    }
}

fn append_loop_vertices(
    mesh: &Mesh,
    loop_vertices: &[u32],
    origin: [f32; 3],
    tangent: [f32; 3],
    bitangent: [f32; 3],
    coords: &mut Vec<[f64; 2]>,
    index_lut: &mut Vec<u32>,
) -> Option<()> {
    if loop_vertices.len() < 3 {
        return None;
    }
    for &idx in loop_vertices {
        if (idx as usize) >= mesh.vertices.len() {
            return None;
        }
        let point = mesh.vertices[idx as usize];
        let (u, v) = project_point(point, origin, tangent, bitangent);
        coords.push([u, v]);
        index_lut.push(idx);
    }
    Some(())
}

fn collect_unique_vertices(mesh: &Mesh, cluster: &PlanarCluster) -> Vec<u32> {
    let mut unique = Vec::new();
    for &face_idx in &cluster.face_indices {
        let face = mesh.faces[face_idx];
        unique.push(face[0]);
        unique.push(face[1]);
        unique.push(face[2]);
    }
    unique
}

fn plane_basis(normal: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    let up = if normal[2].abs() < 0.9 {
        [0.0, 0.0, 1.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let tangent = normalize(cross(up, normal));
    let bitangent = normalize(cross(normal, tangent));
    (tangent, bitangent)
}

fn convex_hull(points: &[(u32, f64, f64)]) -> Vec<u32> {
    if points.len() <= 3 {
        return points.iter().map(|(idx, _, _)| *idx).collect();
    }
    let mut pts = points.to_vec();
    pts.sort_by(|a, b| {
        a.1.partial_cmp(&b.1)
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal))
    });
    let mut lower: Vec<(u32, f64, f64)> = Vec::new();
    for p in &pts {
        while lower.len() >= 2
            && cross_2d(lower[lower.len() - 2], lower[lower.len() - 1], *p) <= 0.0
        {
            lower.pop();
        }
        lower.push(*p);
    }
    let mut upper: Vec<(u32, f64, f64)> = Vec::new();
    for p in pts.iter().rev() {
        while upper.len() >= 2
            && cross_2d(upper[upper.len() - 2], upper[upper.len() - 1], *p) <= 0.0
        {
            upper.pop();
        }
        upper.push(*p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower.iter().map(|(idx, _, _)| *idx).collect()
}

fn cross_2d(a: (u32, f64, f64), b: (u32, f64, f64), c: (u32, f64, f64)) -> f64 {
    (b.1 - a.1) * (c.2 - a.2) - (b.2 - a.2) * (c.1 - a.1)
}

fn fan_triangulate(indices: &[u32]) -> Vec<Face> {
    let mut faces = Vec::new();
    if indices.len() < 3 {
        return faces;
    }
    for i in 1..indices.len() - 1 {
        faces.push([indices[0], indices[i], indices[i + 1]]);
    }
    faces
}

fn classify_loops(
    mesh: &Mesh,
    cluster: &PlanarCluster,
    loops: Vec<Vec<u32>>,
    tolerance: f32,
) -> (Vec<u32>, Vec<Vec<u32>>) {
    if loops.is_empty() {
        return (Vec::new(), Vec::new());
    }
    let (tangent, bitangent) = plane_basis(cluster.normal);
    let origin = mesh.vertices[mesh.faces[cluster.face_indices[0]][0] as usize];
    let min_edge = (tolerance.max(1e-4) * 3.0).max(1e-3);

    let mut loops_with_area: Vec<(Vec<u32>, f32)> = Vec::new();
    for mut loop_vertices in loops {
        sanitize_loop(&mut loop_vertices, mesh, min_edge);
        if loop_vertices.len() < 3 {
            continue;
        }
        let area = polygon_area(mesh, &loop_vertices, origin, tangent, bitangent);
        loops_with_area.push((loop_vertices, area));
    }

    if loops_with_area.is_empty() {
        return (Vec::new(), Vec::new());
    }

    loops_with_area.sort_by(|a, b| b.1.abs().partial_cmp(&a.1.abs()).unwrap_or(Ordering::Equal));

    let mut iter = loops_with_area.into_iter();
    let mut outer = Vec::new();
    let mut inner = Vec::new();

    if let Some((mut loop_vertices, area)) = iter.next() {
        if area < 0.0 {
            loop_vertices.reverse();
        }
        outer = loop_vertices;
    }

    for (mut loop_vertices, area) in iter {
        if area > 0.0 {
            loop_vertices.reverse();
        }
        inner.push(loop_vertices);
    }

    (outer, inner)
}

fn sanitize_loop(loop_vertices: &mut Vec<u32>, mesh: &Mesh, min_edge: f32) {
    loop_vertices.retain(|&idx| (idx as usize) < mesh.vertices.len());
    if loop_vertices.len() < 3 {
        loop_vertices.clear();
        return;
    }
    if loop_vertices.first() == loop_vertices.last() {
        loop_vertices.pop();
    }
    loop_vertices.dedup();
    if loop_vertices.len() < 3 {
        loop_vertices.clear();
        return;
    }
    collapse_short_edges(loop_vertices, mesh, min_edge);
    if loop_vertices.len() >= 3 && loop_vertices.first() == loop_vertices.last() {
        loop_vertices.pop();
    }
    if loop_vertices.len() < 3 {
        loop_vertices.clear();
    }
}

fn collapse_short_edges(loop_vertices: &mut Vec<u32>, mesh: &Mesh, min_edge: f32) {
    if loop_vertices.len() < 3 {
        return;
    }
    let mut cleaned = Vec::with_capacity(loop_vertices.len());
    for &idx in loop_vertices.iter() {
        if cleaned.is_empty() {
            cleaned.push(idx);
            continue;
        }
        let prev = *cleaned.last().unwrap();
        if edge_length(mesh, prev, idx) < min_edge {
            continue;
        }
        cleaned.push(idx);
    }
    if cleaned.len() >= 2 {
        let first = cleaned[0];
        if edge_length(mesh, *cleaned.last().unwrap(), first) < min_edge {
            cleaned.pop();
        }
    }
    if cleaned.len() >= 3 {
        *loop_vertices = cleaned;
    } else {
        loop_vertices.clear();
    }
}

fn edge_length(mesh: &Mesh, a: u32, b: u32) -> f32 {
    if a == b {
        return 0.0;
    }
    let pa = mesh.vertices[a as usize];
    let pb = mesh.vertices[b as usize];
    ((pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2)).sqrt()
}

fn polygon_area(
    mesh: &Mesh,
    loop_vertices: &[u32],
    origin: [f32; 3],
    tangent: [f32; 3],
    bitangent: [f32; 3],
) -> f32 {
    if loop_vertices.len() < 3 {
        return 0.0;
    }
    let mut area = 0.0f32;
    let mut coords = Vec::with_capacity(loop_vertices.len());
    for &idx in loop_vertices {
        let point = mesh.vertices[idx as usize];
        let delta = sub(point, origin);
        coords.push((dot(delta, tangent), dot(delta, bitangent)));
    }
    for i in 0..coords.len() {
        let (x1, y1) = coords[i];
        let (x2, y2) = coords[(i + 1) % coords.len()];
        area += x1 * y2 - x2 * y1;
    }
    0.5 * area
}

fn project_point(
    point: [f32; 3],
    origin: [f32; 3],
    tangent: [f32; 3],
    bitangent: [f32; 3],
) -> (f64, f64) {
    let delta = sub(point, origin);
    (dot(delta, tangent) as f64, dot(delta, bitangent) as f64)
}

pub fn analyze_topology(mesh: &Mesh) -> TopologyReport {
    let mut edge_counts: HashMap<(u32, u32), usize> = HashMap::new();
    for face in &mesh.faces {
        for &(a, b) in &face_edges(face) {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_counts.entry(key).or_insert(0) += 1;
        }
    }
    let mut boundary_edges = 0usize;
    let mut non_manifold_edges = 0usize;
    for count in edge_counts.values() {
        match *count {
            1 => boundary_edges += 1,
            2 => (),
            _ => non_manifold_edges += 1,
        }
    }

    TopologyReport {
        boundary_edges,
        non_manifold_edges,
        is_watertight: boundary_edges == 0 && non_manifold_edges == 0,
    }
}

fn quantize(vertex: &[f32; 3], tolerance: f32) -> (i64, i64, i64) {
    let scale = 1.0 / tolerance;
    (
        (vertex[0] as f64 * scale as f64).round() as i64,
        (vertex[1] as f64 * scale as f64).round() as i64,
        (vertex[2] as f64 * scale as f64).round() as i64,
    )
}

fn boundary_loops(mesh: &Mesh, cluster: &PlanarCluster) -> Vec<Vec<u32>> {
    let mut edge_usage: HashMap<(u32, u32), usize> = HashMap::new();
    for &face_idx in &cluster.face_indices {
        let face = mesh.faces[face_idx];
        for &(a, b) in &face_edges(&face) {
            let key = if a < b { (a, b) } else { (b, a) };
            *edge_usage.entry(key).or_insert(0) += 1;
        }
    }

    let mut adjacency: HashMap<u32, Vec<u32>> = HashMap::new();
    for &face_idx in &cluster.face_indices {
        let face = mesh.faces[face_idx];
        for &(a, b) in &face_edges(&face) {
            let key = if a < b { (a, b) } else { (b, a) };
            if let Some(&count) = edge_usage.get(&key) {
                if count == 1 {
                    adjacency.entry(a).or_default().push(b);
                    adjacency.entry(b).or_default().push(a);
                }
            }
        }
    }

    let mut loops = Vec::new();

    while let Some((&start, _)) = adjacency.iter().find(|(_, v)| !v.is_empty()) {
        let mut loop_vertices = Vec::new();
        loop_vertices.push(start);
        let mut current = start;

        loop {
            let next = match adjacency
                .get_mut(&current)
                .and_then(|neighbors| neighbors.pop())
            {
                Some(next) => next,
                None => break,
            };
            remove_neighbor(&mut adjacency, next, current);
            if next == start {
                break;
            } else {
                loop_vertices.push(next);
                current = next;
            }
        }

        if loop_vertices.len() >= 3 {
            loops.push(loop_vertices);
        }
    }

    if loops.is_empty() {
        if let Some(loop_vertices) = fallback_boundary(mesh, cluster) {
            loops.push(loop_vertices);
        }
    }

    loops
}

fn remove_neighbor(adj: &mut HashMap<u32, Vec<u32>>, a: u32, b: u32) {
    if let Some(neighbors) = adj.get_mut(&a) {
        if let Some(pos) = neighbors.iter().position(|&x| x == b) {
            neighbors.swap_remove(pos);
        }
    }
}

fn fallback_boundary(mesh: &Mesh, cluster: &PlanarCluster) -> Option<Vec<u32>> {
    let mut verts = collect_unique_vertices(mesh, cluster);
    verts.sort_unstable();
    verts.dedup();
    if verts.len() >= 3 {
        Some(verts)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    #[test]
    fn tetrahedron_clusters_into_four_faces() {
        let mesh = Mesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            faces: vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        };
        let analysis = analyze_planarity(&mesh, 2.0).expect("analysis succeeds");
        assert_eq!(analysis.cluster_count(), 4);
        assert_eq!(analysis.largest_cluster_size(), 1);
    }

    #[test]
    fn tetrahedron_topology_is_watertight() {
        let mesh = Mesh {
            vertices: vec![
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 1.0, 0.0],
                [0.0, 0.0, 1.0],
            ],
            faces: vec![[0, 1, 2], [0, 1, 3], [0, 2, 3], [1, 2, 3]],
        };
        let report = analyze_topology(&mesh);
        assert!(report.is_watertight);
        assert_eq!(report.boundary_edges, 0);
        assert_eq!(report.non_manifold_edges, 0);
    }

    #[test]
    fn heal_vertices_merges_close_points() {
        let mut mesh = Mesh {
            vertices: vec![[0.0, 0.0, 0.0], [0.0001, 0.0, 0.0], [0.0, 1.0, 0.0]],
            faces: vec![[0, 1, 2]],
        };
        let report = heal_vertices(&mut mesh, 0.001).expect("heal succeeds");
        assert!(report.merged_vertices >= 1);
        assert_eq!(mesh.vertices.len(), 2);
        assert!(mesh.faces.is_empty());
    }

    #[test]
    fn planar_patch_detects_inner_loop() {
        let mesh = Mesh {
            vertices: vec![
                [-1.0, -1.0, 0.0],
                [1.0, -1.0, 0.0],
                [1.0, 1.0, 0.0],
                [-1.0, 1.0, 0.0],
                [-0.5, -0.5, 0.0],
                [0.5, -0.5, 0.0],
                [0.5, 0.5, 0.0],
                [-0.5, 0.5, 0.0],
            ],
            faces: vec![
                [0, 1, 4],
                [1, 5, 4],
                [1, 2, 5],
                [2, 6, 5],
                [2, 3, 6],
                [3, 7, 6],
                [3, 0, 7],
                [0, 4, 7],
            ],
        };
        let analysis = analyze_planarity(&mesh, 1.0).expect("planarity");
        let shell = extract_planar_patches(&mesh, &analysis, 0.01);
        let patch = &shell.patches[0];
        assert_eq!(patch.inner_boundaries.len(), 1);
        assert!(patch.outer_boundary.len() >= 4);
    }

    #[test]
    fn hole_fixture_is_watertight_after_sewing() {
        let asset = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("..")
            .join("tests")
            .join("resources")
            .join("hole_poly.stl");
        let mut mesh = mesh_io::load_stl(&asset).expect("fixture loads").mesh;
        let tol = 0.1;
        let _heal = heal_vertices(&mut mesh, tol).expect("heal succeeds");
        let analysis = analyze_planarity(&mesh, 2.0).expect("planarity");
        let shell = extract_planar_patches(&mesh, &analysis, tol);
        assert!(shell
            .patches
            .iter()
            .any(|patch| !patch.inner_boundaries.is_empty()));
        let merged = merge_planar_clusters(&mesh, &analysis, tol);
        let report = analyze_topology(&merged);
        assert!(report.is_watertight, "sewn mesh should be watertight");
    }
}

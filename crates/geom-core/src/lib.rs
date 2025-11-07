use mesh_io::Mesh;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Minimal statistics describing the DCEL that would be produced from a mesh.
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
pub struct DcelStats {
    pub vertices: usize,
    pub half_edges: usize,
    pub faces: usize,
}

impl DcelStats {
    pub fn empty() -> Self {
        Self {
            vertices: 0,
            half_edges: 0,
            faces: 0,
        }
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum TopologyError {
    #[error("cannot build DCEL from mesh with fewer than 3 vertices")]
    InsufficientVertices,
}

pub fn estimate_stats(mesh: &Mesh) -> Result<DcelStats, TopologyError> {
    if mesh.vertex_count() < 3 {
        return Err(TopologyError::InsufficientVertices);
    }

    Ok(DcelStats {
        vertices: mesh.vertex_count(),
        half_edges: mesh.face_count() * 3,
        faces: mesh.face_count(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use mesh_io::Mesh;

    #[test]
    fn estimate_stats_counts_vertices_and_faces() {
        let mesh = Mesh {
            vertices: vec![[0.0, 0.0, 0.0]; 4],
            faces: vec![[0, 1, 2], [0, 2, 3]],
        };
        let stats = estimate_stats(&mesh).expect("valid stats");
        assert_eq!(stats.vertices, 4);
        assert_eq!(stats.faces, 2);
        assert_eq!(stats.half_edges, 6);
    }

    #[test]
    fn estimate_stats_rejects_too_few_vertices() {
        let mesh = Mesh {
            vertices: vec![[0.0, 0.0, 0.0]; 2],
            faces: vec![],
        };
        assert_eq!(
            estimate_stats(&mesh).unwrap_err(),
            TopologyError::InsufficientVertices
        );
    }
}

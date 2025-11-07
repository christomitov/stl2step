use geom_core::DcelStats;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq)]
pub struct Tolerance {
    pub linear: f64,
    pub angular_deg: f64,
}

impl Default for Tolerance {
    fn default() -> Self {
        Self {
            linear: 0.1,
            angular_deg: 1.0,
        }
    }
}

#[derive(Debug, Error, PartialEq)]
pub enum BrepError {
    #[error("shell is not watertight")]
    NotWatertight,
}

pub fn derived_tolerance(stats: &DcelStats, user_tol: Option<f64>) -> Tolerance {
    if let Some(tol) = user_tol {
        return Tolerance {
            linear: tol.max(1e-4),
            angular_deg: (tol * 10.0).clamp(0.1, 5.0),
        };
    }

    let scale = (stats.faces.max(1) as f64).sqrt();
    Tolerance {
        linear: (0.05 / scale).max(1e-4),
        angular_deg: (2.0 / scale).clamp(0.25, 5.0),
    }
}

pub fn validate_shell(is_watertight: bool) -> Result<(), BrepError> {
    if is_watertight {
        Ok(())
    } else {
        Err(BrepError::NotWatertight)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn derived_tolerance_respects_user_value() {
        let stats = DcelStats {
            vertices: 100,
            half_edges: 300,
            faces: 100,
        };
        let tol = derived_tolerance(&stats, Some(0.2));
        assert_eq!(tol.linear, 0.2);
        assert!(tol.angular_deg >= 1.0);
    }

    #[test]
    fn default_tolerance_scales_with_faces() {
        let stats = DcelStats {
            vertices: 10,
            half_edges: 30,
            faces: 10,
        };
        let tol_small = derived_tolerance(&stats, None);

        let stats_dense = DcelStats {
            vertices: 10,
            half_edges: 300,
            faces: 100,
        };
        let tol_dense = derived_tolerance(&stats_dense, None);

        assert!(tol_dense.linear < tol_small.linear);
    }

    #[test]
    fn validate_shell_flags_open_geometry() {
        assert!(validate_shell(true).is_ok());
        assert_eq!(validate_shell(false).unwrap_err(), BrepError::NotWatertight);
    }
}

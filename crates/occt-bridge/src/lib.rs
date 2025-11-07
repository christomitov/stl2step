use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BridgeState {
    Disabled,
    Enabled,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BridgeStatus {
    pub state: BridgeState,
    pub notes: Option<String>,
}

impl BridgeStatus {
    pub fn disabled() -> Self {
        Self {
            state: BridgeState::Disabled,
            notes: None,
        }
    }
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum BridgeError {
    #[error("OCCT bridge requested but not compiled in")]
    MissingFeature,
}

pub fn detect() -> BridgeStatus {
    if cfg!(feature = "occt") {
        BridgeStatus {
            state: BridgeState::Enabled,
            notes: Some("feature flag enabled".into()),
        }
    } else {
        BridgeStatus::disabled()
    }
}

pub fn ensure_enabled() -> Result<(), BridgeError> {
    if cfg!(feature = "occt") {
        Ok(())
    } else {
        Err(BridgeError::MissingFeature)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn detect_reflects_feature_flag() {
        let status = detect();
        if cfg!(feature = "occt") {
            assert_eq!(status.state, BridgeState::Enabled);
        } else {
            assert_eq!(status.state, BridgeState::Disabled);
        }
    }
}

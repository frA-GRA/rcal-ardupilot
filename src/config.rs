use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct MavlinkSimConfig {
    pub sitl_address: String,
}

impl Default for MavlinkSimConfig {
    fn default() -> Self {
        Self {
            sitl_address: "udpin:0.0.0.0:14550".to_string(),
        }
    }
}

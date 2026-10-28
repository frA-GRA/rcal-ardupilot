use mavlink::dialects::ardupilotmega::{MavModeFlag, MavState};
use rcal::uci::types::*;

pub fn mav_state_to_system_state(s: MavState) -> SystemStateEnum {
    match s {
        MavState::MAV_STATE_ACTIVE => SystemStateEnum::Operational,
        MavState::MAV_STATE_STANDBY => SystemStateEnum::Inactive,
        MavState::MAV_STATE_CRITICAL | MavState::MAV_STATE_EMERGENCY => SystemStateEnum::Degraded,
        _ => SystemStateEnum::Operational,
    }
}

pub fn mav_state_to_contingency(s: MavState) -> SystemContingencyLevelEnum {
    match s {
        MavState::MAV_STATE_CRITICAL => SystemContingencyLevelEnum::Mission_critical,
        MavState::MAV_STATE_EMERGENCY => SystemContingencyLevelEnum::Flight_critical,
        _ => SystemContingencyLevelEnum::Normal,
    }
}

pub fn is_armed(base_mode: MavModeFlag) -> bool {
    base_mode.contains(MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED)
}

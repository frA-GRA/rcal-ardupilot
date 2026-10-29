pub mod config;
pub mod translate;

use std::sync::{Arc, Mutex};
use std::time::Duration;

use slog::{debug, error, info, warn};

use rcal::cal::{AbstractCal, AbstractWriter, TopicQos};
use rcal::calconfig::CalConfig;
use rcal::service::{AbstractService, AbstractServiceImpl, ServiceLifecycleState};
use rcal::uci::base::UUID;
use rcal::uci::types::*;
use rcal::uci::CalResult;
use rcal::update_message_header;

use config::MavlinkSimConfig;
use translate::{is_armed, mav_state_to_contingency, mav_state_to_system_state};

use mavlink::dialects::ardupilotmega::MavMessage;
use mavlink::error::MessageReadError;
use mavlink::MavConnection;

// Snapshot of MAVLink state shared between the receiver and publishers.
#[derive(Clone)]
struct SimState {
    system_state: SystemStateEnum,
    contingency:  SystemContingencyLevelEnum,
    armed:        bool,
    battery_pct:  i8, // 0-100, -1 = unknown

    lat_rad: Option<f64>,
    lon_rad: Option<f64>,
    alt_m:   Option<f32>,
    vx_ms:   Option<f32>,
    vy_ms:   Option<f32>,
    vz_ms:   Option<f32>,

    roll_rad:  Option<f32>,
    pitch_rad: Option<f32>,
    yaw_rad:   Option<f32>,
}

impl Default for SimState {
    fn default() -> Self {
        Self {
            system_state: SystemStateEnum::Inactive,
            contingency:  SystemContingencyLevelEnum::Normal,
            armed:        false,
            battery_pct:  -1,
            lat_rad:      None,
            lon_rad:      None,
            alt_m:        None,
            vx_ms:        None,
            vy_ms:        None,
            vz_ms:        None,
            roll_rad:     None,
            pitch_rad:    None,
            yaw_rad:      None,
        }
    }
}

pub struct MavlinkSimService {
    lifecycle:     Box<dyn AbstractService>,
    pos_writer:    Option<Box<dyn AbstractWriter<PositionReport_>>>,
    sys_writer:    Option<Box<dyn AbstractWriter<SystemStatus_>>>,
    nav_writer:    Option<Box<dyn AbstractWriter<NavigationReport_>>>,
    fcap_writer:   Option<Box<dyn AbstractWriter<FlightCapabilityStatus_>>>,
    fact_writer:   Option<Box<dyn AbstractWriter<FlightActivity_>>>,
    subsys_writer: Option<Box<dyn AbstractWriter<SubsystemStatus_>>>,
    svc_writer:    Option<Box<dyn AbstractWriter<ServiceStatus_>>>,

    pos_tmpl:    PositionReport_,
    sys_tmpl:    SystemStatus_,
    nav_tmpl:    NavigationReport_,
    fcap_tmpl:   FlightCapabilityStatus_,
    fact_tmpl:   FlightActivity_,
    subsys_tmpl: SubsystemStatus_,
    svc_msg:     ServiceStatus_,

    sim_config:    MavlinkSimConfig,
    logger:        slog::Logger,
    task_handles:  Vec<tokio::task::JoinHandle<()>>,
}

#[rcal_macros::rcal_trace]
impl MavlinkSimService {
    pub fn new<A>(
        service_name: &str,
        cal: A,
        cal_config: Arc<CalConfig>,
        logger: slog::Logger,
    ) -> CalResult<Self>
    where
        A: AbstractCal + 'static,
    {
        let sim_config: MavlinkSimConfig =
            cal_config.get_extension("rcal_sim").unwrap_or_default();

        let system_uuid = cal_config.system.uuid;
        let service_uuid = cal_config
            .get_service(service_name)
            .and_then(|s| s.uuid)
            .unwrap_or_else(UUID::generate_v4);

        let capability_id = UUID::generate_v3(&service_uuid, b"flight_capability");
        let activity_id   = UUID::generate_v3(&service_uuid, b"flight_activity");
        let subsystem_id  = UUID::generate_v3(&system_uuid, b"autopilot_subsystem");

        let mut svc = AbstractServiceImpl::new(
            service_name,
            cal_config.system.id.clone(),
            vec![],
            cal,
            Arc::clone(&cal_config),
            logger.clone(),
        );

        let pos_writer    = svc.create_writer::<PositionReport_>("PositionReport", TopicQos::default())?;
        let sys_writer    = svc.create_writer::<SystemStatus_>("SystemStatus", TopicQos::default())?;
        let nav_writer    = svc.create_writer::<NavigationReport_>("NavigationReport", TopicQos::default())?;
        let fcap_writer   = svc.create_writer::<FlightCapabilityStatus_>("FlightCapabilityStatus", TopicQos::default())?;
        let fact_writer   = svc.create_writer::<FlightActivity_>("FlightActivity", TopicQos::default())?;
        let subsys_writer = svc.create_writer::<SubsystemStatus_>("SubsystemStatus", TopicQos::default())?;
        let svc_writer    = svc.create_writer::<ServiceStatus_>("ServiceStatus", TopicQos::default())?;

        // ── PositionReport template ──────────────────────────────────────────
        let mut pos_tmpl = svc.create_message::<PositionReport_>()?;
        *pos_tmpl.message_data_mut().system_id_mut().uuid_mut() = system_uuid;
        pos_tmpl.message_data_mut().system_id_mut().descriptive_label_set(cal_config.system.id.clone());
        *pos_tmpl.message_data_mut().source_mut() = SystemSourceEnum::Actual;
        *pos_tmpl.message_data_mut().current_operating_domain_mut() = EnvironmentEnum::Air;

        // ── SystemStatus template ────────────────────────────────────────────
        let mut sys_tmpl = svc.create_message::<SystemStatus_>()?;
        *sys_tmpl.message_data_mut().system_id_mut().uuid_mut() = system_uuid;
        sys_tmpl.message_data_mut().system_id_mut().descriptive_label_set(cal_config.system.id.clone());
        *sys_tmpl.message_data_mut().source_mut() = SystemSourceEnum::Actual;
        sys_tmpl.message_data_mut().service_id_mut().resize(1);
        *sys_tmpl.message_data_mut().service_id_mut()[0].uuid_mut() = service_uuid;
        sys_tmpl.message_data_mut().service_id_mut()[0].descriptive_label_set(service_name.into());

        // ── NavigationReport template ────────────────────────────────────────
        let mut nav_tmpl = svc.create_message::<NavigationReport_>()?;
        *nav_tmpl.message_data_mut().system_id_mut().uuid_mut() = system_uuid;
        nav_tmpl.message_data_mut().system_id_mut().descriptive_label_set(cal_config.system.id.clone());
        *nav_tmpl.message_data_mut().source_mut() = SystemSourceEnum::Actual;
        *nav_tmpl.message_data_mut().contingency_level_mut() = SystemContingencyLevelEnum::Normal;

        // ── FlightCapabilityStatus template ─────────────────────────────────
        let mut fcap_tmpl = svc.create_message::<FlightCapabilityStatus_>()?;
        fcap_tmpl.message_data_mut().capability_status_mut().resize(1);
        *fcap_tmpl.message_data_mut().capability_status_mut()[0].capability_id_mut().uuid_mut() = capability_id;
        fcap_tmpl.message_data_mut().capability_status_mut()[0]
            .capability_id_mut()
            .descriptive_label_set("FlightCapability".into());
        *fcap_tmpl.message_data_mut().capability_status_mut()[0].availability_mut() =
            AvailabilityEnum::Available;

        // ── FlightActivity template ──────────────────────────────────────────
        let mut fact_tmpl = svc.create_message::<FlightActivity_>()?;
        fact_tmpl.message_data_mut().activity_mut().resize(1);
        *fact_tmpl.message_data_mut().activity_mut()[0].activity_id_mut().uuid_mut() = activity_id;
        fact_tmpl.message_data_mut().activity_mut()[0]
            .capability_id_mut()
            .resize(1);
        *fact_tmpl.message_data_mut().activity_mut()[0].capability_id_mut()[0].uuid_mut() =
            capability_id;
        *fact_tmpl.message_data_mut().activity_mut()[0].interactive_mut() = false;
        *fact_tmpl.message_data_mut().activity_mut()[0].activity_state_mut() =
            ActivityStateEnum::Enabled;

        // ── SubsystemStatus template ─────────────────────────────────────────
        let mut subsys_tmpl = svc.create_message::<SubsystemStatus_>()?;
        *subsys_tmpl.message_data_mut().subsystem_id_mut().uuid_mut() = subsystem_id;
        subsys_tmpl
            .message_data_mut()
            .subsystem_id_mut()
            .descriptive_label_set("Autopilot".into());
        *subsys_tmpl.message_data_mut().subsystem_state_mut() = SubsystemStateEnum::Operate;
        *subsys_tmpl.message_data_mut().about_mut().model_mut() = "ArduCopter-SITL".into();

        // ── ServiceStatus message ────────────────────────────────────────────
        let mut svc_msg = svc.create_message::<ServiceStatus_>()?;
        *svc_msg.message_data_mut().service_state_mut() = ServiceStateEnum::Normal;
        *svc_msg.message_data_mut().service_id_mut().uuid_mut() = service_uuid;
        svc_msg.message_data_mut().service_id_mut().descriptive_label_set(service_name.into());

        Ok(Self {
            lifecycle: Box::new(svc),
            pos_writer:    Some(pos_writer),
            sys_writer:    Some(sys_writer),
            nav_writer:    Some(nav_writer),
            fcap_writer:   Some(fcap_writer),
            fact_writer:   Some(fact_writer),
            subsys_writer: Some(subsys_writer),
            svc_writer:    Some(svc_writer),
            pos_tmpl,
            sys_tmpl,
            nav_tmpl,
            fcap_tmpl,
            fact_tmpl,
            subsys_tmpl,
            svc_msg,
            sim_config,
            logger,
            task_handles: Vec::new(),
        })
    }
}

#[rcal_macros::rcal_trace]
impl AbstractService for MavlinkSimService {
    fn system_id(&self) -> &str { self.lifecycle.system_id() }
    fn service_id(&self) -> &str { self.lifecycle.service_id() }
    fn subsystem_ids(&self) -> &[String] { self.lifecycle.subsystem_ids() }
    fn lifecycle_state(&self) -> ServiceLifecycleState { self.lifecycle.lifecycle_state() }

    fn activate(&mut self) -> CalResult<()> {
        self.lifecycle.activate()?;

        let state = Arc::new(Mutex::new(SimState::default()));

        // ── MAVLink receiver ─────────────────────────────────────────────────
        let sitl_addr = self.sim_config.sitl_address.clone();
        let recv_state = Arc::clone(&state);
        let recv_logger = self.logger.clone();
        let recv_handle = tokio::task::spawn_blocking(move || {
            let conn = std::sync::Arc::new(loop {
                match mavlink::connect::<MavMessage>(&sitl_addr) {
                    Ok(c) => break c,
                    Err(e) => {
                        warn!(recv_logger, "MAVLink connect failed, retrying in 2s"; "addr" => &sitl_addr, "error" => %e);
                        std::thread::sleep(Duration::from_secs(2));
                    }
                }
            });
            info!(recv_logger, "MAVLink connected"; "addr" => &sitl_addr);

            // SITL won't stream telemetry until it sees a GCS heartbeat.
            let hb_conn = std::sync::Arc::clone(&conn);
            let hb_logger = recv_logger.clone();
            std::thread::spawn(move || {
                use mavlink::dialects::ardupilotmega as apm;
                let header = mavlink::MavHeader { system_id: 255, component_id: 190, sequence: 0 };
                let hb = MavMessage::HEARTBEAT(apm::HEARTBEAT_DATA {
                    custom_mode: 0,
                    mavtype: apm::MavType::MAV_TYPE_GCS,
                    autopilot: apm::MavAutopilot::MAV_AUTOPILOT_INVALID,
                    base_mode: apm::MavModeFlag::empty(),
                    system_status: apm::MavState::MAV_STATE_ACTIVE,
                    mavlink_version: 3,
                });
                loop {
                    if let Err(e) = hb_conn.send(&header, &hb) {
                        warn!(hb_logger, "heartbeat send failed"; "error" => %e);
                    }
                    std::thread::sleep(Duration::from_secs(1));
                }
            });

            loop {
                match conn.recv() {
                    Ok((_, msg)) => update_sim_state(&recv_state, msg, &recv_logger),
                    Err(MessageReadError::Io(ref io_err))
                        if io_err.kind() == std::io::ErrorKind::WouldBlock =>
                    {
                        std::thread::sleep(Duration::from_millis(10));
                    }
                    Err(e) => {
                        warn!(recv_logger, "MAVLink recv error"; "error" => %e);
                        std::thread::sleep(Duration::from_millis(100));
                    }
                }
            }
        });
        self.task_handles.push(recv_handle);

        // ── ServiceStatus (constant, 1 Hz) ───────────────────────────────────
        let mut svc_writer = self.svc_writer.take().expect("svc_writer already taken");
        let mut svc_msg = self.svc_msg.clone();
        let svc_logger = self.logger.clone();
        self.task_handles.push(tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
                update_message_header!(svc_msg);
                if let Err(e) = svc_writer.write(&svc_msg) {
                    error!(svc_logger, "ServiceStatus write failed"; "error" => %e);
                }
            }
        }));

        // ── SystemStatus (1 Hz, dynamic state) ──────────────────────────────
        let mut sys_writer = self.sys_writer.take().expect("sys_writer already taken");
        let mut sys_msg = self.sys_tmpl.clone();
        let sys_state = Arc::clone(&state);
        let sys_logger = self.logger.clone();
        self.task_handles.push(tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
                {
                    let s = sys_state.lock().unwrap();
                    *sys_msg.message_data_mut().system_state_mut() = s.system_state.clone();
                }
                update_message_header!(sys_msg);
                if let Err(e) = sys_writer.write(&sys_msg) {
                    error!(sys_logger, "SystemStatus write failed"; "error" => %e);
                }
            }
        }));

        // ── NavigationReport (1 Hz) ──────────────────────────────────────────
        let mut nav_writer = self.nav_writer.take().expect("nav_writer already taken");
        let mut nav_msg = self.nav_tmpl.clone();
        let nav_state = Arc::clone(&state);
        let nav_logger = self.logger.clone();
        self.task_handles.push(tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
                {
                    let s = nav_state.lock().unwrap();
                    *nav_msg.message_data_mut().contingency_level_mut() = s.contingency.clone();
                    if s.battery_pct >= 0 {
                        nav_msg
                            .message_data_mut()
                            .endurance_mut()
                            .percent_set(s.battery_pct as f64);
                    }
                }
                update_message_header!(nav_msg);
                if let Err(e) = nav_writer.write(&nav_msg) {
                    error!(nav_logger, "NavigationReport write failed"; "error" => %e);
                }
            }
        }));

        // ── FlightCapabilityStatus (1 Hz) ────────────────────────────────────
        let mut fcap_writer = self.fcap_writer.take().expect("fcap_writer already taken");
        let mut fcap_msg = self.fcap_tmpl.clone();
        let fcap_state = Arc::clone(&state);
        let fcap_logger = self.logger.clone();
        self.task_handles.push(tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
                {
                    let s = fcap_state.lock().unwrap();
                    *fcap_msg.message_data_mut().capability_status_mut()[0].availability_mut() =
                        if s.armed {
                            AvailabilityEnum::Available
                        } else {
                            AvailabilityEnum::Unavailable
                        };
                }
                update_message_header!(fcap_msg);
                if let Err(e) = fcap_writer.write(&fcap_msg) {
                    error!(fcap_logger, "FlightCapabilityStatus write failed"; "error" => %e);
                }
            }
        }));

        // ── FlightActivity (1 Hz) ────────────────────────────────────────────
        let mut fact_writer = self.fact_writer.take().expect("fact_writer already taken");
        let mut fact_msg = self.fact_tmpl.clone();
        let fact_state = Arc::clone(&state);
        let fact_logger = self.logger.clone();
        self.task_handles.push(tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
                {
                    let s = fact_state.lock().unwrap();
                    *fact_msg.message_data_mut().activity_mut()[0].activity_state_mut() =
                        if s.armed {
                            ActivityStateEnum::Active_unconstrained
                        } else {
                            ActivityStateEnum::Enabled
                        };
                }
                update_message_header!(fact_msg);
                if let Err(e) = fact_writer.write(&fact_msg) {
                    error!(fact_logger, "FlightActivity write failed"; "error" => %e);
                }
            }
        }));

        // ── SubsystemStatus (1 Hz) ───────────────────────────────────────────
        let mut subsys_writer = self.subsys_writer.take().expect("subsys_writer already taken");
        let mut subsys_msg = self.subsys_tmpl.clone();
        let subsys_state = Arc::clone(&state);
        let subsys_logger = self.logger.clone();
        self.task_handles.push(tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
                {
                    let s = subsys_state.lock().unwrap();
                    *subsys_msg.message_data_mut().subsystem_state_mut() =
                        if matches!(s.system_state, SystemStateEnum::Degraded) {
                            SubsystemStateEnum::Degraded
                        } else {
                            SubsystemStateEnum::Operate
                        };
                }
                update_message_header!(subsys_msg);
                if let Err(e) = subsys_writer.write(&subsys_msg) {
                    error!(subsys_logger, "SubsystemStatus write failed"; "error" => %e);
                }
            }
        }));

        // ── PositionReport (1 Hz, skip until GPS fix) ────────────────────────
        let mut pos_writer = self.pos_writer.take().expect("pos_writer already taken");
        let mut pos_msg = self.pos_tmpl.clone();
        let pos_state = Arc::clone(&state);
        let pos_logger = self.logger.clone();
        self.task_handles.push(tokio::spawn(async move {
            loop {
                tokio::time::sleep(Duration::from_secs(1)).await;
                let has_fix = {
                    let s = pos_state.lock().unwrap();
                    if let (Some(lat), Some(lon), Some(alt)) = (s.lat_rad, s.lon_rad, s.alt_m) {
                        {
                            let pt = pos_msg.message_data_mut().inertial_state_mut().position_mut();
                            *pt.latitude_mut()  = lat;
                            *pt.longitude_mut() = lon;
                            *pt.altitude_mut()  = alt as f64;
                        }
                        if let (Some(vx), Some(vy)) = (s.vx_ms, s.vy_ms) {
                            let is = pos_msg.message_data_mut().inertial_state_mut();
                            is.ground_velocity_enable();
                            if let Some(gv) = is.ground_velocity_mut() {
                                *gv.north_speed_mut() = vx as f64;
                                *gv.east_speed_mut()  = vy as f64;
                            }
                        }
                        if let (Some(roll), Some(pitch), Some(yaw)) =
                            (s.roll_rad, s.pitch_rad, s.yaw_rad)
                        {
                            let is = pos_msg.message_data_mut().inertial_state_mut();
                            is.orientation_enable();
                            if let Some(ori) = is.orientation_mut() {
                                *ori.yaw_mut()   = yaw as f64;
                                *ori.pitch_mut() = pitch as f64;
                                *ori.roll_mut()  = roll as f64;
                            }
                        }
                        true
                    } else {
                        false
                    }
                };
                if !has_fix {
                    continue;
                }
                update_message_header!(pos_msg);
                if let Err(e) = pos_writer.write(&pos_msg) {
                    error!(pos_logger, "PositionReport write failed"; "error" => %e);
                }
            }
        }));

        info!(self.logger, "MavlinkSimService activated");
        Ok(())
    }

    fn deactivate(&mut self) -> CalResult<()> {
        for h in self.task_handles.drain(..) {
            h.abort();
        }
        self.lifecycle.deactivate()?;
        info!(self.logger, "MavlinkSimService deactivated");
        Ok(())
    }

    fn reset(&mut self) -> CalResult<()> {
        self.deactivate()?;
        self.lifecycle.reset()
    }
}

fn update_sim_state(state: &Mutex<SimState>, msg: MavMessage, logger: &slog::Logger) {
    match msg {
        MavMessage::HEARTBEAT(hb) => {
            let new_state   = mav_state_to_system_state(hb.system_status);
            let new_cont    = mav_state_to_contingency(hb.system_status);
            let new_armed   = is_armed(hb.base_mode);
            debug!(logger, "HEARTBEAT"; "armed" => new_armed, "mav_status" => ?hb.system_status);
            let mut s = state.lock().unwrap();
            if new_armed != s.armed {
                info!(logger, "arm state changed"; "armed" => new_armed);
            }
            if std::mem::discriminant(&new_state) != std::mem::discriminant(&s.system_state) {
                info!(logger, "system state changed";
                    "from" => ?s.system_state, "to" => ?new_state,
                    "mav_status" => ?hb.system_status);
            }
            s.system_state = new_state;
            s.contingency  = new_cont;
            s.armed        = new_armed;
        }
        MavMessage::SYS_STATUS(ss) => {
            debug!(logger, "SYS_STATUS"; "battery_pct" => ss.battery_remaining);
            let mut s = state.lock().unwrap();
            if s.battery_pct < 0 && ss.battery_remaining >= 0 {
                info!(logger, "battery status acquired"; "pct" => ss.battery_remaining);
            }
            s.battery_pct = ss.battery_remaining;
        }
        MavMessage::GLOBAL_POSITION_INT(gp) => {
            let lat_deg = gp.lat as f64 * 1e-7;
            let lon_deg = gp.lon as f64 * 1e-7;
            let alt_m   = gp.alt as f32 * 1e-3;
            debug!(logger, "GLOBAL_POSITION_INT";
                "lat_deg" => lat_deg, "lon_deg" => lon_deg, "alt_m" => alt_m,
                "vx_cms" => gp.vx, "vy_cms" => gp.vy, "vz_cms" => gp.vz);
            let mut s = state.lock().unwrap();
            if s.lat_rad.is_none() {
                info!(logger, "GPS fix acquired";
                    "lat_deg" => lat_deg, "lon_deg" => lon_deg, "alt_m" => alt_m);
            }
            s.lat_rad = Some(lat_deg.to_radians());
            s.lon_rad = Some(lon_deg.to_radians());
            s.alt_m   = Some(alt_m);
            s.vx_ms   = Some(gp.vx as f32 * 0.01);
            s.vy_ms   = Some(gp.vy as f32 * 0.01);
            s.vz_ms   = Some(gp.vz as f32 * 0.01);
        }
        MavMessage::ATTITUDE(att) => {
            debug!(logger, "ATTITUDE";
                "roll_rad" => att.roll, "pitch_rad" => att.pitch, "yaw_rad" => att.yaw);
            let mut s = state.lock().unwrap();
            s.roll_rad  = Some(att.roll);
            s.pitch_rad = Some(att.pitch);
            s.yaw_rad   = Some(att.yaw);
        }
        _ => {}
    }
}

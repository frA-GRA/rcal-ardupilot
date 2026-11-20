use std::sync::Arc;

use slog::error;

use rcal::cal::get_cal;
use rcal::service::AbstractService;

use rcal_ardupilot::MavlinkSimService;

#[rcal_macros::rcal_main]
async fn main() {
    let service_name = "rcal_sim";
    let rcal_config = Arc::new(rcal_config);

    let cal = get_cal(
        service_name,
        None::<String>,
        rcal_config.clone(),
        root_logger.clone(),
    )
    .await
    .unwrap_or_else(|err| {
        error!(root_logger, "CAL init failed"; "error" => %err);
        println!("CAL init failed. Error: {}", err);
        std::process::exit(1);
    });

    let mut svc =
        MavlinkSimService::new(service_name, cal, rcal_config.clone(), root_logger.clone())
            .unwrap_or_else(|err| {
                error!(root_logger, "MavlinkSimService init failed"; "error" => %err);
                std::process::exit(2);
            });

    if let Err(e) = svc.activate() {
        error!(root_logger, "MavlinkSimService activate failed"; "error" => %e);
        std::process::exit(3);
    }

    tokio::signal::ctrl_c().await.ok();
    svc.stop().await;

    if let Err(e) = svc.deactivate() {
        error!(root_logger, "MavlinkSimService deactivate failed"; "error" => %e);
    }
    std::process::exit(0);
}

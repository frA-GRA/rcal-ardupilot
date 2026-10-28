# rcal-ardupilot

MAVLink-to-UCI gateway connecting ArduPilot SITL to the [rcal](https://github.com/tclarke/rcal) Common Abstraction Layer.

## Overview

`rcal-sim` bridges ArduPilot SITL via MAVLink (UDP/TCP) into the rcal service ecosystem, exposing simulated UAV telemetry and control through the UCI interface.

## Requirements

- Rust 1.75+
- Running ArduPilot SITL instance (or `docker-compose up`)

## Usage

```bash
# Start ArduPilot SITL + gateway
docker-compose up

# Or run the gateway directly against an existing SITL
cargo run -- --config CALConfig.toml
```

## Configuration

Copy `CALConfig.toml` and adjust MAVLink connection strings and rcal service endpoints as needed.

## License

Licensed under either of:

- MIT License ([LICENSE-MIT](LICENSE-MIT))
- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))

at your option.

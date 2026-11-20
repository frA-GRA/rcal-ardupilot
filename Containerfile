# Builder image
# 
FROM rust:slim-bookworm AS builder

COPY . /build/
WORKDIR /build
RUN RCAL_CALCONFIG_PATH=/build/docker/CALConfig.docker.toml RCAL_XSD_PATH=/build/docker/A-GRA_MessageDefinitions_v6_0_a.xsd cargo build --release --bins

# Final image
# 
FROM debian:bookworm-slim
WORKDIR /app
COPY docker/CALConfig.docker.toml /app/CALConfig.toml
COPY --from=builder /build/target/release/rcal_ardupilot /app/

CMD ["./rcal_ardupilot"]

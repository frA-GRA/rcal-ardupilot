# syntax=docker/dockerfile:1
# Build: docker build -t rcal-ardupilot .
#
# Optional build contexts:
#   --build-context schema=<dir>  directory holding the UCI .xsd to build against
#                                 (default: rcal's bundled schema)
#   --build-context rcal=<dir>    local rcal crate dir (rcal/rcal) to use in place of crates.io

FROM scratch AS schema
FROM scratch AS rcal

FROM rust:slim-bookworm AS builder
WORKDIR /build
COPY --from=schema / /schema/
COPY --from=rcal / /rcal/
COPY Cargo.toml Cargo.lock build.rs ./
COPY src ./src
RUN if [ -f /rcal/Cargo.toml ]; then \
      printf '\n[patch.crates-io]\nrcal = { path = "/rcal" }\nrcal_macros = { path = "/rcal/rcal_macros" }\n' >> Cargo.toml; \
    fi
RUN xsd=$(ls /schema/*.xsd 2>/dev/null | head -n 1); \
    env ${xsd:+RCAL_XSD_PATH=$xsd} cargo build --release --bin rcal_ardupilot

FROM debian:bookworm-slim
WORKDIR /app
COPY --from=builder /build/target/release/rcal_ardupilot /app/rcal_ardupilot
COPY CALConfig.toml /app/CALConfig.toml
ENV RCAL_CALCONFIG_PATH=/app/CALConfig.toml
CMD ["/app/rcal_ardupilot"]

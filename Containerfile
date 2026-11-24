# Builder image
#
FROM rust:slim-bookworm AS builder

COPY . /build/
WORKDIR /build
# rcal 3.0.1 on crates.io doesn't compile against A-GRA (fixed in rcal#130); use the submodule until released
COPY --from=rcal . /rcal
RUN printf '\n[patch.crates-io]\nrcal = { path = "/rcal" }\nrcal_macros = { path = "/rcal/rcal_macros" }\n' >> Cargo.toml
RUN RCAL_CALCONFIG_PATH=/build/docker/CALConfig.docker.toml RCAL_XSD_PATH=/build/docker/A-GRA_MessageDefinitions_v6_0_a.xsd cargo build --release --bins

# Final image
# 
FROM debian:bookworm-slim
WORKDIR /app
COPY docker/CALConfig.docker.toml /app/CALConfig.toml
COPY --from=builder /build/target/release/rcal_ardupilot /app/

CMD ["./rcal_ardupilot"]

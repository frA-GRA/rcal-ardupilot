# Stage 1: build arducopter SITL binary from source
FROM ubuntu:24.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        git python3 python3-pip python3-pexpect \
        gcc g++ make cmake binutils \
        libcurl4-openssl-dev python3-empy \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ardupilot
RUN git clone --depth=1 --branch ArduCopter-stable \
        https://github.com/ArduPilot/ardupilot.git . \
    && git submodule update --init --recursive --depth=1

RUN python3 waf configure --board sitl && python3 waf copter

# Stage 2: minimal Python runtime
FROM python:3-slim

RUN pip install --no-cache-dir mavproxy empy==3.3.4

COPY --from=builder /ardupilot/build/sitl/bin/arducopter /usr/local/bin/arducopter
COPY docker/sitl/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 5760/tcp

WORKDIR /run/sitl
ENTRYPOINT ["/entrypoint.sh"]

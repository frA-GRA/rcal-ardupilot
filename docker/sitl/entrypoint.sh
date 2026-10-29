#!/bin/sh
set -e
mkdir -p /run/sitl
cd /run/sitl
# arducopter SITL listens as TCP server on port 5760; rcal-sim connects as client.
exec arducopter \
    -S \
    --model + \
    --speedup 1 \
    --home -35.363261,149.165230,584,353 \
    --instance 0 \
    --uartA tcp:5760:wait

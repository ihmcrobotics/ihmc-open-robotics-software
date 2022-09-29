#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

docker run \
    --tty \
    --interactive \
    --rm \
    --network host \
    --dns=1.1.1.1 \
    --privileged \
    --gpus all \
    --device /dev/dri:/dev/dri \
    ihmcrobotics/nvidia:0.4 bash

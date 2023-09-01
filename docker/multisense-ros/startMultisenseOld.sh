#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

xhost +local:docker

docker run \
--tty \
--rm \
--interactive \
--network host \
--dns 1.1.1.1 \
--env "DISPLAY" \
--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged \
--gpus all \
--device /dev/dri:/dev/dri \
ihmcrobotics/multisense-sl:0.1

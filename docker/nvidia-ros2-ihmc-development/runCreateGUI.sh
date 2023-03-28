#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

xhost +local:docker

docker run \
    --tty \
    --interactive \
    --network host \
    --dns=1.1.1.1 \
    --privileged \
    --gpus all \
    --device /dev/dri:/dev/dri \
    --env DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --name perception \
    ihmcrobotics/nvidia-ros2-ihmc-development:0.3

#!/bin/bash
# Uncomment for debugging this script
set -o xtrace

# Make sure it works one way or the other to reduce possible errors
if (( EUID == 0 )); then
    echo "Run without sudo." 1>&2
    exit 1
fi

sudo -u $(whoami) xhost +local:docker

sudo -u root docker run \
    --tty \
    --interactive \
    --rm \
    --network host \
    --dns=1.1.1.1 \
    --privileged \
    --gpus all \
    --device /dev/dri:/dev/dri \
    --env DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    ihmcrobotics/nvidia-ros2-ihmc-development:0.3

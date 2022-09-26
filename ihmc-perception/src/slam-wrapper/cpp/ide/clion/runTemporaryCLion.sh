#!/bin/bash
# Uncomment for debugging this script
set -o xtrace

# Make sure it works one way or the other to reduce possible errors
if (( EUID == 0 )); then
    echo "Run without sudo." 1>&2
    exit 1
fi

xhost +local:docker

docker run \
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
    --volume $DOCKER_JETBRAINS_CONFIG_HOME:/home/robotlab/.config/JetBrains:rw \
    --volume $DOCKER_WORKSPACE:/home/robotlab/dev/workspace:rw \
    --volume /usr/share/fonts:/usr/share/fonts \
    ihmcrobotics/slam-wrapper-clion:0.1 clion

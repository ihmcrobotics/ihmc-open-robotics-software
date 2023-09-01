#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

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
    --volume $DOCKER_WORKSPACE:/home/robotlab/dev/catkin_ws:rw \
    --volume /usr/share/fonts:/usr/share/fonts \
    ihmcrobotics/nvidia-ros-ide:0.3 $1

#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

xhost +local:docker

if [ ! "$(docker ps -a | grep ' multisense$')" ]; then
    echo "multisense not found. Running new container."
    docker run \
    --tty \
    --interactive \
    --name multisense \
    --network host \
    --dns 1.1.1.1 \
    --env "DISPLAY" \
    --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume "$HOME"/dev/multisense_ws:/home/robotlab/dev/multisense_ws:rw \
    --privileged \
    --gpus all \
    --device /dev/dri:/dev/dri \
    ihmcrobotics/multisense-ros:0.2
else
    docker start --attach multisense
fi
#!/bin/bash
# Uncomment for debugging this script
set -o xtrace

# Make sure it works one way or the other to reduce possible errors
if (( EUID == 0 )); then
    echo "Run without sudo." 1>&2
    exit 1
fi

sudo -u $(whoami) xhost +local:docker

if [ ! "$(sudo -u root docker ps -a | grep multisense)" ]; then
    echo "multisense not found. Running new container."
    sudo -u root docker run \
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
    sudo -u root docker start --attach multisense
fi
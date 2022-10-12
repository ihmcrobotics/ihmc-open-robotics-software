#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

xhost +local:docker

# readlink -f provides an absolute path from a relative path, since docker
# will not accept relative paths in a volume command
# Setting TERM=xterm-256color enables color in the terminal
if [ ! "$(docker ps -a | grep ' nvidia-ros2$')" ]; then
    echo "nvidia-ros2 not found. Running new container."
    docker run \
        --tty \
        --interactive \
        --name nvidia-ros2 \
        --network host \
        --dns=1.1.1.1 \
        --privileged \
        --gpus all \
        --device /dev/dri:/dev/dri \
        --env DISPLAY \
        --env "TERM=xterm-256color" \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        --volume $(readlink -f ../../ihmc-interfaces/src/main/messages/ihmc_interfaces):/home/robotlab/colcon_ws/src \
        ihmcrobotics/nvidia-ros2:0.3
else
    docker start --attach nvidia-ros2
fi

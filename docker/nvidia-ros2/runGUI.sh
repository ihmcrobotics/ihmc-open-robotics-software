#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

xhost +local:docker

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
        --env "TERM=xterm-256color" `# Enable color in the terminal` \
        --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
        `# Provides an absolute path from a relative path, since docker` \
        `# will not accept relative paths in a volume command` \
        --volume $(readlink -f ../../ihmc-interfaces/src/main/messages/ihmc_interfaces):/home/robotlab/colcon_ws/src \
        ihmcrobotics/nvidia-ros2:0.4
else
    docker start --attach nvidia-ros2
fi

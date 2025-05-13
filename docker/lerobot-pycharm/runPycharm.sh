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
    --dns=1.1.1.1 \
    --env "TERM=xterm-256color" `# Enable color in the terminal` \
    --privileged \
    --gpus all \
    --device /dev/dri:/dev/dri \
    --env DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume /home/$USER/.config/JetBrainsDocker:/home/robotlab/.config/JetBrains:rw \
    --volume /home/$USER/dev/lerobot:/home/robotlab/dev/lerobot \
    --volume /usr/share/fonts:/usr/share/fonts \
    ihmcrobotics/lerobot-pycharm:0.1 bash

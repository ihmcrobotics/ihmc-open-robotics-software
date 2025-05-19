#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

docker run \
    --tty \
    --interactive \
    --rm \
    --dns=1.1.1.1 \
    --env "TERM=xterm-256color" `# Enable color in the terminal` \
    --privileged \
    --gpus all \
    --volume /home/$USER/.config/JetBrainsDocker:/home/robotlab/.config/JetBrains:rw \
    --volume /home/$USER/dev/lerobot:/home/robotlab/dev/lerobot \
    ihmcrobotics/lerobot:0.1 bash

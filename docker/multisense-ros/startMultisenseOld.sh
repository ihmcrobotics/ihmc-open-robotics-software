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
--rm \
--interactive \
--network host \
--dns 1.1.1.1 \
--env "DISPLAY" \
--volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--privileged \
--gpus all \
--device /dev/dri:/dev/dri \
ihmcrobotics/multisense-sl:0.1

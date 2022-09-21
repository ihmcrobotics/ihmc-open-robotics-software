#!/bin/bash
set -e -o xtrace

# Make sure it has been run as sudo, needed for Docker
if (( EUID == 0 )); then
    echo "Run with sudo." 1>&2
    exit 1
fi

sudo -u root docker run \
    --tty \
    --interactive \
    --rm \
    --network host \
    --dns=1.1.1.1 \
    ihmcrobotics/slam-wrapper:0.1 bash

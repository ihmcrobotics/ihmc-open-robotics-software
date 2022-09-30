#!/bin/bash
set -e -o xtrace

# Make sure it has been run as sudo, needed for Docker
#if (( EUID != 0 )); then
#    echo "Run without sudo." 1>&2
#    exit 1
#fi

docker run \
    --tty \
    --interactive \
    --rm \
    --network host \
    --dns=1.1.1.1 \
    ihmcrobotics/slam-wrapper:0.2 bash

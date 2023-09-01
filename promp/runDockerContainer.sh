#!/bin/bash
set -e -o xtrace

docker run \
    --tty \
    --interactive \
    --rm \
    --network host \
    --dns=1.1.1.1 \
    ihmcrobotics/promp:0.2 bash
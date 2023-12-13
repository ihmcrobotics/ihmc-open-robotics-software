#!/bin/bash
set -e -o xtrace

docker run \
    --tty \
    --interactive \
    --rm \
    --volume $(pwd):/home/robotlab/slam-wrapper \
    --workdir /home/robotlab/slam-wrapper \
    --user $(id -u):$(id -g) \
    ihmcrobotics/slam-wrapper:0.5 bash

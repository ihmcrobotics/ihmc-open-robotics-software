#!/bin/bash
set -e -o xtrace

docker run \
    --tty \
    --interactive \
    --rm \
    --volume $(pwd):/home/robotlab/mapsense-sandbox \
    --workdir /home/robotlab/mapsense-sandbox \
    ihmcrobotics/nvidia-mapsense-sandbox:0.1 bash

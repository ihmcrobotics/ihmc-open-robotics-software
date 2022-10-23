#!/bin/bash
set -e -o xtrace

docker run \
    --rm \
    --volume $(pwd):/home/robotlab/mapsense-sandbox \
    --workdir /home/robotlab/mapsense-sandbox \
    --user $(id -u):$(id -g) \
    ihmcrobotics/mapsense-nvidia-sandbox:0.1 bash /home/robotlab/mapsense-sandbox/generateJavaMappings.sh

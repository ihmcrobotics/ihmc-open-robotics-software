#!/bin/bash
set -e -o xtrace

docker run \
    --volume $(pwd):/home/robotlab/mapsense-sandbox \
    --workdir /home/robotlab/mapsense-sandbox \
    ihmcrobotics/nvidia-mapsense-sandbox:0.1 bash /home/robotlab/mapsense-sandbox/generateJavaMappings.sh

#!/bin/bash
set -e -o xtrace

docker run \
    --rm \
    --volume $(pwd):/home/robotlab/slam-wrapper \
    --workdir /home/robotlab/slam-wrapper \
    --user $(id -u):$(id -g) \
    ihmcrobotics/slam-wrapper:0.3 bash /home/robotlab/slam-wrapper/generateJavaMappings.sh

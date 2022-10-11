#!/bin/bash
set -e -o xtrace

docker run \
    --rm \
    --volume $(pwd):/home/robotlab/promp \
    --workdir /home/robotlab/promp \
    --user $(id -u):$(id -g) \
    ihmcrobotics/promp:0.2 bash /home/robotlab/promp/generateJavaMappings.sh
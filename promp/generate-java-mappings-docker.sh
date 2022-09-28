#!/bin/bash
set -e -o xtrace

docker run \
    --rm \
    --volume $(pwd):/root/dev/promp \
    --workdir /root/dev/promp \
    ihmcrobotics/promp:0.1 bash /root/dev/promp/generate-java-mappings.sh
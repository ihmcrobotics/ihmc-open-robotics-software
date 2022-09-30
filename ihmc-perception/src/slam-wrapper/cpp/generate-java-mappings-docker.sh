#!/bin/bash
set -e -o xtrace

# Make sure it has been run as sudo, needed for Docker
# if (( EUID == 0 )); then
#     echo "Run with sudo." 1>&2
#     exit 1
# fi

docker run \
    --rm \
    --volume $(pwd)/..:/root/dev/slam-wrapper \
    --workdir /root/dev/slam-wrapper/cpp \
    ihmcrobotics/slam-wrapper:0.2 bash /root/dev/slam-wrapper/cpp/generate-java-mappings.sh

#!/bin/bash

# Run this if you want to rebuild all the layers
# docker build --no-cache -t foundationpose-ros2:0.0.3 . 

# Run this if you want to rebuild all the layers with log output
# docker build --no-cache --progress=plain -f Dockerfile.zed2 -t foundationpose-ros2:0.0.3 . 2>&1 | tee docker_build.log

# Run this if you don't want to rebuild all the layers
docker build -t foundationpose-ros2:0.0.3 .


random_id=$(uuidgen | cut -c-8)

# Build centerpose with GPUs
docker run -it --name "foundationpose-build-$random_id" --network=host --runtime=nvidia --gpus all --mount type=bind,source=.,target=/root/foundationpose-ros2 foundationpose-ros2:0.0.3 bash compile_foundationpose.sh

# Commit
docker commit "foundationpose-build-$random_id" foundationpose-ros2:0.0.3

# Remove the intermediate build container
docker rm "foundationpose-build-$random_id"
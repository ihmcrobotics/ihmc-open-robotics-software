#!/bin/bash
docker build --no-cache -t centerpose-ros2:0.0.1 . # Run this if you want to rebuild all the layers
#docker build -t centerpose-ros2:0.0.1 .

random_id=$(uuidgen | cut -c-8)

# Build centerpose with GPUs
docker run -it --name "centerpose-build-$random_id" --network=host --runtime=nvidia --gpus all --mount type=bind,source=.,target=/root/centerpose-ros2 centerpose-ros2:0.0.1 bash compile_centerpose.sh

# Commit
docker commit "centerpose-build-$random_id" centerpose-ros2:0.0.1

# Remove the intermediate build container
docker rm "centerpose-build-$random_id"
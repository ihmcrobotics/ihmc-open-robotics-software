#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

xhost +local:docker

docker run \
    --tty \
    --interactive \
    --rm \
    --network host \
    --dns=1.1.1.1 \
    --privileged \
    --runtime=nvidia \
    --gpus all \
    --device /dev/dri:/dev/dri \
    --env DISPLAY \
    --env "ACCEPT_EULA=Y" \
    --env "PRIVACY_CONSENT=Y" \
    --name isaac-sim \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume ~/docker/isaac-sim/cache/kit:/isaac-sim/kit/cache:rw \
    --volume ~/docker/isaac-sim/cache/ov:/root/.cache/ov:rw \
    --volume ~/docker/isaac-sim/cache/pip:/root/.cache/pip:rw \
    --volume ~/docker/isaac-sim/cache/glcache:/root/.cache/nvidia/GLCache:rw \
    --volume ~/docker/isaac-sim/cache/computecache:/root/.nv/ComputeCache:rw \
    --volume ~/docker/isaac-sim/logs:/root/.nvidia-omniverse/logs:rw \
    --volume ~/docker/isaac-sim/data:/root/.local/share/ov/data:rw \
    --volume ~/docker/isaac-sim/documents:/root/Documents:rw \
    --entrypoint bash \
    nvcr.io/nvidia/isaac-sim:4.2.0

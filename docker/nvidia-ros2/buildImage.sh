#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

# Copy messages into this directory just so we can copy them into the build
rm -rf ihmc_interfaces
cp -r ../../ihmc-interfaces/src/main/messages/ihmc_interfaces .

docker build --tag ihmcrobotics/nvidia-ros2:0.4 .

# Clean up
rm -rf ihmc_interfaces

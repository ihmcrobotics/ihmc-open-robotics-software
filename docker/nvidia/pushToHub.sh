#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

docker login

docker push ihmcrobotics/nvidia:0.6

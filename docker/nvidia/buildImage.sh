#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

docker build --tag ihmcrobotics/nvidia:0.6 .

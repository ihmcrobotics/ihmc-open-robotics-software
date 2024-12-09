#!/bin/bash
set -e -o xtrace

# NOTE: the version of the image should be updated when the image is updated
docker build -t ihmc-external-wrapper:0.1 .
#!/bin/bash
set -e -o xtrace

docker run \
  --rm \
  --volume $(pwd):/home/robotlab/ihmc-external-wrapper \
  --workdir /home/robotlab/ihmc-external-wrapper \
  ihmc-external-wrapper:0.1 bash -c """
  /home/robotlab/ihmc-external-wrapper/scripts/buildWrapper.sh --test && \
  /home/robotlab/ihmc-external-wrapper/scripts/generateBindings.sh
  """
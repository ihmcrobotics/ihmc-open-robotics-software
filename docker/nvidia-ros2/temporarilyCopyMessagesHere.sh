#!/bin/bash
# Immediately exit on any errors.
set -e
# Print commands as they are run.
set -o xtrace

rm -rf ihmc_interfaces
cp -r ../../ihmc-interfaces/src/main/messages/ihmc_interfaces .

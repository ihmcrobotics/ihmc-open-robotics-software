#!/bin/bash
# Immediately exit on any errors.
set -e

# Setup the ROS environment.
source "/opt/ros/$ROS_DISTRO/setup.bash"
source colcon_ws/install/setup.bash

# Takes any command line arguments passed to ros_entrypoint.sh and execs them as a command.
# The intention is basically "Do everything in this .sh script, then in the same shell
# run the command the user passes in on the command line".
# Taken from https://stackoverflow.com/a/39082923/1070333
exec "$@"

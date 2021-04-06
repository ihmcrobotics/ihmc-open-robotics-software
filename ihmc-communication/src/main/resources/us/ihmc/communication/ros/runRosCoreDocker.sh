#!/bin/bash
set -o xtrace

function cleanup() {
	sudo -u root docker kill roscore
}

sudo -u root docker run \
   --name roscore \
   --rm \
   --tty \
   --network host \
   --env ROS_MASTER_URI=http://localhost:11311 \
   ros:noetic-robot-focal \
   roscore &

DOCKER_ROSCORE_PID=$!

trap "cleanup" INT TERM KILL
wait $DOCKER_ROSCORE_PID
trap - INT TERM KILL

#!/bin/bash
set -o xtrace

# Make sure it works one way or the other to reduce possible errors
if (( EUID == 0 )); then
    echo "Run without sudo." 1>&2
    exit 1
fi

sudo -u $(whoami) xhost +local:docker

OS_NAME=$(awk -F= '/^NAME/{print $2}' /etc/os-release)
OS_NAME="${OS_NAME%\"}"
OS_NAME="${OS_NAME#\"}"

echo "OS: $OS_NAME"

function cleanup() {
	sudo -u root docker kill mapsense
}

if [ ! "$(sudo -u root docker ps -a | grep mapsense$)" ]; then
  if [[ "$OS_NAME" == *"Arch"* ]]; then
    sudo -u root docker run \
      --name mapsense \
      --tty \
      --network host \
      --env "DISPLAY" \
      --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --volume "$HOME"/dev/mapsense/SharedVolume:/home/robotlab/SharedVolume:rw \
      --volume "$HOME"/dev/mapsense/srcHeadless:/home/robotlab/dev/mapsense_ws/src:rw \
      --privileged \
      --gpus all \
      --device /dev/dri:/dev/dri \
      ihmcrobotics/mapsense-nvidia-ros:0.5 /home/robotlab/dev/mapsense_ws/src/MapSenseROS/scripts/startMapSenseHeadless.sh &
  else
    sudo -u root docker run \
      --name mapsense \
      --tty \
      --network host \
      --env "DISPLAY" \
      --volume "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --volume "$HOME"/dev/mapsense/SharedVolume/home/robotlab/SharedVolume:rw \
      --volume "$HOME"/dev/mapsense/srcHeadless:/home/robotlab/dev/mapsense_ws/src:rw \
      --privileged \
      --runtime nvidia \
      --gpus all \
      --device /dev/dri:/dev/dri \
      ihmcrobotics/mapsense-nvidia-ros:0.5 /home/robotlab/dev/mapsense_ws/src/MapSenseROS/scripts/startMapSenseHeadless.sh &
  fi
else
  sudo -u root docker start --attach mapsense &
fi

DOCKER_MAPSENSE_PID=$!

trap "cleanup" INT TERM KILL
wait $DOCKER_MAPSENSE_PID
trap - INT TERM KILL

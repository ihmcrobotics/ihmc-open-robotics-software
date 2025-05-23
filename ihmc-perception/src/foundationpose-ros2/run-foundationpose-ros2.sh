#!/bin/sh

if [ -z "${ROS_DOMAIN_ID+x}" ]; then
  echo "Set ROS_DOMAIN_ID before running this script"
  exit 1
fi

docker run -it \
  --runtime=nvidia \
  --network host \
  --gpus all \
  --privileged \
  --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --env FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  --workdir /root/foundationpose-ros2 \
  --mount type=bind,source=.,target=/root/foundationpose-ros2 \
  ihmc/foundationpose:latest \
  bash -c "
  
  source /opt/ros/humble/setup.bash
  source /root/ihmc_ros2_ws/install/setup.bash
  export PYTHONPATH=/root/FoundationPose:\$PYTHONPATH
  echo Using ROS_DOMAIN_ID=$ROS_DOMAIN_ID
  echo Current topic list:
  ros2 topic list
  echo
  ros2 daemon stop
  ros2 daemon start
  python foundationpose_ros2_node.py
  "

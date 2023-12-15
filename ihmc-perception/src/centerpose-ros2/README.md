# CenterPose ROS2 Nodes

## CenterPose ROS2 Docker Image
This docker image contains CenterPose, it's dependencies, and ROS2. The ROS2 nodes below utilize it. Build the docker image with `./build_centerpose_ros2_image.sh`.

Installation directories:
```
/root/ihmc_ros2_ws # The IHMC ROS2 interfaces get built and installed to here; source this to use IHMC ROS2 interfaces
/root/centerpose-install # The CenterPose library; cloned from https://github.com/NVlabs/CenterPose
/root/centerpose-ros2 # This directory ([...]/ihmc-perception/src/centerpose-ros2) mounted into the volume
```

## ZED2
With a ZED2 camera connected, run ZEDColorStereoDepthPublisher first, then run `./run_zed2_centerpose_node_docker.sh`. The ROS2 node will publish the poses detected from CenterPose.

## Intel RealSense
TODO?

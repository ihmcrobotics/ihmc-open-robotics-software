# CenterPose ROS2 Nodes

## CenterPose ROS2 Docker Image
This docker image contains CenterPose, it's dependencies, and ROS2. The ROS2 nodes below utilize it. Build the docker image with `./build_centerpose_ros2_image.sh`.

## ZED2
With a ZED2 camera connected, run ZEDColorStereoDepthPublisher first, then run `./run_zed2_centerpose_node_docker.sh`. The ROS2 node will publish the poses detected from CenterPose.

## Intel RealSense
TODO?

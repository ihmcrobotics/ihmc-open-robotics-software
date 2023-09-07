# CenterPose ROS2 Nodes
Download pretrained models from here https://drive.google.com/drive/folders/16HbCnUlCaPcTg4opHP_wQNPsWouUlVZe

Put the model files in a `models` directory inside this directory. For example:
```
centerpose_ros2/
  models/
    chair_v1_140.pth
    book_v1_140.pth
    ...
```

## CenterPose ROS2 Docker Image
This docker image contains CenterPose, it's dependencies, and ROS2. The ROS2 nodes below utilize it. Build the docker image with `./build_centerpose_ros2_image.sh`.

## ZED2
With a ZED2 camera connected, run ZEDColorStereoDepthPublisher first, then run `./run_zed2_centerpose_node_docker.sh`. The ROS2 node will publish the poses detected from CenterPose.

## Intel RealSense
TODO?

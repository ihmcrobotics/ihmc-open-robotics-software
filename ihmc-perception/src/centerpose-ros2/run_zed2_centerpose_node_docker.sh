#!/bin/bash
bash download_pretrained_models.sh
xhost +si:localuser:root  # allow containers to communicate with X server
docker run -it --network=host --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --runtime=nvidia --gpus all -v centerpose-cache:/root/.cache --mount type=bind,source=.,target=/root/centerpose-ros2 centerpose-ros2:0.0.1 bash /root/centerpose-ros2/zed2/run_zed2_centerpose_node.sh
# python3 demo.py --demo /root/centerpose-ros2/zed2/IMG_1132.jpg --arch dlav1_34 --load_model /root/centerpose-ros2/models/cup_mug_v1_140.pth --debug 4
#!/bin/bash
bash download_pretrained_models.sh
docker run -it --network=host --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID --runtime=nvidia --gpus all -v centerpose-cache:/root/.cache --mount type=bind,source=.,target=/root/centerpose-ros2 centerpose-ros2:0.0.1 bash /root/centerpose-ros2/zed2/run_zed2_centerpose_node.sh
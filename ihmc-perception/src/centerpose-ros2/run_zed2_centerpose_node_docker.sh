#!/bin/bash
bash download_pretrained_models.sh
docker run -it --network=host --rm --runtime=nvidia --gpus all --mount type=bind,source=.,target=/home/CenterPose_ws centerpose-ros2:0.0.1 bash /home/CenterPose_ws/zed2/run_zed2_centerpose_node.sh
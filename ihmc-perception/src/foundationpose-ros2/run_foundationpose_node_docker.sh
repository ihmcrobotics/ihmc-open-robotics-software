#!/bin/bash
bash download_pretrained_models.sh
#xhost +si:localuser:root  # allow containers to communicate with X server
# for ((i=1;i<=$#;i++)); do
#     echo "Spawing a detector for $i: ${!i} model"
#     #docker run -it --network=host --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --runtime=nvidia --gpus all -v foundationpose-cache:/root/.cache --mount type=bind,source=.,target=/root/foundationpose-ros2 foundationpose-ros2:0.0.1 bash /root/foundationpose-ros2/zed2/run_zed2_foundationpose_node.sh
#     docker run -it --network=host --rm -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID --runtime=nvidia --gpus all -v foundationpose-cache:/root/.cache --mount type=bind,source=.,target=/root/foundationpose-ros2 foundationpose-ros2:0.0.1 bash /root/foundationpose-ros2/zed2/run_zed2_foundationpose_node.sh ${!i} &
# done


random_id=$(uuidgen | cut -c-8)

xhost +local:root

docker run -it \
  --name "foundationpose-build-$random_id" \
  --network=host \
  --ipc=host \
  --runtime=nvidia \
  --gpus all \
  --env DISPLAY=$DISPLAY \
  --env QT_X11_NO_MITSHM=1 \
  --env ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume /dev:/dev \
  --mount type=bind,source=.,target=/root/foundationpose-ros2 \
  foundationpose-ros2:0.0.1 \
  bash -c "source /opt/ros/humble/setup.bash && python3 foundationpose_ros_multi.py"
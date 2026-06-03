#!/bin/sh

echo Ensure you have at least 20-25GB of RAM free before running this build script. Continuing in 5 seconds.

sleep 5

docker build . -t ihmc/foundationpose # Add --no-cache to rebuild the entire Dockerfile

##
# Below are things we want to run every time we build and do not want cached
##

# FoundationPose build_all.sh requires GPUs, we cannot run this step in docker build, only docker run
random_id=$(uuidgen | cut -c-8)
docker run -it --name "foundationpose-build-$random_id" --runtime=nvidia --gpus all ihmc/foundationpose:latest \
	bash -c "
	bash /root/foundationpose-ros2/FoundationPose/build_all.sh
	"
docker commit "foundationpose-build-$random_id" ihmc/foundationpose
docker rm "foundationpose-build-$random_id"

# Build the ihmc msgs
random_id=$(uuidgen | cut -c-8)
docker run -it --name "foundationpose-build-$random_id" ihmc/foundationpose:latest \
	bash -c "
	mkdir -p /root/ihmc_ros2_ws
	cd /root/ihmc_ros2_ws
	git clone -b feature/foundation-pose-integration https://github.com/ihmcrobotics/ihmc-open-robotics-software.git /root/ihmc-open-robotics-software
	mkdir -p src
	cp -r /root/ihmc-open-robotics-software/ihmc-interfaces-jros2/messages/ihmc_interfaces ./src/ihmc_interfaces
	rm -rf /root/ihmc-open-robotics-software
	source /opt/ros/humble/setup.bash && cd /root/ihmc_ros2_ws
	colcon build
	"
docker commit "foundationpose-build-$random_id" ihmc/foundationpose
docker rm "foundationpose-build-$random_id"

# Download weights
random_id=$(uuidgen | cut -c-8)
docker run --workdir "/root/FoundationPose" -it --name "foundationpose-build-$random_id" ihmc/foundationpose:latest \
	bash -c "
	pip install gdown
	apt-get install -y unzip
	mkdir -p weights
	gdown 1cyI3wKcdWAWyXZZrsVmhcLLk9qfL_fRi -O weights/2024-01-11-20-02-45.zip
	unzip -o weights/2024-01-11-20-02-45.zip -d weights/
	gdown 15gBTLShNNPRoYJwRwkOB_neeWZy-34Vb -O weights/2023-10-28-18-33-37.zip
	unzip -o weights/2023-10-28-18-33-37.zip -d weights/
	"
docker commit "foundationpose-build-$random_id" ihmc/foundationpose
docker rm "foundationpose-build-$random_id"

# NVLabs centerpose_ros2 version 0.0.1

FROM nvcr.io/nvidia/cuda:11.6.2-devel-ubuntu20.04
LABEL Name=centerpose_ros2 Version=0.0.1

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip python3-dev locales software-properties-common curl git

RUN pip install numba progress matplotlib scipy albumentations pyrr simplejson tqdm opencv-python-headless numpy scikit-learn scipy filterpy frozendict tensorflow requests

RUN pip install torch==1.12.1+cu116 --extra-index-url https://download.pytorch.org/whl/cu116

# Install ROS -----------------------------------------------------------
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

RUN add-apt-repository universe

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt-get update
RUN apt-get install -y --no-install-recommends ros-galactic-ros-base

RUN apt-get install -y --no-install-recommends python3-colcon-common-extensions

# Setup ros2 ws --------------------------------------------------------------------
WORKDIR /home/ros2ws
RUN git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git /ihmc-openihmc-open-robotics-software
RUN mkdir src
RUN cp -r /ihmc-openihmc-open-robotics-software/ihmc-interfaces/src/main/messages/ihmc_interfaces /home/ros2ws/src/ihmc_interfaces
RUN rm -rf /ihmc-openihmc-open-robotics-software
RUN . /opt/ros/galactic/setup.sh && \
    colcon build

# Setting up CenterPose -----------------------------------------------------------------------
ENV CenterPose_ROOT=/home/CenterPose
RUN git clone -b replace_linear_assignment https://github.com/ihmcrobotics/CenterPose.git $CenterPose_ROOT
RUN rm -r $CenterPose_ROOT/src/lib/models/networks/DCNv2
RUN git clone https://github.com/ihmcrobotics/DCNv2_latest.git $CenterPose_ROOT/src/lib/models/networks/DCNv2

WORKDIR /home/CenterPose_ws

# NVLabs centerpose-ros2 version 0.0.1
# Do not upload to dockerhub

FROM nvcr.io/nvidia/cuda:11.6.2-devel-ubuntu20.04
LABEL Name=centerpose-ros2 Version=0.0.1

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y --no-install-recommends python3-pip python3-dev locales software-properties-common curl git && \
    rm -rf /var/lib/apt/lists/*
RUN pip install numba progress matplotlib scipy albumentations pyrr simplejson tqdm opencv-python-headless numpy scikit-learn scipy filterpy frozendict tensorflow requests
RUN pip install torch==1.12.1+cu116 --extra-index-url https://download.pytorch.org/whl/cu116

# Install ROS
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN add-apt-repository universe
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-galactic-ros-base python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# Install IHMC ROS2 interfaces
WORKDIR /root/ihmc_ros2_ws
RUN git clone https://github.com/ihmcrobotics/ihmc-open-robotics-software.git /opt/ihmc-open-robotics-software
RUN mkdir src
RUN cp -r /opt/ihmc-open-robotics-software/ihmc-interfaces/src/main/messages/ihmc_interfaces /root/ihmc_ros2_ws/src/ihmc_interfaces
RUN rm -rf /opt/ihmc-open-robotics-software
RUN . /opt/ros/galactic/setup.sh && colcon build

# Install CenterPose
ENV CENTERPOSE_INSTALL_DIR=/root/centerpose-install
RUN git clone -b replace_linear_assignment https://github.com/ihmcrobotics/CenterPose.git $CENTERPOSE_INSTALL_DIR && \
    rm -rf $CENTERPOSE_INSTALL_DIR/.git
RUN rm -r $CENTERPOSE_INSTALL_DIR/src/lib/models/networks/DCNv2
RUN git clone https://github.com/ihmcrobotics/DCNv2_latest.git $CENTERPOSE_INSTALL_DIR/src/lib/models/networks/DCNv2 && \
    rm -rf $CENTERPOSE_INSTALL_DIR/src/lib/models/networks/DCNv2/.git

# Cleanup to reduce image size
RUN apt-get remove curl git -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /root/centerpose-ros2

# FROM osrf/ros2:ardent-full
FROM ubuntu:xenial

#Set locales
RUN apt-get clean &&\
    apt-get update &&\
    apt-get install -y locales &&\
    locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 &&\
    export LANG=en_US.UTF-8

# Add ROS sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' &&\
    apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install prerequisites for building ROS libraries
RUN apt-get update && apt-get install -y \
    git \
    wget \
    build-essential \
    cppcheck \
    cmake \
    libopencv-dev \
    python-empy \
    python3-dev \
    python3-empy \
    python3-nose \
    python3-pip \
    python3-pyparsing \
    python3-setuptools \
    python3-vcstool \
    python3-yaml \
    libtinyxml-dev \
    libeigen3-dev \
    libasio-dev \
    libtinyxml2-dev &&\
    rm -rf /var/lib/apt/lists/* &&\
    pip3 install argcomplete

RUN mkdir -p /root/ros2_ws

WORKDIR /root/ros2_ws

#RUN wget https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos

COPY ./ros1-bridge.repos /root/ros2_ws/

RUN mkdir src

RUN vcs-import src < ros1-bridge.repos

RUN src/ament/ament_tools/scripts/ament.py build --symlink-install --skip-packages ros1_bridge

# COPY ./ros1_bridge_entrypoint.sh /

CMD ["bash"]
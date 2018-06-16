FROM ubuntu:xenial

# Install packages for establishing locales
RUN apt-get clean &&\
    apt-get update &&\
    apt-get install -y locales &&\
    locale-gen en_US en_US.UTF-8 &&\
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 &&\
    export LANG=en_US.UTF-8

# Set locale
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8
ENV SHELL /bin/bash

# install curl
RUN apt-get install -y curl

# Add ROS 2 apt sources
RUN curl http://repo.ros2.org/repos.key | apt-key add -

RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list'

# Add ROS 1 apt sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' &&\
           apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install build dependencies
RUN apt-get update && apt-get install -y ros-ardent-ros1-bridge build-essential cppcheck cmake \
                                         libopencv-dev python-empy python3-dev python3-empy python3-nose \
                                         python3-pip python3-pyparsing python3-setuptools python3-vcstool \
                                         python3-pytest python3-yaml libtinyxml-dev libeigen3-dev \
                                         libasio-dev libtinyxml2-dev git

# Set up ROS 2 sources workspace
RUN mkdir -p /root/ros2_ws

WORKDIR /root/ros2_ws

# download .yaml with required repos to rebuild latest release of ROS 2
RUN curl -O https://raw.githubusercontent.com/ros2/ros2/release-latest/ros2.repos

RUN mkdir src

# Clone ROS 2 repos
RUN vcs-import src < ros2.repos

# Don't build rviz since we're in a headless environment
RUN touch src/ros2/rviz/AMENT_IGNORE

# Build everything BUT the ros1_bridge
RUN src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --skip-packages ros1_bridge

# TODO mount the ROS 1 and ROS 2 versions of any custom messages that need to be in the bridge in their own workspaces

# TODO build the ROS 1 and ROS 2 workspaces containing the relevant messages

# Rebuild the ros1_bridge; sourcing ROS 1 and ROS 2 environments first so that messages are picked up
# TODO source the environments for the additional ROS 1 and ROS 2 workspaces containing custom messages
# The final version of this will: Source kinetic, source ardent, source the ROS 1 workspace w/ messages, source the ROS 2 workspace w/ messages,
# then finally locally source the ros2_ws and rebuild the bridge.
RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
                  && source /opt/ros/ardent/setup.bash \
                  && source /root/ros2_ws/install/local_setup.bash \
                  && cd /root/ros2_ws \
                  && src/ament/ament_tools/scripts/ament.py build --symlink-install --only ros1_bridge --force-cmake-configure -j1"

CMD ["/bin/bash"]
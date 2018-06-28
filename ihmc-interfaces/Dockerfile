FROM ubuntu:xenial as ros2_build_base

ARG ROS_VERSION_ARG

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
ENV ROS_VERSION $ROS_VERSION_ARG

# install curl
RUN apt-get install -y curl

# Add ROS 2 apt sources
RUN curl http://repo.ros2.org/repos.key | apt-key add -

RUN sh -c 'echo "deb [arch=amd64,arm64] http://repo.ros2.org/ubuntu/main xenial main" > /etc/apt/sources.list.d/ros2-latest.list'

# Add ROS 1 apt sources
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list' &&\
           apt-key adv --keyserver ha.pool.sks-keyservers.net --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116

# Install build dependencies
RUN apt-get update && apt-get install -y ros-${ROS_VERSION}-ros1-bridge build-essential cppcheck cmake \
                                         libopencv-dev python-empy python3-dev python3-empy python3-nose \
                                         python3-pip python3-pyparsing python3-setuptools python3-vcstool \
                                         python3-pytest python3-yaml libtinyxml-dev libeigen3-dev \
                                         libasio-dev libtinyxml2-dev git

# Set up ROS 2 sources workspace
RUN mkdir -p /root/ros2_ws

WORKDIR /root/ros2_ws

# download .yaml with required repos to rebuild latest release of ROS 2
RUN curl -O https://raw.githubusercontent.com/ros2/ros2/release-${ROS_VERSION}/ros2.repos

RUN mkdir src

# Clone ROS 2 repos
RUN vcs-import src < ros2.repos

# Don't build rviz since we're in a headless environment
RUN touch src/ros2/rviz/AMENT_IGNORE

# Build everything BUT the ros1_bridge
RUN src/ament/ament_tools/scripts/ament.py build --build-tests --symlink-install --skip-packages ros1_bridge

# Next Build Stage
FROM ros2_build_base as ros2_ihmc_base

ARG ROS_VERSION_ARG

ENV ROS_VERSION ${ROS_VERSION_ARG}

COPY src/main/messages/ihmc_interfaces/controller_msgs /root/ihmc_ros2_ws/src/controller_msgs

WORKDIR /root/ihmc_ros2_ws/

RUN /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash \
                  && source /root/ros2_ws/install/local_setup.bash\
                  && ament build --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Setup Runtime
FROM ros2_ihmc_base as ros2_ihmc_runtime

ENV ROS_VERSION ardent

COPY ./runtime_entrypoint.sh /

RUN ["chmod", "+x", "/runtime_entrypoint.sh"]

RUN /bin/bash -c "echo 'source /runtime_entrypoint.sh' >> /root/.bashrc"

ENTRYPOINT []

CMD ["/bin/bash"]
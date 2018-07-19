FROM osrf/ros2:ardent-ros1-bridge

RUN apt-get update && \
    apt-get install -y curl git python-catkin-pkg python3-vcstool ros-${ROS2_DISTRO}-ament-tools && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y ros-${ROS2_DISTRO}-ros1-bridge build-essential cppcheck cmake \
                                         libopencv-dev python-empy python3-catkin-pkg-modules \
                                         python3-dev python3-empy python3-nose \
                                         python3-pip python3-pyparsing python3-setuptools python3-vcstool \
                                         python3-pytest python3-yaml libtinyxml-dev libeigen3-dev \
                                         libasio-dev libtinyxml2-dev git curl

RUN mkdir -p /root/ihmc_ros1_bridge/src

WORKDIR /root/ihmc_ros1_bridge

# Add controller_msgs
COPY src/main/messages/ihmc_interfaces/controller_msgs src/controller_msgs

RUN /bin/bash -c "source /opt/ros/$ROS2_DISTRO/setup.bash \
    && ament build --symlink-install --only controller_msgs"

# Add ihmc demos (build separately to save time)
COPY src/ihmc_demo_py src/ihmc_demo_py

RUN /bin/bash -c "source /opt/ros/$ROS2_DISTRO/setup.bash \
    && ament build --symlink-install --only ihmc_demo_py"

RUN mkdir -p /root/ihmc_catkin_ws/src

WORKDIR /root/ihmc_catkin_ws/src

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
    && catkin_init_workspace"

WORKDIR /root/ihmc_catkin_ws

COPY src/main/messages/ros1/controller_msgs src/controller_msgs

RUN /bin/bash -c "source /opt/ros/kinetic/setup.bash \
    && catkin_make && catkin_make install"

WORKDIR /root/ihmc_ros1_bridge
# Add ros1_bridge
COPY src/rosdocker/linux/ihmc_ros1_bridge.repos .
RUN vcs import src < ihmc_ros1_bridge.repos

ENV MAKEFLAGS=-j1

RUN /bin/bash -c "source /opt/ros/$ROS2_DISTRO/setup.bash \
               && source /root/ihmc_ros1_bridge/install/local_setup.bash \
               && source /opt/ros/kinetic/setup.bash \
               && source /root/ihmc_catkin_ws/install/setup.bash \
               && ament build --symlink-install --only ros1_bridge --force-cmake-configure"

COPY ./runtime_entrypoint.sh /

RUN ["chmod", "+x", "/runtime_entrypoint.sh"]

RUN /bin/bash -c "echo 'source /runtime_entrypoint.sh' >> /root/.bashrc"

CMD ["/bin/bash"]
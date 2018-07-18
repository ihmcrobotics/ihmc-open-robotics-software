FROM osrf/ros2:bouncy-ros1-bridge

RUN apt-get update && \
    apt-get install -y curl git python3-colcon-common-extensions python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

RUN mkdir -p /root/ihmc_ros1_bridge/src

WORKDIR /root/ihmc_ros1_bridge

# Add controller_msgs
COPY src/main/messages/ihmc_interfaces/controller_msgs src/controller_msgs

RUN /bin/bash -c "source /opt/ros/$ROS2_DISTRO/setup.bash \
    && colcon build --symlink-install --packages-select controller_msgs --event-handlers console_direct+"

# Add ihmc demos (build separately to save time)
COPY src/ihmc_demo_py src/ihmc_demo_py

RUN /bin/bash -c "source /opt/ros/$ROS2_DISTRO/setup.bash \
    && colcon build --symlink-install --packages-select ihmc_demo_py --event-handlers console_direct+"

# Add ros1_bridge
COPY src/rosdocker/linux/ihmc_ros1_bridge.repos .
RUN vcs import src < ihmc_ros1_bridge.repos

COPY ./runtime_entrypoint.sh /

RUN ["chmod", "+x", "/runtime_entrypoint.sh"]

RUN /bin/bash -c "echo 'source /runtime_entrypoint.sh' >> /root/.bashrc"

CMD ["/bin/bash"]
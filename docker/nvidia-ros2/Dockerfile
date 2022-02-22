# An image for developing GPU accelerated applications on NVIDIA hardware with ROS 2.
# This image is published to https://hub.docker.com/r/ihmcrobotics/nvidia-ros2
# Current version: 0.1
# This file is mostly taken from https://github.com/osrf/docker_images/blob/master/ros/rolling/ubuntu/focal/ros-core/Dockerfile
# We aren't yet sure how to get around copying this code.
FROM ihmcrobotics/nvidia:0.4

# Setup ROS 2 Rolling
USER root

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO rolling

# If interactive, apt-get will prompt for keyboard configuration
RUN apt-get --quiet 2 --yes update \
 && DEBIAN_FRONTEND=noninteractive \
    apt-get --quiet 2 --yes install \
    keyboard-configuration \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

RUN echo "deb http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2-latest.list

RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    ros-rolling-desktop \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /home/robotlab/ros_entrypoint.sh

RUN chmod a+x /home/robotlab/ros_entrypoint.sh

USER robotlab
WORKDIR /home/robotlab

# Commands passed to i.e. 'docker run nvidia-ros [...]' will go through this entrypoint.
# Therefore, if you pass i.e. 'docker run nvidia-ros rviz', then it will have passed
# 'rviz' to this script as an argument, and, in this case, the result will be that rviz
# will be run with the ROS environment already sourced/setup.
ENTRYPOINT ["/home/robotlab/ros_entrypoint.sh"]
# Default command to run, i.e. 'docker run nvidia-ros' will be as if you ran 'docker run nvidia-ros bash'.
# If you specify a command, i.e. 'docker run nvidia-ros roscore', this won't be run.
CMD ["bash"]

# Installation of IHMCPerception

*Requirement: Ubuntu 14.04 Trusty & ROS Indigogo*

## 1. Install Third-Party Software

### ihmc_ros

> cd ~/<WORKSPACE NAME>/IHMCPerception/catkin_ws/src
> git clone https://bitbucket.org/ihmcrobotics/ihmc_ros.git

### libnabo (for ethz_icp_mapper)

> sudo add-apt-repository ppa:stephane.magnenat/trusty
> sudo apt-get update
> sudo apt-get install libnabo-dev

### libpointmatcher (for ethz_icp_mapper)

(Not yet available for Trusty yet: https://launchpad.net/~stephane.magnenat/+archive/ubuntu/trusty)

> cd ~/<WORKSPACE NAME>/IHMCPerception/third-party/libpointmatcher/
> mkdir build && cd build
> cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ..
> make -j8
> sudo make install

### Kindr (for elevation_mapping)

> cd ~/<WORKSPACE NAME>/IHMCPerception/third-party/kindr/
> mkdir build
> cd build
> cmake ..
> make
> sudo make install

### moveit_ros_perception (for lidar_to_point_cloud_transformer)

> sudo apt-get install ros-indigo-moveit-ros-perception
> sudo apt-get install ros-indigo-moveit-ros-planning

## 2. Setup Catkin Workspace

> cd ~/<WORKSPACE NAME>/IHMCPerception/catkin_ws/
> catkin_make

Add this to your ~/.bashrc file:
> source /opt/ros/indigo/setup.bash
> source ~/workspace/IHMCPerception/catkin_ws/devel/setup.bash
> export ROS_PACKAGE_PATH=~/workspace/DarpaRoboticsChallenge/ModelGenerator:$ROS_PACKAGE_PATH

Close your console and open a new one for the next steps.

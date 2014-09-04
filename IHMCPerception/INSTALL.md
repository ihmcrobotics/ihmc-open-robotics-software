# Installation of IHMCPerception

*Requirement: Ubuntu 14.04 Trusty & ROS Indigogo*

## 1. Install Third-Party Software

### libnabo (for ethz_icp_mapper)

> sudo add-apt-repository ppa:stephane.magnenat/trusty
> sudo apt-get update
> sudo apt-get install libnabo-dev

### libpointmatcher (for ethz_icp_mapper)

(Not yet available for Trusty: https://launchpad.net/~stephane.magnenat/+archive/ubuntu/trusty)

Download: wget https://launchpad.net/~stephane.magnenat/+archive/ubuntu/raring/+files/libpointmatcher1_1.1.0_amd64.deb

> sudo dpkg -i libpointmatcher1_1.1.0_amd64.deb 

## 2. Setup Catkin Workspace

> cd ~/<WORKSPACE NAME>/IHMCPerception/catkin_ws/
> catkin_make

Add this to your ~/.bashrc file:
> source /opt/ros/indigo/setup.bash
> source ~/workspace/IHMCPerception/catkin_ws/devel/setup.bash
> export ROS_PACKAGE_PATH=~/workspace/DarpaRoboticsChallenge/ModelGenerator:$ROS_PACKAGE_PATH

Close your console and open a new one for the next steps.




# Update third-party software

### ethz_icp_mapper

>

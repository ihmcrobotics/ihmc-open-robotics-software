# Current version: 0.8
FROM ihmcrobotics/nvidia:0.5

ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /root/dev
USER root

# Install commonly required libraries and packages
# Note: Removing /var/lib/apt/lists/* at the end can reduce the
# compressed image size by about 25 MB.
RUN apt-get --quiet 2 --yes update  \
 && apt-get --quiet 2 --yes install \
    sudo \
    iproute2 \
    build-essential \
    libboost-all-dev \
    cmake \
    libtbb-dev \
    openjdk-17-jdk \
    libeigen3-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Needed to link with GTSAM
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> /root/.bashrc

# Ensure eigen is available at /usr/include/Eigen
# Ubuntu installs the headers at /usr/include/eigen3/Eigen
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#
# All package installations above this point.
#++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++#

# Install graphics and UI tools magnum and ImGui
USER robotlab
WORKDIR /home/robotlab/


# Current version: 0.1
FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

# Install commonly required libraries and packages
# Note: Removing /var/lib/apt/lists/* at the end can reduce the
# compressed image size by about 25 MB.
RUN apt-get --quiet 2 --yes update  \
 && apt-get --quiet 2 --yes install \
    nano \
    git \
    wget \
    curl \
    unzip \
    rsync \
    apt-transport-https \
    iputils-ping \
    ca-certificates \
    curl \
    software-properties-common \
    python3-opencv \
    sudo \
    iproute2 \
    build-essential \
    libboost-all-dev \
    cmake \
    openjdk-17-jdk \
    libeigen3-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Ensure eigen is available at /usr/include/Eigen
# Ubuntu installs the headers at /usr/include/eigen3/Eigen
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

WORKDIR /root/dev

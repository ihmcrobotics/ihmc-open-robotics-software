# Current version: 0.1
FROM ubuntu:22.04

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
    libtbb-dev \
    openjdk-17-jdk \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Clone and checkout gtsam to a specific commit; using the develop branch of gtsam
# Should periodically update the commit as needed
WORKDIR /root/dev
RUN git clone --depth 1 --branch develop https://github.com/borglab/gtsam.git gtsam
WORKDIR /root/dev/gtsam
RUN git checkout 0909c46339e0c604b0f26c8a67c7144c1358db4c

WORKDIR /root/dev/gtsam/build
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    ..

RUN make -j4 install && make clean

# Needed to link with GTSAM
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:LD_LIBRARY_PATH' >> /root/.bashrc

WORKDIR /root/dev

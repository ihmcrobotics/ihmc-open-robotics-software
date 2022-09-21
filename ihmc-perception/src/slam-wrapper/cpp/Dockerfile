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
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Setup a robotlab user as the development user, to avoid using root.
RUN addgroup robotlab \
 && adduser --home /home/robotlab --gecos "Rosie Robot,1117,1234567,2345678" --ingroup robotlab --disabled-password robotlab \
 && chown -R robotlab /home/robotlab \
 && adduser robotlab sudo \
 && echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
USER robotlab
WORKDIR /home/robotlab

# Make sure the .config folder exists and is owned by the robotlab user.
# This is useful for later installed apps.
RUN mkdir -p /home/robotlab/.config
RUN chown -R robotlab:robotlab /home/robotlab/.config

RUN mkdir dev
WORKDIR /home/robotlab/dev
RUN git clone --depth 1 --branch 4.1.1 https://github.com/borglab/gtsam.git
WORKDIR /home/robotlab/dev/gtsam/build
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

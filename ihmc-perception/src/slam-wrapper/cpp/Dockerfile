# Current version: 0.2
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
    libeigen3-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Clone and checkout GTSAM to a specific commit; using the develop branch of GTSAM
# Should periodically update the commit as needed
WORKDIR /root/dev
RUN git clone --depth 1 --branch develop https://github.com/borglab/gtsam.git gtsam
WORKDIR /root/dev/gtsam
RUN git checkout 0909c46339e0c604b0f26c8a67c7144c1358db4c

WORKDIR /root/dev/gtsam/build

# Build GTSAM
RUN cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
    -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
    -DGTSAM_BUILD_TIMING_ALWAYS=OFF \
    -DGTSAM_BUILD_TESTS=OFF \
    -DGTSAM_WITH_EIGEN_MKL=OFF \
    ..
RUN make -j$(nproc) install

# Needed to link with GTSAM
RUN echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> /root/.bashrc

# Ensure eigen is available at /usr/include/Eigen
# Ubuntu installs the headers at /usr/include/eigen3/Eigen
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

WORKDIR /root/dev

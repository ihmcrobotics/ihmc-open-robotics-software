# Current version: 0.5
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
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Install Eigen 3.4.0
WORKDIR /root/eigen
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
RUN tar -xvf eigen-3.4.0.tar.gz
WORKDIR /root/eigen/eigen-3.4.0
RUN mkdir build
WORKDIR /root/eigen/eigen-3.4.0/build
RUN cmake ..
RUN make install
# Ensure eigen is available at /usr/include/Eigen
# By default, the eigen headers are installed at /usr/local/include/eigen3/Eigen
RUN ln -s /usr/local/include/eigen3/Eigen /usr/include/Eigen

# Clone and checkout GTSAM to a specific commit; should update periodically
WORKDIR /root/dev
RUN git clone https://github.com/borglab/gtsam.git gtsam
WORKDIR /root/dev/gtsam
RUN git checkout 213543d9170783a5b3af21e3d394cd6988ff75fe

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

RUN useradd robotlab
# Switch to our robotlab user last because prior tasks need to run as root
USER robotlab

WORKDIR /home/robotlab
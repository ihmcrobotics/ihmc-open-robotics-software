# Current version: 0.1
FROM ihmcrobotics/nvidia:0.4

USER root

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# If interactive, apt-get will prompt for keyboard configuration
RUN apt-get --quiet 2 --yes update \
 && DEBIAN_FRONTEND=noninteractive \
    apt-get --quiet 2 --yes install \
    keyboard-configuration \
    build-essential \
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

# CLion (https://www.jetbrains.com/clion/download/#section=linux)
RUN mkdir -p Downloads \
 && cd Downloads \
 && curl -sL https://download.jetbrains.com/cpp/CLion-2022.2.3.tar.gz -o clion.tar.gz \
 && tar -xzf clion.tar.gz \
 && mv clion-2022.2.3/ /opt/clion \
 && ln -s /opt/clion/bin/clion.sh /usr/local/bin/clion \
 && cd .. \
 && rm -rf Downloads

USER robotlab
WORKDIR /home/robotlab

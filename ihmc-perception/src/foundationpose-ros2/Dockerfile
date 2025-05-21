FROM nvcr.io/nvidia/cuda:12.1.0-devel-ubuntu22.04
LABEL Name=foundationpose-ros2 Version="0.0.2"

ARG DEBIAN_FRONTEND=noninteractive

# 1. Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    lsb-release \
    curl \
    wget \
    gnupg2 \
    software-properties-common \
    git \
    sudo \
    python3-pip \
    v4l-utils \
    udev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    cmake \
    libgtk-3-dev \
    libusb-1.0-0-dev \
    libssl-dev \
    libudev-dev \
    libboost-all-dev \
    pkg-config \
    libx11-dev \
    libxext-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    x11proto-dev \
    && rm -rf /var/lib/apt/lists/*

# 2. Set up locale
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# 4. Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/*

# 5. Install IHMC ROS2 interfaces
WORKDIR /root/ihmc_ros2_ws
# TODO: Remove branch before merging to develop
RUN git clone -b feature/foundation-pose-integration https://github.com/ihmcrobotics/ihmc-open-robotics-software.git /opt/ihmc-open-robotics-software && \
    mkdir src && \
    cp -r /opt/ihmc-open-robotics-software/ihmc-interfaces/src/main/messages/ihmc_interfaces ./src/ihmc_interfaces && \
    rm -rf /opt/ihmc-open-robotics-software && \
    . /opt/ros/humble/setup.sh && colcon build

SHELL ["/bin/bash", "--login", "-c"]

# 6. Install Miniconda + FoundationPose Env
RUN cd / && wget --quiet https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O /miniconda.sh && \
    /bin/bash /miniconda.sh -b -p /opt/conda && \
    ln -s /opt/conda/etc/profile.d/conda.sh /etc/profile.d/conda.sh && \
    echo ". /opt/conda/etc/profile.d/conda.sh" >> ~/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    /opt/conda/bin/conda update -n base -c defaults conda -y && \
    /opt/conda/bin/conda config --set ssl_verify no && \
    /opt/conda/bin/conda config --add channels conda-forge && \
    /opt/conda/bin/conda create -n foundationpose-ros2 python=3.10 -y

ENV PATH=$PATH:/opt/conda/envs/foundationpose-ros2/bin

COPY requirements.txt /root/foundationpose-ros2/

WORKDIR /root/foundationpose-ros2

# Conda init and environment activation
RUN conda init bash && \
    echo "conda activate foundationpose-ros2" >> ~/.bashrc && \
    pip install torchvision==0.16.0+cu121 torchaudio==2.1.0 torch==2.1.0+cu121 --index-url https://download.pytorch.org/whl/cu121 && \
    pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable"
    
ENV LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6
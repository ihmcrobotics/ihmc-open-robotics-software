# An image for developing GPU accelerated applications on NVIDIA hardware.
# This image is published to https://hub.docker.com/r/ihmcrobotics/nvidia
# Current version: 0.5
# Refer to the NVIDIA/CUDA base image here: (https://hub.docker.com/r/nvidia/cuda/)
# This is currently based on Ubuntu 20.04 Focal Fossa.
FROM nvidia/cuda:11.6.2-cudnn8-devel-ubuntu20.04
# Max: swapping out base images
#FROM pytorch/pytorch:1.8.1-cuda11.1-cudnn8-runtime

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
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Install OpenGL libraries on nvidia-docker2
# Reference: (https://medium.com/@benjamin.botto/opengl-and-cuda-applications-in-docker-af0eece000f1)
RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    opencl-headers \
    ocl-icd-opencl-dev \
    libgflags-dev \
    libglvnd-dev \
    libgl1-mesa-dev \
    libegl1-mesa-dev \
    libgles2-mesa-dev \
    libxext6 \
    libx11-6 \
    freeglut3-dev \
    mesa-utils \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# Configure OpenCL (https://stackoverflow.com/questions/54243119/how-to-run-opencl-opengl-inside-a-docker-container)
RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install clinfo > /dev/null \
 && rm -rf /var/lib/apt/lists/* \
 && mkdir -p /etc/OpenCL/vendors \
 && echo "libnvidia-opencl.so.1" > /etc/OpenCL/vendors/nvidia.icd

#Configure NVIDIA runtime for Docker
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute

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

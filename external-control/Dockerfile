# Current version: 0.1
FROM ubuntu:22.04

ARG DEBIAN_FRONTEND=noninteractive

# Install commonly required libraries and packages
RUN apt-get --quiet 2 --yes update

# Common utilities
RUN apt-get --quiet 2 --yes install \
    sudo \
    apt-transport-https \
    ca-certificates \
    software-properties-common \
    nano \
    git \
    wget \
    curl \
    unzip \
    python3 \
    > /dev/null

# Java
RUN apt-get --quiet 2 --yes install \
    openjdk-17-jdk \
    > /dev/null

# C++ development utilities -- we need to add Kitware's repositories in order to get a specific version of CMake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
RUN apt-get update
RUN apt-get install kitware-archive-keyring
RUN rm /etc/apt/trusted.gpg.d/kitware.gpg
RUN apt-get --quiet 2 --yes install \
    build-essential \
    gdb \
    ninja-build \
    clang \
    mold \
    cmake \
    valgrind \
    patchelf \
    catch2 \
    > /dev/null

# Install Eigen
RUN apt-get --quiet 2 --yes install libeigen3-dev
# Ensure eigen is available at /usr/include/Eigen, Ubuntu installs the headers at /usr/include/eigen3/Eigen
RUN ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
# Ditto for unsupported
RUN ln -s /usr/include/eigen3/unsupported /usr/include/unsupported
# Install Python 3 which is required for a gdb extension that allows pretty printing of Eigen types in debugger.
# Because of docker integration with CLion, we must place the .gdbinit file in the root directory -- all containers will
# run with / as the $HOME directory, which is where .gdbinit should be placed
WORKDIR /
RUN touch .gdbinit
RUN printf "python\n\
import sys\n\
sys.path.insert(0, '/home/robotlab/gdb-eigen')\n\
from printers import register_eigen_printers\n\
register_eigen_printers(None)\n\
end" > .gdbinit
WORKDIR /home/robotlab/gdb-eigen
RUN wget -O printers.py https://gitlab.com/libeigen/eigen/-/raw/master/debug/gdb/printers.py
RUN touch __init__.py

# Update alternatives so that clang is the default compiler
RUN update-alternatives --install /usr/bin/cc cc /usr/bin/clang 100
RUN update-alternatives --install /usr/bin/c++ c++ /usr/bin/clang++ 100
# Symlink mold to ld so it is the default linker, recommended by: https://lld.llvm.org/ (Using LLD)
RUN ln -sf /usr/bin/mold /usr/bin/ld
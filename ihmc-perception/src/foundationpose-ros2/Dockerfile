##
# Please do not publish this giant image to Dockerhub
##

FROM nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update
RUN apt-get install -y curl locales git python3-pip build-essential cmake libboost-all-dev wget python-is-python3 libgl1

# Set up locale
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update
RUN apt-get install -y ros-humble-desktop ros-humble-cv-bridge python3-colcon-common-extensions
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# Install FoundationPose
RUN git clone https://github.com/ArghyaChatterjee/FoundationPose.git /root/FoundationPose
WORKDIR /root/FoundationPose
RUN pip install -r requirements.txt
RUN git clone https://github.com/pybind/pybind11 && \
	cd pybind11 && \
	git checkout v2.10.0 && \
	mkdir build && \
	cd build && \
	cmake .. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release -DPYBIND11_INSTALL=ON -DPYBIND11_TEST=OFF && \
	make -j$(nproc) && \
	make install
RUN wget https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz && \
	tar xvzf eigen-3.4.0.tar.gz && rm eigen-3.4.0.tar.gz && \
	cd eigen-3.4.0 && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make -j$(nproc) && \
	make install
RUN git clone https://github.com/NVlabs/nvdiffrast && \
	cd nvdiffrast && \
	pip install .
RUN cd mycpp && \
	rm -rf build && \
	mkdir build && \
	cd build && \
	cmake .. && \
	make -j$(nproc)
RUN cd bundlesdf/mycuda && \
	rm -rf build *egg* *.so && \
	TORCH_CUDA_ARCH_LIST="8.0;8.6;8.9+PTX" python3 -m pip install -e .
RUN git clone --recursive https://github.com/NVIDIAGameWorks/kaolin /kaolin && \
	pip install Cython && \
	cd /kaolin && \
	FORCE_CUDA=1 python setup.py develop
RUN pip install "git+https://github.com/facebookresearch/pytorch3d.git@stable"

WORKDIR /root/foundationpose-ros2

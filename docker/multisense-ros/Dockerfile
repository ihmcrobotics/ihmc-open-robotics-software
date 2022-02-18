# Current version: 0.2
FROM ihmcrobotics/nvidia-ros:0.2

USER root

RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    python3-rosinstall \
    build-essential \
    cmake \
    ros-noetic-image-geometry \
    libturbojpeg0-dev \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

COPY ./multisense_entrypoint.sh /home/robotlab/multisense_entrypoint.sh
RUN chmod a+x /home/robotlab/multisense_entrypoint.sh

USER robotlab
WORKDIR /home/robotlab

RUN mkdir -p dev/multisense_ws
VOLUME /home/robotlab/dev/multisense_ws
WORKDIR /home/robotlab/dev/multisense_ws

ENTRYPOINT ["/home/robotlab/multisense_entrypoint.sh"]
CMD ["bash"]

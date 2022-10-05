# Current version: 0.1
FROM ihmcrobotics/nvidia-ros2:0.1

USER root

RUN apt-get --quiet 2 --yes update \
 && DEBIAN_FRONTEND=noninteractive \
    apt-get --quiet 2 --yes install \
    build-essential \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# IntelliJ Installation
RUN mkdir -p Downloads \
 && cd Downloads \
 && curl -sL https://download.jetbrains.com/idea/ideaIC-2022.2.3.tar.gz -o intellij.tar.gz \
 && tar -xzf intellij.tar.gz \
 && mv intellij-2021.1.2/ /opt/intellij \
 && ln -s /opt/intellij/bin/intellij.sh /usr/local/bin/intellij \
 && cd .. \
 && rm -rf Downloads

USER robotlab
WORKDIR /home/robotlab

ENTRYPOINT ["/home/robotlab/ros_entrypoint.sh"]
CMD ["bash"]
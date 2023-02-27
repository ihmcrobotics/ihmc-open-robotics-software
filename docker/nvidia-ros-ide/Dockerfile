# Current version: 0.3
FROM ihmcrobotics/nvidia-ros:0.4

USER root

RUN apt-get --quiet 2 --yes update \
 && DEBIAN_FRONTEND=noninteractive \
    apt-get --quiet 2 --yes install \
    build-essential \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# CLion (https://www.jetbrains.com/clion/download/#section=linux)
RUN mkdir -p Downloads \
 && cd Downloads \
 && curl -sL https://download.jetbrains.com/cpp/CLion-2021.1.tar.gz -o clion.tar.gz \
 && tar -xzf clion.tar.gz \
 && mv clion-2021.1/ /opt/clion \
 && ln -s /opt/clion/bin/clion.sh /usr/local/bin/clion \
 && cd .. \
 && rm -rf Downloads

# PyCharm (https://www.jetbrains.com/pycharm/download/#section=linux)
RUN mkdir -p Downloads \
 && cd Downloads \
 && curl -sL https://download.jetbrains.com/python/pycharm-community-2021.1.tar.gz -o pycharm.tar.gz \
 && tar -xzf pycharm.tar.gz \
 && mv pycharm-community-2021.1/ /opt/pycharm \
 && ln -s /opt/pycharm/bin/pycharm.sh /usr/local/bin/pycharm \
 && cd .. \
 && rm -rf Downloads

USER robotlab
WORKDIR /home/robotlab

ENTRYPOINT ["/home/robotlab/ros_entrypoint.sh"]
CMD ["bash"]

FROM ihmcrobotics/nvidia:0.6

USER root

RUN apt-get --quiet 2 --yes update \
 && apt-get --quiet 2 --yes install \
    cmake \
    g++ \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*

# PyCharm (https://www.jetbrains.com/pycharm/download/#section=linux)
RUN mkdir -p Downloads \
 && cd Downloads \
 && curl -sL https://download.jetbrains.com/python/pycharm-2025.1.1.tar.gz -o pycharm.tar.gz \
 && tar -xzf pycharm.tar.gz \
 && mv pycharm-2025.1.1/ /opt/pycharm \
 && ln -s /opt/pycharm/bin/pycharm.sh /usr/local/bin/pycharm \
 && cd .. \
 && rm -rf Downloads

USER robotlab

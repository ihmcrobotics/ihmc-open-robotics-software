# Current version: 0.1
FROM ihmcrobotics/nvidia-ros2:0.2

USER root

RUN apt-get --yes update \
 && DEBIAN_FRONTEND=noninteractive \
    apt-get --yes install \
    openjdk-17-jdk \
    openjdk-17-doc \
    openjdk-17-source \
    build-essential \
    > /dev/null \
 && rm -rf /var/lib/apt/lists/*


RUN mkdir -p Downloads \
  && cd Downloads \
  && curl -sL https://services.gradle.org/distributions/gradle-7.5.1-all.zip -o gradle.zip \
  && unzip -q gradle.zip -d gradle \
  && mv gradle/gradle-7.5.1/ /opt/. \
  && cd .. \
  && rm -rf Downloads
#
# IntelliJ Installation
RUN mkdir -p Downloads \
 && cd Downloads \
 && curl -sL https://download.jetbrains.com/idea/ideaIC-2022.2.3.tar.gz -o intellij.tar.gz \
 && tar -xzf intellij.tar.gz \
 && mv idea-* /opt/intellij \
 && ln -s /opt/intellij/bin/intellij.sh /usr/local/bin/intellij \
 && cd .. \
 && rm -rf Downloads

USER robotlab
WORKDIR /home/robotlab

ENTRYPOINT ["/home/robotlab/ros_entrypoint.sh"]
CMD ["bash"]
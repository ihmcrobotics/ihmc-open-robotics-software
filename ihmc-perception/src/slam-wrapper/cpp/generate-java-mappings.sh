#!/bin/bash
set -e -o xtrace

rm -rf build
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=. ..
make -j 4
make install

# Use the latest release on GitHub
# https://github.com/bytedeco/javacpp/releases
JAVACPP_VERSION=1.5.7

curl -L https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip -o javacpp-platform-$JAVACPP_VERSION-bin.zip
unzip -j javacpp-platform-$JAVACPP_VERSION-bin.zip

# Copy the javacpp InfoMapper into build
mkdir -p us/ihmc/perception/slamWrapper
cp ../../java/us/ihmc/perception/slamWrapper/ihmc_slam_wrapper.java us/ihmc/perception/slamWrapper/ihmc_slam_wrapper.java

java -jar javacpp.jar us/ihmc/perception/slamWrapper/ihmc_slam_wrapper.java -d ../../generated-java/us/ihmc/perception/slamWrapper
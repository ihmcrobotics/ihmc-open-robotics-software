#!/bin/bash
set -e -o xtrace

# Clean and create build directory
rm -rf build && mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=. ..
make -j$(nproc)
make install

# Use the latest release on GitHub
# https://github.com/bytedeco/javacpp/releases
JAVACPP_VERSION=1.5.8

# Copy all Java code from the root of slam-wrapper into the build directory
cp -r ../java/ .

# Move into the java directory; javacpp.jar needs to reside here
cd java

# Download and unzip javacpp into the java source directory
curl -L https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip -o javacpp-platform-$JAVACPP_VERSION-bin.zip
unzip -j javacpp-platform-$JAVACPP_VERSION-bin.zip

java -jar javacpp.jar us/ihmc/bytedeco/slamWrapper/presets/SlamWrapperInfoMapper.java
# This will generate the jni shared library and place it into the classpath resources dir
java -jar javacpp.jar us/ihmc/bytedeco/slamWrapper/SlamWrapper.java -d ../../resources

# Clean old generated Java code
rm -rf ../../generated-java/*

# Copy newly generated Java into generated-java
mkdir -p ../../generated-java/us/ihmc/bytedeco/slamWrapper
cp -r us/ihmc/bytedeco/slamWrapper/SlamWrapper.java ../../generated-java/us/ihmc/bytedeco/slamWrapper

cp ../lib/libslam-wrapper.so ../../resources

# Temp hack
# This really assumes we are building within the docker container OR we have GTSAM installed at this exact location
# We need to copy these GTSAM related library files into our classpath resources dir
cp /usr/local/lib/libgtsam.so ../../resources
cp /usr/local/lib/libmetis-gtsam.so ../../resources
cp /usr/lib/x86_64-linux-gnu/libtbb.so ../../resources
cp /usr/lib/x86_64-linux-gnu/libboost_filesystem.so ../../resources
cp /usr/lib/x86_64-linux-gnu/libboost_serialization.so ../../resources
cp /usr/lib/x86_64-linux-gnu/libboost_timer.so ../../resources
cp /usr/lib/x86_64-linux-gnu/libboost_chrono.so ../../resources

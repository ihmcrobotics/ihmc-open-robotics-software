#!/bin/bash
set -e -o xtrace

# Clean and create build directory
cd cpp
rm -rf build && mkdir build
cd build

cmake -DCMAKE_INSTALL_PREFIX=. ..
make -j$(nproc)
make install

# Use the latest release on GitHub
# https://github.com/bytedeco/javacpp/releases
JAVACPP_VERSION=1.5.7

# Copy all Java code from the root of slam-wrapper into the build directory
cp -r ../../java/ .

# Move into the java directory; javacpp.jar needs to reside here
cd java

# Download and unzip javacpp into the java source directory
curl -L https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip -o javacpp-platform-$JAVACPP_VERSION-bin.zip
unzip -j javacpp-platform-$JAVACPP_VERSION-bin.zip

java -jar javacpp.jar us/ihmc/bytedeco/mapsenseWrapper/presets/MapsenseWrapperInfoMapper.java
# This will generate the jni shared library and place it into the classpath resources dir

java -jar javacpp.jar us/ihmc/bytedeco/mapsenseWrapper/MapsenseWrapper.java -d ../../../resources

# Clean old generated Java code
rm -rf ../../../generated-java/*

# Copy newly generated Java into generated-java
mkdir -p ../../../generated-java/us/ihmc/bytedeco/mapsenseWrapper
cp -r us/ihmc/bytedeco/mapsenseWrapper/MapsenseWrapper.java ../../../generated-java/us/ihmc/bytedeco/mapsenseWrapper

# Run ldd on libmapsense-wrapper.so to find required .so file names.
cp ../lib/libmapsense-wrapper.so ../../../resources
cp /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libwebp.so.6 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libIlmImf-2_3.so.24 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libHalf.so.24 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libIex-2_3.so.24 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libIlmThread-2_3.so.24 ../../../resources
cp /usr/lib/x86_64-linux-gnu/libIlmThread-2_3.so.24 ../../../resources
cp /lib/libgdal.so.26 ../../../resources
cp /lib/libarmadillo.so.9 ../../../resources
cp /lib/x86_64-linux-gnu/libpoppler.so.97 ../../../resources
cp /lib/x86_64-linux-gnu/libjson-c.so.4 ../../../resources
cp /lib/x86_64-linux-gnu/libqhull.so.7 ../../../resources
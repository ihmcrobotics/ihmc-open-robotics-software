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
JAVACPP_VERSION=1.5.8

# Copy all Java code from the root of mapsense-wrapper into the build directory
cp -r ../../java/ .

# Move into the java directory; javacpp.jar needs to reside here
cd java

# Download and unzip javacpp into the java source directory
curl -L https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip -o javacpp-platform-$JAVACPP_VERSION-bin.zip
unzip -j javacpp-platform-$JAVACPP_VERSION-bin.zip

java -jar javacpp.jar us/ihmc/mpc/mpcWrapper/presets/MPCWrapperInfoMapper.java
# This will generate the jni shared library and place it into the classpath resources dir

java -jar javacpp.jar us/ihmc/mpc/mpcWrapper/MPCWrapper.java -d ../../../resources/mpcWrapper/linux-x86_64

# Clean old generated Java code
rm -rf ../../../generated-java/us/ihmc/mpc/mpcWrapper

# Copy newly generated Java into generated-java
mkdir -p ../../../generated-java/us/ihmc/mpc/mpcWrapper
cp -r us/ihmc/mpc/mpcWrapper/MPCWrapper.java ../../../generated-java/us/ihmc/mpc/mpcWrapper

# Run ldd on <libwrapper-name>.so to find required .so file names.
cp ../libmpc-wrapper.so ../../../resources/mpcWrapper/linux-x86_64

cp /opt/hpipm/lib/libhpipm.so ../../../resources/mpcWrapper/linux-x86_64
cp /opt/blasfeo/lib/libblasfeo.so ../../../resources/mpcWrapper/linux-x86_64
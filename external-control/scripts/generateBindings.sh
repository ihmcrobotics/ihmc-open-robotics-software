#!/bin/bash
set -e -o xtrace

cd /home/robotlab/ihmc-external-wrapper/build

# Copy all Java code from the root of crocoddyl-wrapper into the build directory
cp -r ../external-control/src/main/java/ .

# Move into the java directory; javacpp.jar needs to reside here
cd java

# Clone and checkout JavaCPP from specific tag; should update periodically
JAVACPP_VERSION=1.5.10
# Download and unzip javacpp into the java source directory
# Check if the javacpp.jar already exists -- we can skip the fetching step if it does
if [ ! -f javacpp.jar ]; then
  curl -L https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip -o javacpp-platform-$JAVACPP_VERSION-bin.zip
  unzip -j javacpp-platform-$JAVACPP_VERSION-bin.zip
fi

# This will generate the JNI shared library and place it in the classpath resources dir
java -jar javacpp.jar us/ihmc/externalControl/presets/ExternalControlInfoMapper.java
java -jar javacpp.jar us/ihmc/externalControl/ExternalControl.java -d ../../external-control/src/main/resources/externalControl/linux-x86_64 \
-Dplatform.compiler="clang++" \
-Dplatform.compiler.default="-O3"

# Clean old generated code
rm -rf ../../external-control/src/main/generated-java/*

# Copy newly generated Java into generated-java
mkdir -p ../../external-control/src/main/generated-java/us/ihmc/externalControl
cp -r us/ihmc/externalControl/ExternalControl.java ../../external-control/src/main/generated-java/us/ihmc/externalControl
chmod 664 ../../external-control/src/main/generated-java/us/ihmc/externalControl/ExternalControl.java # Add write permissions to the generated file, so it can be modified

# Copy over the wrapper and its dependencies -- this will be pinocchio and crocoddyl, as well as their
# dependencies recursively
LINUX_RESOURCES_DIR='../../external-control/src/main/resources/externalControl/linux-x86_64'
cp ../lib/libexternal-control.so $LINUX_RESOURCES_DIR
# All other dependencies are system libraries, so we look to /usr/lib/x86_64-linux-gnu and loop over them
DEP_NAMES=('libboost_filesystem.so.1.74.0' \
           'libboost_system.so.1.74.0' \
           'libboost_serialization.so.1.74.0' \
           'liburdfdom_sensor.so.3.0' \
           'liburdfdom_model_state.so.3.0' \
           'liburdfdom_model.so.3.0' \
           'liburdfdom_world.so.3.0' \
           'libconsole_bridge.so.1.0' \
           'libomp.so.5')
for LIB_NAME in "${DEP_NAMES[@]}"; do
  cp /usr/lib/x86_64-linux-gnu/$LIB_NAME $LINUX_RESOURCES_DIR
done
# We also modify all the libraries that end up in the resources folder also have a relative runtime path to their own directory
RPATH='/usr/lib/x86_64-linux-gnu:$ORIGIN'
for LIB_NAME in $LINUX_RESOURCES_DIR/*; do
  patchelf --set-rpath $RPATH $LIB_NAME
done
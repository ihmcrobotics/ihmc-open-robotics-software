#!/bin/bash
rm -rf cppbuild
mkdir cppbuild
cd cppbuild

#### Installing Eigen headers ####
EIGEN_VERSION=3.4.0
curl https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-$EIGEN_VERSION.zip -O
unzip -n eigen-$EIGEN_VERSION.zip
cd eigen-$EIGEN_VERSION
mkdir build
cd build
cmake -DCMAKE_INSTALL_PREFIX=../../ ..
cmake --install .
cd ../../

# Build external-control
cmake -DCMAKE_INSTALL_PREFIX=. ..
cmake --build . --config Release --target install

### Java generation ####
cp -r ../src/main/java/* .

JAVACPP_VERSION=1.5.10
if [ ! -f javacpp.jar ]; then
  curl -L https://github.com/bytedeco/javacpp/releases/download/$JAVACPP_VERSION/javacpp-platform-$JAVACPP_VERSION-bin.zip -o javacpp-platform-$JAVACPP_VERSION-bin.zip
  unzip -j javacpp-platform-$JAVACPP_VERSION-bin.zip
fi

java -jar javacpp.jar us/ihmc/externalControl/ExternalControlInfoMapper.java
cp us/ihmc/externalControl/*.java ../src/main/java/us/ihmc/externalControl
cp us/ihmc/externalControl/global/*.java ../src/main/java/us/ihmc/externalControl/global

java -jar javacpp.jar us/ihmc/externalControl/global/*.java -d javainstall

##### Copy shared libs to resources ####
# Linux
mkdir -p ../src/main/resources/externalControl/native/linux-x86_64
if [ -f "javainstall/libjniExternalControlWrapper.so" ]; then
  cp javainstall/libjniExternalControlWrapper.so ../src/main/resources/externalControl/native/linux-x86_64
fi
if [ -f "lib/libexternal-control.so" ]; then
  cp lib/libexternal-control.so ../src/main/resources/externalControl/native/linux-x86_64
fi

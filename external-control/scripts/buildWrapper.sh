#!/bin/bash
set -e -o xtrace

cd /home/robotlab/ihmc-external-wrapper

# Create build directory if it doesn't already exist
mkdir -p build/lib && cd build

# Default to not building tests, unless --test flag is passed in
TEST="OFF"
if [ "$1" == "--test" ]; then
  TEST="ON"
fi

# Build and install the wrapper
cmake \
-G Ninja \
-DCMAKE_INSTALL_PREFIX=. \
-DCMAKE_BUILD_TYPE=Release \
-DBUILD_EXAMPLES=OFF \
-DBUILD_TESTING="$TEST" \
-DCMAKE_CXX_FLAGS="-O3" \
..
cmake --build . --target install
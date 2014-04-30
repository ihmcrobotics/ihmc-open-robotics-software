#!/bin/bash

rm -rf build/*
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make -j4 VERBOSE=1 install
cd ..

rm -rf build/*
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=MinSizeRel ../
make -j4 VERBOSE=1 install
cd ..

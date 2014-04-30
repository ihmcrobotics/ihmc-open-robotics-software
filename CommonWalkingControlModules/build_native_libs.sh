#!/bin/bash

rm -rf build/*
cd build
cmake -DCMAKE_BUILD_TYPE=Release ../
make -j4 VERBOSE=1 install

rm -rf build/*
cd build
cmake -DCMAKE_BUILD_TYPE=RelMinSize ../
make -j4 VERBOSE=1 install

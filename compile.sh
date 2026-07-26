#!/bin/bash
set -e

# Release build -> build/
rm -rf build
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
cd ..

# Debug build -> build-debug/
rm -rf build-debug
mkdir build-debug
cd build-debug
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -j
cd ..

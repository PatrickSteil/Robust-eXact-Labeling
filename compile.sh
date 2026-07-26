#!/bin/bash
set -euo pipefail

JOBS="${JOBS:-$(nproc)}"

# Debug build -> build-debug/
rm -rf build-debug
cmake -S . -B build-debug \
  -DCMAKE_BUILD_TYPE=Debug

cmake --build build-debug -j"$JOBS"


# Release build -> build/
rm -rf build
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release

cmake --build build -j"$JOBS"


# Performance/profiling build -> build-perf/
rm -rf build-perf
cmake -S . -B build-perf \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo

cmake --build build-perf -j"$JOBS"

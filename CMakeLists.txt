cmake_minimum_required(VERSION 3.10)
project(RXLProject LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Threads REQUIRED)
add_library(rxl
  src/graph.cpp src/dijkstra.cpp src/pruned_labeling.cpp src/query_support.cpp
  src/index_io.cpp src/statistics.cpp)
target_include_directories(rxl PUBLIC include)
# Vendored google::sparse_hash_map (see third_party/sparsehash/README.md),
# used by include/sample_tree_storage.h to back small sampled trees during
# SamPG ordering (Appendix A.2). PRIVATE: it's an implementation detail of
# rxl's .cpp files, not part of the public API in include/pruned_labeling.h
# and friends, so it shouldn't leak to consumers that just link against rxl.
target_include_directories(rxl PRIVATE third_party/sparsehash)
target_link_libraries(rxl PUBLIC Threads::Threads)
target_compile_options(rxl PRIVATE $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall;-Wextra;-Wpedantic>)
add_executable(rxl_app src/main.cpp)
target_link_libraries(rxl_app PRIVATE rxl)
include(CTest)
if(BUILD_TESTING)
  add_executable(rxl_tests tests/test_main.cpp)
  target_link_libraries(rxl_tests PRIVATE rxl)
  add_test(NAME rxl_unit_tests COMMAND rxl_tests)
endif()

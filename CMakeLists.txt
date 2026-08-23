cmake_minimum_required(VERSION 3.10)

project(RXLProject LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)

if(ipo_supported)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELWITHDEBINFO TRUE)
else()
    message(WARNING "IPO/LTO not supported: ${ipo_error}")
endif()

find_package(Threads REQUIRED)

add_library(rxl
  src/csr_graph.cpp
  src/graph.cpp
  src/dijkstra.cpp
  src/addressable_heap.cpp
  src/pruned_dijkstra.cpp
  src/sampg.cpp
  src/pruned_labeling.cpp
  src/query_support.cpp
  src/index_io.cpp
  src/statistics.cpp
)

target_include_directories(rxl PUBLIC include)
target_include_directories(rxl PRIVATE third_party/sparsehash)

target_link_libraries(rxl PUBLIC Threads::Threads)

target_compile_options(rxl PRIVATE
  $<$<CXX_COMPILER_ID:GNU,Clang>:
    -Wall
    -Wextra
    -Wpedantic
  >
)

add_executable(RXL src/main.cpp)
target_link_libraries(RXL PRIVATE rxl)
target_include_directories(RXL PRIVATE third_party/cmdparser)

add_executable(benchmark src/bench_grid.cpp)
target_link_libraries(benchmark PRIVATE rxl)
target_include_directories(benchmark PRIVATE third_party/cmdparser)

include(CTest)

if(BUILD_TESTING)
  add_executable(tests 
    tests/test_main.cpp
    tests/test_delta_label.cpp
    tests/test_directed_weighted.cpp
    tests/test_rank_reorder.cpp
    tests/test_hub_ids_are_ranks.cpp
    tests/test_export_roundtrip.cpp
    tests/test_default_encoding.cpp
    tests/test_varint_encoding.cpp
    tests/test_parallel_determinism.cpp
    tests/test_sparse_tree_storage.cpp
    tests/test_statistics.cpp
    tests/test_zero_one_bfs.cpp
  )
  target_link_libraries(tests PRIVATE rxl)
  target_include_directories(tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME rxl_unit_tests COMMAND tests)
endif()

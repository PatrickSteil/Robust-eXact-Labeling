# RXL: sampled pruned hub labeling

C++17 exact distance index for directed, positively weighted graphs. It combines
weighted Pruned Labeling with a SamPG-style sampled path-greedy vertex order.

## Build and test

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build --output-on-failure
```

## CLI

```sh
./build/rxl_app graph.gr --verbose --threads 8 --export graph.rxl
./build/rxl_app graph.gr --benchmark          # build, then 10000 random queries
./build/rxl_app --import graph.rxl --benchmark 50000
```

Options:

- `--export FILE`: writes a portable little-endian binary index containing both
  label directions and the rank maps. `IndexIO::import_binary()` reads it back.
- `--import FILE`: loads labels and rank maps from a previously exported RXL
  index instead of building them. When `--import` is given, the graph
  argument is optional (it is only needed if `--verbose` should also print
  graph statistics).
- `--benchmark [N]`: runs `N` (default 10000) random vertex-to-vertex queries
  drawn uniformly from the index's vertex range and reports the average
  per-query runtime (microseconds) and how many of the pairs were reachable
  (i.e. how many shortest paths were found).
- `--verbose`: graph degree/size statistics, label entry/size statistics,
  sampling work, label work, sampled-tree counts, progress, and elapsed time.
- `--threads N`: number of sample-tree workers. Trees in a growth batch use the
  same immutable partial-label snapshot and are grown concurrently; score/tree
  integration remains serial and race-free. PL hub iterations remain sequential
  because each one depends on all earlier labels.
- `--degree`: disable sampling and use descending total degree.
- `--no-reorder`: do not physically rename graph/label vertices by importance.

## Structure

- `types.h`: `uint32_t` vertex IDs, distances, ranks and shared containers.
- `graph.*`: zero-based graph and rank renumbering.
- `pruned_labeling.*`: SamPG ordering, parallel sample growth, exact PL.
- `index_io.*`: versioned binary export/import.
- `statistics.*`: graph and label statistics.
- `tests/test_main.cpp`: exactness, rank renumbering, export round-trip,
  parallel sampling, and statistics tests.

`UINT32_MAX` is reserved for infinity/invalid IDs. Edge weights must be positive
and every finite shortest-path distance must be below it.

The implementation follows the paper's SamPG design (sampled shortest-path
trees, robust counter buckets, covered-subtree deletion, pruned replenishment,
and work balancing), while favoring clear dense tree storage. The paper's more
complex hash-table/reverse-index memory engineering is not included.

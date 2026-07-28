# RXL: sampled pruned hub labeling

C++17 exact distance index for directed, positively weighted graphs. It
builds a weighted Pruned Labeling index (Akiba, Iwata, and Yoshida, "Fast
Exact Shortest-Path Distance Queries on Large Networks by Pruned Landmark
Labeling", SIGMOD 2013) using a SamPG-style sampled vertex order, following
Delling, Goldberg, Pajor, and Werneck, "Robust Distance Queries on Massive
Networks", ESA 2014.

That paper's algorithm picks hub vertices by growing a rotating forest of
sampled shortest-path trees from random roots, scoring vertices by how much
labeling work removing them would save, and repeatedly promoting the
best-scoring vertex to the next hub. This implementation follows that
design (sampled trees, robust counter buckets, covered-subtree deletion,
pruned tree replenishment, and work balancing between sampling and
labeling), including the paper's memory engineering for small sampled trees
and the reverse-index acceleration from the paper's appendix.

A query looks up the intersection of two vertices' labels; this project
builds those labels once so distance queries afterward are fast.

## Layout

- `include/`, `src/` -- the library (`librxl`) plus the `RXL` CLI
- `tests/` -- unit tests
- `data/sample_graphs/` -- small graphs in each supported format, for
  trying things out
- `third_party/` -- vendored `cmdparser` and `sparsehash`

Graphs can be loaded in DIMACS, SNAP, METIS, or plain CSV edge-list format.
Internally the graph is stored as CSR (a forward star: one flat edge array
plus per-vertex offsets), and the graph-format loaders build into a
growable adjacency list first, then compact it into that CSR array once
parsing is done.

## Building

Requires CMake 3.10+ and a C++17 compiler.

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

This builds the `RXL` CLI and the `benchmark` binary. Add
`-DBUILD_TESTING=ON` to also build the unit tests:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build build -j
ctest --test-dir build
```

`compile.sh` does this for you, producing three side-by-side builds:
`build-debug` (assertions on, unoptimized), `build` (release), and
`build-perf` (release with debug symbols, for profiling):

```
./compile.sh
```

## Using the CLI

Build an index and run a benchmark of random queries:

```
build/RXL data/sample_graphs/tiny.graph -v -b
```

`-f` picks the input format (`dimacs`, `snap`, `metis`, or `csv`; default
`dimacs`). `-e path` exports the built index to a binary file; `-i path`
imports one instead of building from a graph. `-d` switches to plain
degree order instead of SamPG, mainly useful as a baseline. `-t N` uses N
threads for sample-tree construction. Run `build/RXL --help` for the rest.

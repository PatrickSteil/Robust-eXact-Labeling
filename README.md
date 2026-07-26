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
./build/rxl_app graph.gr --verbose --threads 8 --export graph.rxl --label-encoding varint
./build/rxl_app graph.gr --benchmark          # build, then 10000 random queries
./build/rxl_app --import graph.rxl --benchmark 50000
```

Options:

- `--export FILE`: writes a portable little-endian binary index containing both
  label directions and the rank maps. `IndexIO::import_binary()` reads it back
  and auto-detects which encoding it was written with -- readers never need
  to be told.
- `--label-encoding {two-block|varint}`: on-disk byte packing for hub-id
  deltas and distances, used by `--export` (default: `two-block`; no effect
  without `--export`). `two-block` is the paper's Section 4.1 scheme: a
  fixed 1-byte-per-entry prefix for deltas that fit in a byte, a fixed
  4-byte-per-entry suffix for the rest, distances at one fixed width for the
  whole file -- cheap, branch-light decoding. `varint` instead packs every
  delta and every distance individually as an unsigned LEB128 varint: 7
  payload bits per byte, with the top bit set to mean "one more byte
  follows" (so decoding a value is "keep reading bytes while the top bit is
  set"). No global width to choose and no size class per label, so it's
  often smaller -- but each entry costs a small variable-length decode loop
  instead of a fixed-offset read, so `two-block` stays the default.
- `--import FILE`: loads labels and rank maps from a previously exported RXL
  index instead of building them. When `--import` is given, the graph
  argument is optional (it is only needed if `--verbose` should also print
  graph statistics).
- `--benchmark [N]`: runs `N` (default 10000) random vertex-to-vertex queries
  drawn uniformly from the index's vertex range and reports the average
  per-query runtime (microseconds) and how many of the pairs were reachable
  (i.e. how many shortest paths were found).
- `--verbose`: graph degree/size statistics, label entry/size statistics,
  sampling work, label work, sampled-tree counts (including how many trees
  switched to the sparse hash-map backend), progress, and elapsed time.
- `--threads N`: number of sample-tree workers. Trees in a growth batch use the
  same immutable partial-label snapshot and are grown concurrently; score/tree
  integration remains serial and race-free. PL hub iterations remain sequential
  because each one depends on all earlier labels.
- `--degree`: disable sampling and use descending total degree.
- `--no-reorder`: do not physically rename graph/label vertices by importance.
- `--threads` also controls how many threads the `--benchmark` query loop
  uses, in addition to the sample-tree build.

## Performance notes

The ordering loop reuses scratch buffers across sample-tree builds and
per-vertex priority computations instead of reallocating them every call, and
tracks live sample-tree vertex counts incrementally instead of rescanning
O(n) arrays every rank. This changes constant factors only: the same sampled
trees, same selection order, and bit-identical exported indices come out for
a given `--threads 1` run; it's roughly 2-5x faster on graphs in the
hundreds-to-thousands-of-vertices range with no change in output. Label
sorting, rank reordering, and the `--benchmark` query loop are parallelized
across `--threads` since each of their iterations is independent and they
each only run once. The per-rank best-vertex-selection scan is *not*
thread-parallelized even though it's technically parallel-safe: it re-enters
every rank, so spawning threads there n times would likely cost more than it
saves without a persistent thread pool.

## Structure

- `types.h`: `uint32_t` vertex IDs, distances, ranks and shared containers.
- `graph.*`: zero-based graph and rank renumbering.
- `pruned_labeling.*`: SamPG ordering, parallel sample growth, exact PL.
- `sample_tree_storage.h`: per-tree storage backing SamPG's sampled trees;
  dense arrays while a tree is large, switching to a `google::sparse_hash_map`
  once it shrinks below n/8 live vertices (Appendix A.2).
- `index_io.*`: versioned binary export/import, in either of two on-disk
  label encodings (`LabelEncoding`): the default two-block fixed-width delta
  scheme (Section 4.1), or a per-value LEB128 varint scheme, selected at
  export time and self-described in the file header.
- `statistics.*`: graph and label statistics.
- `tests/test_main.cpp`: exactness, rank renumbering, export round-trip (both
  label encodings, plus the varint codec's byte-length boundaries),
  parallel sampling, sparse-tree-storage, and statistics tests.
- `third_party/sparsehash/`: vendored `google::sparse_hash_map` and its
  include closure (see that directory's own README for provenance).

`UINT32_MAX` is reserved for infinity/invalid IDs. Edge weights must be positive
and every finite shortest-path distance must be below it.

The implementation follows the paper's SamPG design (sampled shortest-path
trees, robust counter buckets, covered-subtree deletion, pruned replenishment,
and work balancing), including the paper's hash-table memory engineering for
small sampled trees (Appendix A.2). The paper's reverse-index acceleration
for the vertex-selection scan is not included.

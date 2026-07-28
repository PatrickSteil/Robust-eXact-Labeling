#include <cstdio>
#include <random>

#include "index_io.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "test_common.h"

void test_varint_encoding_roundtrip() {
  std::mt19937 rng(4242);
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  for (VertexId u = 0; u < 60; ++u)
    for (VertexId v = 0; v < 60; ++v)
      if (u != v && rng() % 6 == 0) arcs.emplace_back(u, v, 1 + rng() % 500);
  auto g = make_graph(60, arcs);
  SamplingOptions o;
  o.initial_trees = 16;
  o.counter_buckets = 4;
  o.discarded_max_buckets = 1;
  o.random_seed = 4242;
  auto result = PrunedLabeling::compute(g, o);
  const std::string path = "rxl_test_varint_roundtrip.bin";
  IndexIO::export_binary(result, path, LabelEncoding::Varint);
  auto loaded = IndexIO::import_binary(path);
  std::remove(path.c_str());
  CHECK(loaded.rank_to_vertex == result.rank_to_vertex);
  CHECK(loaded.vertex_to_rank == result.vertex_to_rank);
  check_exact(g, loaded.labels);
}

void test_varint_encoding_byte_boundaries() {
  const std::vector<Distance> boundary_values = {
      0,     1,       126,     127,       128,       16383,
      16384, 2097151, 2097152, 268435455, 268435456, kInfinity - 1};
  const VertexId n = static_cast<VertexId>(boundary_values.size());
  LabelingResult result;
  result.labels.resize(n);
  result.rank_to_vertex.resize(n);
  result.vertex_to_rank.resize(n);
  for (VertexId i = 0; i < n; ++i) result.rank_to_vertex[i] = i;
  for (VertexId i = 0; i < n; ++i) result.vertex_to_rank[i] = i;
  const std::vector<VertexId> deltas(n, 0);
  const std::vector<Distance> distances(boundary_values.begin(),
                                        boundary_values.end());
  result.labels[0].forward = Label::from_deltas(deltas, distances, n);
  result.labels[0].backward = Label::from_deltas(deltas, distances, n);

  const std::string path = "rxl_test_varint_boundaries.bin";
  IndexIO::export_binary(result, path, LabelEncoding::Varint);
  auto loaded = IndexIO::import_binary(path);
  std::remove(path.c_str());

  std::vector<Distance> forward, backward;
  for (const auto& [hub, d] : loaded.labels[0].forward) forward.push_back(d);
  for (const auto& [hub, d] : loaded.labels[0].backward) backward.push_back(d);
  CHECK(forward == distances);
  CHECK(backward == distances);
}

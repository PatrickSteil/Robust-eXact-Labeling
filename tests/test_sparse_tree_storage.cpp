#include <random>

#include "pruned_labeling.h"
#include "query_support.h"
#include "test_common.h"

void test_sparse_tree_storage() {
  std::mt19937 rng(2024);
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  const VertexId n = 120;
  for (VertexId u = 0; u < n; ++u)
    for (VertexId v = 0; v < n; ++v)
      if (u != v && rng() % 9 == 0) arcs.emplace_back(u, v, 1 + rng() % 50);
  auto g = make_graph(n, arcs);
  SamplingOptions o;
  o.initial_trees = 32;
  o.counter_buckets = 8;
  o.discarded_max_buckets = 1;
  o.random_seed = 2024;
  auto result = PrunedLabeling::compute(g, o);
  check_exact(g, result.labels);
  CHECK(result.statistics.sparse_downgrades > 0);
}

#include <random>

#include "pruned_labeling.h"
#include "query_support.h"
#include "test_common.h"

void test_parallel_determinism() {
  std::mt19937 rng(17);
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  for (VertexId u = 0; u < 25; ++u)
    for (VertexId v = 0; v < 25; ++v)
      if (u != v && rng() % 7 == 0) arcs.emplace_back(u, v, 1 + rng() % 20);
  auto g = make_graph(25, arcs);
  SamplingOptions a;
  a.initial_trees = 8;
  a.counter_buckets = 4;
  a.discarded_max_buckets = 1;
  a.random_seed = 99;
  a.num_threads = 1;
  auto b = a;
  b.num_threads = 4;
  auto one = PrunedLabeling::compute(g, a);
  auto four = PrunedLabeling::compute(g, b);
  check_exact(g, one.labels);
  check_exact(g, four.labels);
}

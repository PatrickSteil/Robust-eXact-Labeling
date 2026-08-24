#include <random>

#include "pruned_labeling.h"
#include "query_support.h"
#include "test_common.h"

namespace {

Graph make_random_graph(std::size_t n, double edge_prob, std::mt19937& rng) {
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  for (VertexId u = 0; u < n; ++u)
    for (VertexId v = 0; v < n; ++v)
      if (u != v &&
          std::uniform_real_distribution<double>(0.0, 1.0)(rng) < edge_prob)
        arcs.emplace_back(u, v, 1 + rng() % 20);
  return make_graph(n, arcs);
}

}  // namespace

// Batch size 1 must reproduce the un-batched algorithm exactly (same
// labels, not just the same query answers), since it takes the original
// per-vertex code path.
void test_batch_size_one_matches_sequential() {
  std::mt19937 rng(7);
  auto g = make_random_graph(40, 0.15, rng);
  SamplingOptions base;
  base.initial_trees = 8;
  base.counter_buckets = 4;
  base.discarded_max_buckets = 1;
  base.random_seed = 123;
  base.num_threads = 2;

  auto plain = PrunedLabeling::compute(g, base);

  SamplingOptions batched = base;
  batched.initial_batch_size = 1;
  batched.max_batch_size = 1;
  auto with_batching_disabled = PrunedLabeling::compute(g, batched);

  CHECK(plain.rank_to_vertex.size() == with_batching_disabled.rank_to_vertex.size());
  for (std::size_t i = 0; i < plain.rank_to_vertex.size(); ++i)
    CHECK(plain.rank_to_vertex[i] == with_batching_disabled.rank_to_vertex[i]);
  for (VertexId v = 0; v < g.num_vertices(); ++v) {
    CHECK(plain.labels[v].forward.size() ==
          with_batching_disabled.labels[v].forward.size());
    CHECK(plain.labels[v].backward.size() ==
          with_batching_disabled.labels[v].backward.size());
  }
}

// Distances must stay exact for every batch configuration, regardless of
// batch size, growth, diversity filtering, or an adaptive/tail split --
// batching only trades label-set optimality, never correctness.
void test_batched_labeling_is_exact() {
  std::mt19937 rng(2024);
  auto g = make_random_graph(60, 0.12, rng);

  struct Config {
    std::size_t initial_batch_size;
    std::size_t max_batch_size;
    double batch_growth_factor;
    bool diversity_filter;
    double sampling_fraction;
  };
  const std::vector<Config> configs = {
      {4, 4, 1.0, true, 1.0},     // fixed batch size, no tail
      {4, 4, 1.0, false, 1.0},    // fixed batch size, diversity filter off
      {2, 64, 2.0, true, 1.0},    // geometric growth, no tail
      {2, 64, 2.0, true, 0.34},   // geometric growth + heuristic tail (~1/3)
      {8, 8, 1.0, true, 0.5},     // fixed batch + tail
  };

  for (const Config& c : configs) {
    SamplingOptions options;
    options.initial_trees = 10;
    options.counter_buckets = 4;
    options.discarded_max_buckets = 1;
    options.random_seed = 42;
    options.num_threads = 4;
    options.initial_batch_size = c.initial_batch_size;
    options.max_batch_size = c.max_batch_size;
    options.batch_growth_factor = c.batch_growth_factor;
    options.batch_diversity_filter = c.diversity_filter;
    options.sampling_fraction = c.sampling_fraction;

    auto result = PrunedLabeling::compute(g, options);
    check_exact(g, result.labels);
    // Every vertex must still have been assigned exactly one rank.
    CHECK(result.rank_to_vertex.size() == g.num_vertices());
    std::vector<char> seen(g.num_vertices(), 0);
    for (VertexId v : result.rank_to_vertex) {
      CHECK(!seen[v]);
      seen[v] = 1;
    }
  }
}

// Batched runs with different thread counts must still agree with each
// other on query answers (determinism of *correctness*, not of the exact
// label set -- thread count can change which lane happens to settle a
// vertex first within a wave, and thus which of two equally-valid
// same-batch hubs ends up producing the entry).
void test_batched_labeling_thread_count_agrees() {
  std::mt19937 rng(99);
  auto g = make_random_graph(50, 0.15, rng);

  SamplingOptions a;
  a.initial_trees = 8;
  a.counter_buckets = 4;
  a.discarded_max_buckets = 1;
  a.random_seed = 5;
  a.initial_batch_size = 16;
  a.max_batch_size = 16;
  a.num_threads = 1;
  auto b = a;
  b.num_threads = 6;

  auto one = PrunedLabeling::compute(g, a);
  auto many = PrunedLabeling::compute(g, b);
  check_exact(g, one.labels);
  check_exact(g, many.labels);
}

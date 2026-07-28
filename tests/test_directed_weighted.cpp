#include "pruned_labeling.h"
#include "test_common.h"

void test_directed_weighted() {
  auto g = make_graph(
      6, {{0, 1, 2}, {0, 2, 9}, {1, 2, 3}, {2, 3, 4}, {1, 4, 20}, {3, 4, 1}});
  auto degree = PrunedLabeling::compute_with_degree_order(g);
  check_exact(g, degree.labels);
  SamplingOptions o;
  o.initial_trees = 4;
  o.counter_buckets = 2;
  o.discarded_max_buckets = 1;
  o.num_threads = 2;
  auto sampled = PrunedLabeling::compute(g, o);
  check_exact(g, sampled.labels);
}

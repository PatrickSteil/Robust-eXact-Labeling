#include "pruned_labeling.h"
#include "test_common.h"

void check_hub_ids_are_ranks(const LabelingResult& result) {
  for (VertexId v = 0; v < result.labels.size(); ++v) {
    const VertexId expected = result.vertex_to_rank[v];
    bool forward_ok = false, backward_ok = false;
    for (const auto& [hub, d] : result.labels[v].forward)
      if (d == 0 && hub == expected) forward_ok = true;
    for (const auto& [hub, d] : result.labels[v].backward)
      if (d == 0 && hub == expected) backward_ok = true;
    CHECK(forward_ok);
    CHECK(backward_ok);
  }
}

void test_hub_ids_are_ranks() {
  auto g = make_graph(
      6, {{0, 1, 2}, {0, 2, 9}, {1, 2, 3}, {2, 3, 4}, {1, 4, 20}, {3, 4, 1}});
  check_hub_ids_are_ranks(PrunedLabeling::compute_with_degree_order(g));
  SamplingOptions o;
  o.initial_trees = 4;
  o.counter_buckets = 2;
  o.discarded_max_buckets = 1;
  o.num_threads = 2;
  check_hub_ids_are_ranks(PrunedLabeling::compute(g, o));
}

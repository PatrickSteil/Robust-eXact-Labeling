#include "pruned_labeling.h"
#include "statistics.h"
#include "test_common.h"

void test_statistics() {
  auto g = make_graph(3, {{0, 1, 1}, {1, 2, 2}});
  auto gs = compute_graph_statistics(g);
  CHECK(gs.vertices == 3);
  CHECK(gs.arcs == 2);
  CHECK(gs.max_out_degree == 1);
  auto result = PrunedLabeling::compute_with_degree_order(g);
  auto ls = compute_label_statistics(result.labels);
  CHECK(ls.forward_entries > 0);
  CHECK(ls.backward_entries > 0);
}

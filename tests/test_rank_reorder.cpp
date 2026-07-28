#include "pruned_labeling.h"
#include "query_support.h"
#include "test_common.h"

void test_rank_reorder() {
  auto g = make_graph(
      5, {{0, 1, 1}, {1, 2, 2}, {0, 3, 8}, {2, 3, 1}, {3, 4, 3}, {4, 0, 7}});
  auto result = PrunedLabeling::compute_with_degree_order(g);
  std::vector<std::vector<Distance>> truth(g.num_vertices());
  for (VertexId s = 0; s < g.num_vertices(); ++s)
    truth[s] = Dijkstra::shortest_distances(g.adjacency(), s);
  auto map = g.reorder_by_rank(result.rank_to_vertex);
  PrunedLabeling::reorder_labels_by_rank(result);
  for (VertexId s = 0; s < g.num_vertices(); ++s)
    for (VertexId t = 0; t < g.num_vertices(); ++t)
      CHECK(QuerySupport::distance(result.labels, map[s], map[t]) ==
            truth[s][t]);
}

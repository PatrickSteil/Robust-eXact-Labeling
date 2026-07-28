#include <cstdio>

#include "index_io.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "test_common.h"

void test_export_roundtrip() {
  auto g = make_graph(4, {{0, 1, 5}, {1, 2, 6}, {2, 0, 2}});
  auto result = PrunedLabeling::compute_with_degree_order(g);
  const std::string path = "rxl_test_index.bin";
  IndexIO::export_binary(result, path);
  auto loaded = IndexIO::import_binary(path);
  std::remove(path.c_str());
  CHECK(loaded.rank_to_vertex == result.rank_to_vertex);
  CHECK(loaded.vertex_to_rank == result.vertex_to_rank);
  CHECK(loaded.rank_reordered == result.rank_reordered);
  for (VertexId s = 0; s < g.num_vertices(); ++s)
    for (VertexId t = 0; t < g.num_vertices(); ++t)
      CHECK(QuerySupport::distance(loaded.labels, s, t) ==
            QuerySupport::distance(result.labels, s, t));
}

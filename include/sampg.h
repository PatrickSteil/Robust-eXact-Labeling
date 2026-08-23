#ifndef RXL_SAMPG_H
#define RXL_SAMPG_H
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "graph.h"
#include "hub_label.h"
#include "types.h"

namespace rxl {

struct SamplingOptions {
  std::size_t initial_trees = 64;
  std::size_t counter_buckets = 16;
  std::size_t discarded_max_buckets = 2;
  std::size_t num_threads = 1;
  std::uint64_t random_seed = 0x5eedULL;
  bool verbose = false;
  std::size_t max_live_trees = std::numeric_limits<std::size_t>::max();
  std::size_t min_tree_vertices = 8;
  bool zero_one_bfs = false;
};

class SamPG {
 public:
  static std::vector<VertexId> build_order(const Graph& graph,
                                           const SamplingOptions& options,
                                           HubLabels& labels,
                                           BuildStatistics& stats);
};

}  // namespace rxl
#endif

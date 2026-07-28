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

// Tuning knobs for SamPG, the sample-tree-based vertex-ordering strategy
// behind PrunedLabeling::compute() (see sampg.cpp for the algorithm).
struct SamplingOptions {
  std::size_t initial_trees = 64;
  std::size_t counter_buckets = 16;
  std::size_t discarded_max_buckets = 2;
  std::size_t num_threads = 1;
  std::uint64_t random_seed = 0x5eedULL;
  bool verbose = false;
  std::size_t max_live_trees = std::numeric_limits<std::size_t>::max();
  std::size_t min_tree_vertices = 8;
};

// SamPG selects hub vertices by growing a rotating forest of pruned
// shortest-path "sample trees" from random roots, scoring each live vertex
// by a robust estimate of how much labeling work removing it would save
// across the current trees, and repeatedly promoting the top-scoring
// vertex to the next rank. This class owns only that sampling/selection
// loop; the hub-label construction for each selected vertex is
// HubExpansion::add_hub (pruned_dijkstra.h), shared with the plain
// degree-order strategy in pruned_labeling.cpp.
class SamPG {
 public:
  // Runs SamPG end to end: builds the vertex order and, along the way, the
  // hub labels for that order (writing into `labels`) and build statistics
  // (writing into `stats`). Returns the selected vertices in rank order.
  //
  // Throws std::invalid_argument if
  // options.discarded_max_buckets >= options.counter_buckets, or if
  // options.counter_buckets exceeds the supported maximum (see sampg.cpp).
  static std::vector<VertexId> build_order(const Graph& graph,
                                           const SamplingOptions& options,
                                           HubLabels& labels,
                                           BuildStatistics& stats);
};

}  // namespace rxl
#endif

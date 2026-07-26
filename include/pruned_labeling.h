#ifndef PRUNED_LABELING_H
#define PRUNED_LABELING_H
#include <cstddef>
#include <cstdint>

#include "graph.h"
#include "hub_label.h"
namespace rxl {
struct SamplingOptions {
  std::size_t initial_trees = 64;
  std::size_t counter_buckets = 16;
  std::size_t discarded_max_buckets = 2;
  std::size_t num_threads = 1;
  std::uint64_t random_seed = 0x5eedULL;
  bool verbose = false;
  std::size_t max_live_trees = 1024;
  std::size_t min_tree_vertices = 8;
};
class PrunedLabeling {
 public:
  static LabelingResult compute(const Graph&, const SamplingOptions& = {});
  static LabelingResult compute_with_degree_order(const Graph&);
  static void reorder_labels_by_rank(LabelingResult&);

 private:
  static std::vector<VertexId> degree_order(const Graph&);
};
}  // namespace rxl
#endif

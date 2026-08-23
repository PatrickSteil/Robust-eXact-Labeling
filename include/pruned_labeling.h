#ifndef PRUNED_LABELING_H
#define PRUNED_LABELING_H
#include <vector>

#include "graph.h"
#include "hub_label.h"
#include "sampg.h"

namespace rxl {

class PrunedLabeling {
 public:
  static LabelingResult compute(const Graph&, const SamplingOptions& = {});
  static LabelingResult compute_with_degree_order(
      const Graph&, const bool verbose = false,
      const bool zero_one_bfs = false);
  static void reorder_labels_by_rank(LabelingResult&);
  static std::vector<VertexId> degree_order(const Graph&);
};
}  // namespace rxl
#endif

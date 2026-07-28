#ifndef PRUNED_LABELING_H
#define PRUNED_LABELING_H
#include <vector>

#include "graph.h"
#include "hub_label.h"
#include "sampg.h"

namespace rxl {

// The two vertex-ordering strategies this project supports, unified behind
// one entry point: SamPG (compute(), the default -- see sampg.h) and plain
// descending degree order (compute_with_degree_order(), mainly useful as a
// baseline). Both build a full hub-labeling via the same underlying
// pruned-Dijkstra hub expansion (HubExpansion::add_hub, pruned_dijkstra.h);
// they differ only in which order they promote vertices to hubs.
class PrunedLabeling {
 public:
  static LabelingResult compute(const Graph&, const SamplingOptions& = {});
  static LabelingResult compute_with_degree_order(const Graph&,
                                                  const bool verbose = false);
  static void reorder_labels_by_rank(LabelingResult&);

 private:
  static std::vector<VertexId> degree_order(const Graph&);
};
}  // namespace rxl
#endif

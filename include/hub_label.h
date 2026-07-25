#ifndef HUB_LABEL_H
#define HUB_LABEL_H
#include <cstddef>
#include <cstdint>
#include <vector>

#include "types.h"
namespace rxl {

// A HubLabel's entries are (hub_id, distance) pairs. hub_id is always the
// *rank* (processing order) of the hub vertex, assigned at the moment that
// vertex is added as a hub -- not that vertex's own array index. Labels
// themselves stay indexed by whatever id space HubLabels is addressed with
// (original vertex ids by default; rank ids only if the caller explicitly
// physically reorders via PrunedLabeling::reorder_labels_by_rank). This
// decoupling means hub ids cluster into small, delta-compressible values
// for important/frequent hubs without needing to renumber any vertex.

struct HubLabel {
  Label forward;
  Label backward;
};
using HubLabels = std::vector<HubLabel>;
struct BuildStatistics {
  std::uint64_t labeling_work = 0;
  std::uint64_t sampling_work = 0;
  std::size_t sampled_trees = 0;
  std::size_t peak_live_trees = 0;
  double ordering_and_labeling_seconds = 0.0;
};
struct LabelingResult {
  HubLabels labels;
  std::vector<VertexId> rank_to_vertex;
  std::vector<Rank> vertex_to_rank;
  BuildStatistics statistics;
  bool rank_reordered = false;
};
}  // namespace rxl
#endif

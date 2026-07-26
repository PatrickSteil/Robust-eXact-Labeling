#ifndef HUB_LABEL_H
#define HUB_LABEL_H
#include <cstddef>
#include <cstdint>
#include <vector>

#include "delta_label.h"
#include "types.h"
namespace rxl {

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
  std::size_t sparse_downgrades = 0;
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

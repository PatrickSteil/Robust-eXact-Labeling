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
  std::size_t counter_buckets = 32;
  std::size_t discarded_max_buckets = 2;
  std::size_t num_threads = 1;
  std::uint64_t random_seed = 0x5eedULL;
  bool verbose = false;
  std::size_t max_live_trees = std::numeric_limits<std::size_t>::max();
  std::size_t min_tree_vertices = 8;
  bool zero_one_bfs = false;

  // --- Batched labeling (BVC-PLL-style), off by default. ---
  // Number of hubs picked "at once" (without letting sample-tree scores
  // react in between) at the start of the run, and processed through the
  // batched, multi-lane labeling kernel instead of one at a time. 1 keeps
  // the original fully-sequential behavior.
  std::size_t initial_batch_size = 1;
  // Upper bound the adaptive-phase batch size grows to (see
  // batch_growth_factor). Ignored while <= initial_batch_size.
  std::size_t max_batch_size = 1;
  // Each adaptive-phase batch's size is multiplied by this factor
  // (rounded up), then clamped to max_batch_size. 1.0 keeps the batch
  // size fixed at initial_batch_size. Values > 1 give small, faithful
  // batches early (when picking the truly-best vertex matters most) and
  // large, cheap batches later (once marginal hub value has flattened).
  double batch_growth_factor = 1.0;
  // When popping a batch from the selection heap, skip a candidate that
  // shares a live sample tree with a vertex already accepted into this
  // batch, and requeue it for the next batch instead. Cuts down on
  // picking several near-duplicate (spatially overlapping) hubs in the
  // same batch, which the batched kernel cannot prune against each other.
  bool batch_diversity_filter = true;
  // Fraction of vertices (by rank) ordered via full adaptive SamPG
  // sampling. The remaining tail is ordered once, cheaply, by freezing
  // whatever SamPG priority scores exist at that point (no further
  // sampling), and labeled through the same batched kernel using
  // tail_batch_size. 1.0 (default) samples the entire order, matching
  // the un-batched algorithm's coverage.
  double sampling_fraction = 1.0;
  // Batch size used once sampling_fraction has been reached. Defaults to
  // max_batch_size when left at 0.
  std::size_t tail_batch_size = 0;
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

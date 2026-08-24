#ifndef RXL_BATCH_PRUNED_LABELING_H
#define RXL_BATCH_PRUNED_LABELING_H
#include <cstdint>
#include <utility>
#include <vector>

#include "graph.h"
#include "hub_label.h"
#include "types.h"

namespace rxl {

// Per-lane scratch: one instance is reused by whichever root currently
// occupies "lane i" of a wave, so its backing arrays only get sized once
// (on first use) rather than once per root. Frontier is DijkstraFrontier or
// ZeroOneBfsFrontier, matching HubExpansion's template parameter.
template <typename Frontier>
struct BatchLaneScratch {
  std::vector<Distance> root_out, root_in, distance;
  std::vector<VertexId> lookup_touched, search_touched;
  Frontier frontier;
  // (target, distance) pairs produced by this lane's forward search (these
  // become backward-label entries for target) and its reverse search
  // (these become forward-label entries). Cleared and refilled every call.
  std::vector<std::pair<VertexId, Distance>> backward_updates;
  std::vector<std::pair<VertexId, Distance>> forward_updates;
};

// Wave-wide scratch, reused across waves and across add_hub_batch calls to
// amortize its O(n) backing arrays (mirrors the touched-list-then-clear
// pattern used elsewhere in this codebase instead of re-zeroing on every
// call). Despite the name, this only ever holds one *wave's* worth of
// pending updates at a time -- see the correctness note on add_hub_batch
// below for why that matters.
struct BatchCommitScratch {
  // pending_*[v]: (position of the contributing root within the current
  // wave, distance) pairs waiting to be committed. Not yet in hub-id
  // order -- add_hub_batch sorts each vertex's list right before writing
  // it into the real label, since DeltaLabel requires hub ids to be
  // pushed in strictly increasing order and a wave's roots can settle v
  // in any relative order.
  std::vector<std::vector<std::pair<std::uint32_t, Distance>>>
      pending_backward;
  std::vector<std::vector<std::pair<std::uint32_t, Distance>>> pending_forward;
  std::vector<VertexId> touched_backward;
  std::vector<VertexId> touched_forward;

  void ensure_size(std::size_t n) {
    if (pending_backward.size() != n) pending_backward.assign(n, {});
    if (pending_forward.size() != n) pending_forward.assign(n, {});
  }
};

// Adds roots.size() hubs "at once": roots[i] is assigned hub id
// first_hub_id + i.
//
// This is a batched, embarrassingly-parallel relative of BVC-PLL
// (Jin et al., ICS'20): roots are processed in waves of up to `threads`
// concurrently-running pruned searches, and EACH WAVE'S RESULTS ARE
// COMMITTED TO `labels` BEFORE THE NEXT WAVE STARTS. Lanes within one wave
// never prune each other -- each one's search only ever consults labels
// that existed before that wave started, which is what lets the search
// phase run with zero synchronization between lanes -- but because waves
// commit sequentially, wave i+1 sees everything wave i produced. The
// "blind spot" where two hubs can end up redundant in the same vertex's
// label is therefore bounded by `threads`, not by roots.size(): asking
// for a bigger batch only means more waves, not a wider blind spot.
// Pruning against everything from earlier batches (and earlier waves of
// this batch) happens exactly as in the sequential algorithm, so query
// results stay exact regardless of batch or wave size.
//
// (An earlier version of this function accumulated updates across the
// whole batch and committed once at the end, making the blind spot equal
// to the full batch size -- that made label size scale with the
// *requested* batch size instead of with hardware parallelism, which is
// a much worse trade than intended. Per-wave commit is the fix.)
//
// lane_scratch is grown to fit internally; pass the same instance across
// calls to reuse its buffers. commit_scratch likewise persists across
// calls; ensure_size(graph.num_vertices()) is called internally.
class BatchHubExpansion {
 public:
  template <typename Frontier>
  static std::uint64_t add_hub_batch(
      const Graph& graph, const std::vector<VertexId>& roots,
      VertexId first_hub_id, HubLabels& labels, std::size_t threads,
      std::vector<BatchLaneScratch<Frontier>>& lane_scratch,
      BatchCommitScratch& commit_scratch);
};

}  // namespace rxl
#endif

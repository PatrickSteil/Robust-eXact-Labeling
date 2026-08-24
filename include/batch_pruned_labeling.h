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

// Batch-wide scratch, reused across add_hub_batch calls to amortize its
// O(n) backing arrays (mirrors the touched-list-then-clear pattern used
// elsewhere in this codebase instead of re-zeroing on every call).
struct BatchCommitScratch {
  // pending_*[v]: (position of the contributing root within the *current*
  // batch, distance) pairs waiting to be committed. Not yet in hub-id
  // order -- add_hub_batch sorts each vertex's list right before writing
  // it into the real label, since DeltaLabel requires hub ids to be
  // pushed in strictly increasing order and this batch's roots can settle
  // v in any relative order.
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
// concurrently-running pruned searches. Unlike BVC-PLL's scatter/gather,
// candidates within the SAME batch never prune each other -- each root's
// search only ever consults labels that existed before the batch started,
// which is what lets the search phase run with zero synchronization
// between lanes. Pruning against everything from *earlier* (already
// committed) batches still happens exactly as in the sequential
// algorithm, so query results stay exact; what a larger batch can cost
// you is label-set optimality, since two same-batch hubs that would have
// pruned one another sequentially can both end up in a vertex's label.
// That's the intended knob: bigger batches trade some label size for more
// parallelism and less synchronization overhead.
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

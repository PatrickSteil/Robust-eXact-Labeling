#ifndef RXL_PRUNED_DIJKSTRA_H
#define RXL_PRUNED_DIJKSTRA_H
#include <cstdint>
#include <vector>

#include "csr_graph.h"
#include "graph.h"
#include "hub_label.h"
#include "indexed_minheap.h"
#include "types.h"

namespace rxl {

// The pruned-Dijkstra hub-expansion step shared by every vertex-ordering
// strategy (SamPG's sampled order in sampg.cpp and the plain degree order
// in pruned_labeling.cpp alike): given the next vertex to promote to "hub"
// (its final rank), run a pruned Dijkstra from it in both directions and
// append it to the label of every vertex it reaches, skipping anything
// already covered by the labels built so far.
class HubExpansion {
 public:
  // True if `label` (the appropriate side of some vertex u's *existing*
  // label) already witnesses a path from the search's root to u of length
  // <= search_distance -- i.e. exploring past u cannot yield a shorter
  // witness, so it can be pruned. `root_distance` gives, for each of the
  // root's own hubs, that hub's distance from the root (kInfinity where
  // unset); it is also used by SamPG's sample-tree growth (sampg.cpp),
  // which prunes the same way while it isn't yet adding a hub.
  static bool covered(const Label& label,
                      const std::vector<Distance>& root_distance,
                      Distance search_distance);

  // Adds `root` (assigned hub id `hub_id`, its final rank) to the label of
  // every vertex it can reach without being pruned by covered(), in both
  // the forward and reverse graphs. Returns the Dijkstra work performed
  // (settled-vertex count plus the out-degree scanned at each), which
  // callers use to balance sampling effort against labeling effort.
  //
  // `root_out`/`root_in` and `distance` are caller-owned scratch buffers
  // that must be all-kInfinity on entry and are restored to that state
  // before returning; `lookup_touched`/`search_touched` and `heap` are
  // likewise reusable scratch, cleared by this call. Reusing the same
  // buffers across calls (rather than reallocating per rank) is what makes
  // this cheap enough to call n times in a row.
  static std::uint64_t add_hub(const Graph& graph, VertexId root,
                               VertexId hub_id, HubLabels& labels,
                               std::vector<Distance>& root_out,
                               std::vector<Distance>& root_in,
                               std::vector<Distance>& distance,
                               std::vector<VertexId>& lookup_touched,
                               std::vector<VertexId>& search_touched,
                               dijkstra_detail::IndexedMinHeap& heap);

 private:
  static std::uint64_t pruned_dijkstra(
      const CsrAdjacency& graph, VertexId root, VertexId hub_id,
      const std::vector<Distance>& root_distance, HubLabels& labels,
      bool forward, std::vector<Distance>& distance,
      std::vector<VertexId>& touched, dijkstra_detail::IndexedMinHeap& heap);
};

}  // namespace rxl
#endif

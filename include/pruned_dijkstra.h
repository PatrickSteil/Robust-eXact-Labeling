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

class HubExpansion {
 public:
  static bool covered(const Label& label,
                      const std::vector<Distance>& root_distance,
                      Distance search_distance);

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

#include "dijkstra.h"

#include <stdexcept>

#include "abstract_dijkstra.h"
namespace rxl {
std::vector<Distance> Dijkstra::shortest_distances(const AdjacencyList& graph,
                                                   VertexId source) {
  if (source >= graph.size()) throw std::out_of_range("Invalid source vertex");
  std::vector<Distance> distance(graph.size(), kInfinity);
  std::vector<VertexId> touched;
  dijkstra_detail::IndexedMinHeap heap(graph.size());
  AbstractDijkstra::search(
      source,
      [&graph](VertexId u) -> const std::vector<Edge>& { return graph[u]; },
      distance, touched, heap);
  return distance;
}
}  // namespace rxl

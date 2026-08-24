#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "csr_graph.h"
#include "types.h"
namespace rxl {
class Dijkstra {
 public:
  static std::vector<Distance> shortest_distances(const CsrAdjacency& graph,
                                                  VertexId source);

  static std::vector<Distance> shortest_distances_01bfs(
      const CsrAdjacency& graph, VertexId source);
};
}  // namespace rxl
#endif

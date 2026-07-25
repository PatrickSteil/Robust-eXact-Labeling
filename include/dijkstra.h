#ifndef DIJKSTRA_H
#define DIJKSTRA_H
#include "types.h"
namespace rxl {
class Dijkstra {
public:
  static std::vector<Distance> shortest_distances(const AdjacencyList &graph,
                                                   VertexId source);
};
} // namespace rxl
#endif

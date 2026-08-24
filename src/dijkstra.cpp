#include "dijkstra.h"

#include <stdexcept>

#include "abstract_dijkstra.h"
#include "search_frontier.h"
namespace rxl {
std::vector<Distance> Dijkstra::shortest_distances(const CsrAdjacency& graph,
                                                   VertexId source) {
  if (source >= graph.num_vertices())
    throw std::out_of_range("Invalid source vertex");
  std::vector<Distance> distance(graph.num_vertices(), kInfinity);
  std::vector<VertexId> touched;
  DijkstraFrontier frontier(graph.num_vertices());
  AbstractDijkstra::search(
      source, [&graph](VertexId u) { return graph[u]; }, distance, touched,
      frontier);
  return distance;
}

std::vector<Distance> Dijkstra::shortest_distances_01bfs(
    const CsrAdjacency& graph, VertexId source) {
  if (source >= graph.num_vertices())
    throw std::out_of_range("Invalid source vertex");
  std::vector<Distance> distance(graph.num_vertices(), kInfinity);
  std::vector<VertexId> touched;
  ZeroOneBfsFrontier frontier(graph.num_vertices());
  AbstractDijkstra::search(
      source, [&graph](VertexId u) { return graph[u]; }, distance, touched,
      frontier);
  return distance;
}
}  // namespace rxl

#include "dijkstra.h"

#include <functional>
#include <queue>
#include <stdexcept>
namespace rxl {
std::vector<Distance> Dijkstra::shortest_distances(const AdjacencyList& graph,
                                                   VertexId source) {
  if (source >= graph.size()) throw std::out_of_range("Invalid source vertex");
  using Item = std::pair<Distance, VertexId>;
  using PQ = std::priority_queue<Item, std::vector<Item>, std::greater<Item>>;

  std::vector<Distance> distance(graph.size(), kInfinity);
  PQ queue;
  distance[source] = 0;
  queue.emplace(0, source);
  while (!queue.empty()) {
    const auto [du, u] = queue.top();
    queue.pop();
    if (du != distance[u]) continue;
    for (const auto& [v, weight] : graph[u]) {
      const std::uint64_t candidate = std::uint64_t(du) + weight;
      if (candidate < distance[v] && candidate < kInfinity) {
        distance[v] = static_cast<Distance>(candidate);
        queue.emplace(distance[v], v);
      }
    }
  }
  return distance;
}
}  // namespace rxl

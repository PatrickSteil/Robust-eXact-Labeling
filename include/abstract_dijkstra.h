#ifndef RXL_ABSTRACT_DIJKSTRA_H
#define RXL_ABSTRACT_DIJKSTRA_H
#include <cassert>
#include <cstdint>
#include <functional>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "types.h"

namespace rxl {
namespace dijkstra_detail {
struct NoOpOnPop {
  void operator()(VertexId, Distance) const {}
};
struct NeverPrune {
  bool operator()(VertexId, Distance) const { return false; }
};
struct NoOpOnSettle {
  void operator()(VertexId, Distance) const {}
};
struct NoOpOnRelax {
  void operator()(VertexId, VertexId, Distance) const {}
};
struct NoOpOnNonImprovingEdge {
  void operator()(VertexId, VertexId, std::uint64_t) const {}
};
}  // namespace dijkstra_detail

class AbstractDijkstra {
 public:
  template <
      typename NeighborsFn, typename OnPop = dijkstra_detail::NoOpOnPop,
      typename ShouldPrune = dijkstra_detail::NeverPrune,
      typename OnSettle = dijkstra_detail::NoOpOnSettle,
      typename OnRelax = dijkstra_detail::NoOpOnRelax,
      typename OnNonImprovingEdge = dijkstra_detail::NoOpOnNonImprovingEdge>
  static void search(VertexId source, NeighborsFn neighbors,
                     std::vector<Distance>& distance,
                     std::vector<VertexId>& touched, OnPop on_pop = {},
                     ShouldPrune should_prune = {}, OnSettle on_settle = {},
                     OnRelax on_relax = {},
                     OnNonImprovingEdge on_non_improving_edge = {}) {
    if (source >= distance.size())
      throw std::out_of_range("AbstractDijkstra: source vertex out of range");

    assert(distance[source] == kInfinity &&
           "AbstractDijkstra: distance[source] must be kInfinity on entry "
           "(buffer not freshly reset)");

    using Item = std::pair<Distance, VertexId>;
    using PQ = std::priority_queue<Item, std::vector<Item>, std::greater<Item>>;
    PQ queue;
    distance[source] = 0;
    touched.push_back(source);
    queue.emplace(0, source);
    while (!queue.empty()) {
      const auto [du, u] = queue.top();
      queue.pop();
      if (du != distance[u]) continue;  // stale entry, superseded already
      on_pop(u, du);
      if (should_prune(u, du)) continue;
      on_settle(u, du);
      for (const auto& [v, w] : neighbors(u)) {
        const std::uint64_t candidate = std::uint64_t(du) + w;
        if (candidate < distance[v] && candidate < kInfinity) {
          if (distance[v] == kInfinity) touched.push_back(v);
          distance[v] = static_cast<Distance>(candidate);
          queue.emplace(distance[v], v);
          on_relax(u, v, distance[v]);
        } else {
          on_non_improving_edge(u, v, candidate);
        }
      }
    }
  }
};
}  // namespace rxl
#endif

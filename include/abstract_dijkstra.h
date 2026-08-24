#ifndef RXL_ABSTRACT_DIJKSTRA_H
#define RXL_ABSTRACT_DIJKSTRA_H
#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <stdexcept>
#include <utility>
#include <vector>

#include "indexed_minheap.h"
#include "search_frontier.h"
#include "types.h"

namespace rxl {

class AbstractDijkstra {
 public:
  template <
      typename Frontier, typename NeighborsFn,
      typename OnPop = dijkstra_detail::NoOpOnPop,
      typename ShouldPrune = dijkstra_detail::NeverPrune,
      typename OnSettle = dijkstra_detail::NoOpOnSettle,
      typename OnRelax = dijkstra_detail::NoOpOnRelax,
      typename OnNonImprovingEdge = dijkstra_detail::NoOpOnNonImprovingEdge>
  static void search(VertexId source, NeighborsFn neighbors,
                     std::vector<Distance>& distance,
                     std::vector<VertexId>& touched, Frontier& frontier,
                     OnPop on_pop = {}, ShouldPrune should_prune = {},
                     OnSettle on_settle = {}, OnRelax on_relax = {},
                     OnNonImprovingEdge on_non_improving_edge = {}) {
    if (source >= distance.size())
      throw std::out_of_range("AbstractDijkstra: source vertex out of range");

    assert(distance[source] == kInfinity &&
           "AbstractDijkstra: distance[source] must be kInfinity on entry "
           "(buffer not freshly reset)");
    assert(frontier.empty() &&
           "AbstractDijkstra: frontier must be empty on entry -- reuse the "
           "same instance across calls rather than passing a fresh or "
           "in-progress one");
    distance[source] = 0;
    touched.push_back(source);
    frontier.relax(source, 0, 0);

    while (!frontier.empty()) {
      const auto [du, u] = frontier.pop();
      if (du != distance[u]) continue;  // stale entry, see comment above
      on_pop(u, du);
      if (should_prune(u, du)) continue;
      on_settle(u, du);

      const auto& edges = neighbors(u);
      const std::size_t num_edges = edges.size();
      constexpr std::size_t kRelaxPrefetchAhead = 4;
      if (kRelaxPrefetchAhead < num_edges) {
#pragma GCC unroll(kRelaxPrefetchAhead)
        for (std::size_t k = 0; k < kRelaxPrefetchAhead; ++k)
          __builtin_prefetch(&distance[edges[k].first], /*rw=*/0,
                             /*locality=*/1);
      }

      for (std::size_t i = 0; i < num_edges; ++i) {
        if (i + kRelaxPrefetchAhead < num_edges)
          __builtin_prefetch(&distance[edges[i + kRelaxPrefetchAhead].first], 0,
                             1);

        const auto& [v, w] = edges[i];
        const std::uint64_t candidate = std::uint64_t(du) + w;
        if (candidate < distance[v] && candidate < kInfinity) {
          if (distance[v] == kInfinity) touched.push_back(v);
          distance[v] = static_cast<Distance>(candidate);
          frontier.relax(v, distance[v], w);
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

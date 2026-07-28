#ifndef RXL_TEST_COMMON_H
#define RXL_TEST_COMMON_H

#include <iostream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>

#include "dijkstra.h"
#include "graph.h"
#include "query_support.h"

using namespace rxl;

#define CHECK(x)                                                            \
  do {                                                                      \
    if (!(x)) throw std::runtime_error(std::string("CHECK failed: ") + #x); \
  } while (0)

// Common helper functions (inline to avoid duplicate symbols)
inline Graph make_graph(
    std::size_t n,
    const std::vector<std::tuple<VertexId, VertexId, Distance>>& arcs) {
  AdjacencyList a(n), r(n);
  for (auto [u, v, w] : arcs) {
    a[u].emplace_back(v, w);
    r[v].emplace_back(u, w);
  }
  return Graph(std::move(a), std::move(r));
}

inline void check_exact(const Graph& g, const HubLabels& labels) {
  for (VertexId s = 0; s < g.num_vertices(); ++s) {
    auto truth = Dijkstra::shortest_distances(g.adjacency(), s);
    for (VertexId t = 0; t < g.num_vertices(); ++t)
      CHECK(QuerySupport::distance(labels, s, t) == truth[t]);
  }
}

#endif  // RXL_TEST_COMMON_H

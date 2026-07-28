#include "csr_graph.h"

#include <algorithm>
#include <utility>

namespace rxl {

CsrAdjacency::CsrAdjacency(AdjacencyList lists) {
  const std::size_t n = lists.size();
  offsets_.assign(n + 1, 0);
  for (std::size_t u = 0; u < n; ++u)
    offsets_[u + 1] = offsets_[u] + lists[u].size();
  edges_.resize(offsets_[n]);
  for (std::size_t u = 0; u < n; ++u) {
    std::move(lists[u].begin(), lists[u].end(),
              edges_.begin() + static_cast<std::ptrdiff_t>(offsets_[u]));
    std::vector<Edge>().swap(lists[u]);
  }
}

CsrAdjacency::CsrAdjacency(std::vector<Edge> edges,
                           std::vector<std::size_t> offsets)
    : edges_(std::move(edges)), offsets_(std::move(offsets)) {}

CsrAdjacency CsrAdjacency::reversed() const {
  const std::size_t n = num_vertices();
  std::vector<std::size_t> out_offsets(n + 1, 0);
  for (const Edge& e : edges_) ++out_offsets[e.first + 1];
  for (std::size_t v = 0; v < n; ++v) out_offsets[v + 1] += out_offsets[v];

  std::vector<Edge> out_edges(edges_.size());
  std::vector<std::size_t> cursor(out_offsets.begin(), out_offsets.end() - 1);
  for (VertexId u = 0; u < n; ++u) {
    for (std::size_t i = offsets_[u]; i < offsets_[u + 1]; ++i) {
      const Edge& e = edges_[i];
      out_edges[cursor[e.first]++] = Edge(u, e.second);
    }
  }
  return CsrAdjacency(std::move(out_edges), std::move(out_offsets));
}

}  // namespace rxl

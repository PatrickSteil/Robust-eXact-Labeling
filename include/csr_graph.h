#ifndef RXL_CSR_GRAPH_H
#define RXL_CSR_GRAPH_H
#include <cstddef>
#include <vector>

#include "types.h"

namespace rxl {

// Forward-star (CSR) adjacency: every vertex's outgoing edges live in one
// contiguous, shared `edges_` array, ordered by source vertex, with
// `offsets_[u] .. offsets_[u + 1]` giving vertex u's slice of it. Compared
// to the "builder" representation (AdjacencyList, i.e. vector<vector<Edge>>,
// still used by the graph-format loaders while arc counts are unknown),
// this is a single allocation instead of one per vertex, keeps a vertex's
// edges contiguous with its neighbors' (good iteration locality for
// Dijkstra-style traversal), and is considerably more compact (no per-
// vector capacity/pointer overhead). The trade-off is that it's read-only
// once built: growing one vertex's edge list means rebuilding the whole
// structure, which is why construction always happens from a fully
// populated AdjacencyList rather than incrementally.
class CsrAdjacency {
 public:
  // A read-only view of one vertex's outgoing edges: a [begin, end) slice
  // of the shared edges_ array. Deliberately mimics enough of
  // std::vector<Edge>'s interface (size/empty/operator[]/iteration) to be a
  // drop-in replacement for the old "adjacency()[u]" access pattern.
  class Neighbors {
   public:
    using const_iterator = std::vector<Edge>::const_iterator;

    Neighbors() = default;
    Neighbors(const_iterator begin, const_iterator end)
        : begin_(begin), end_(end) {}

    const_iterator begin() const { return begin_; }
    const_iterator end() const { return end_; }
    std::size_t size() const {
      return static_cast<std::size_t>(end_ - begin_);
    }
    bool empty() const { return begin_ == end_; }
    const Edge& operator[](std::size_t i) const { return begin_[i]; }

   private:
    const_iterator begin_{};
    const_iterator end_{};
  };

  CsrAdjacency() : offsets_(1, 0) {}

  // Compacts a builder-style adjacency list into CSR form. Takes `lists` by
  // value so a caller with an rvalue (the common case: a loader's local
  // builder) doesn't pay for an extra copy; each per-vertex vector is
  // released as soon as its edges are copied out, so peak memory only ever
  // holds the not-yet-consumed tail of `lists` alongside the growing CSR
  // arrays rather than two full copies of the edge set.
  explicit CsrAdjacency(AdjacencyList lists);

  // Wraps already-packed edges + offsets directly. `offsets` must have
  // exactly `offsets.back() == edges.size()` and one entry per vertex plus
  // a final sentinel. Used internally by reversed().
  CsrAdjacency(std::vector<Edge> edges, std::vector<std::size_t> offsets);

  std::size_t num_vertices() const { return offsets_.size() - 1; }
  // vector-like alias so generic code (e.g. Dijkstra::shortest_distances)
  // can treat a CsrAdjacency and a raw AdjacencyList interchangeably.
  std::size_t size() const { return num_vertices(); }
  std::size_t num_edges() const { return edges_.size(); }

  Neighbors operator[](VertexId u) const {
    return Neighbors(edges_.begin() + static_cast<std::ptrdiff_t>(offsets_[u]),
                     edges_.begin() +
                         static_cast<std::ptrdiff_t>(offsets_[u + 1]));
  }

  // Builds the transposed graph (each edge (u, v, w) becomes (v, u, w))
  // directly in CSR form via a counting sort over destinations, without
  // ever materializing a vector<vector<Edge>> intermediate.
  CsrAdjacency reversed() const;

  const std::vector<Edge>& edges() const { return edges_; }
  const std::vector<std::size_t>& offsets() const { return offsets_; }

 private:
  std::vector<Edge> edges_;
  std::vector<std::size_t> offsets_;  // size num_vertices() + 1
};

}  // namespace rxl
#endif

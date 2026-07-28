#ifndef RXL_CSR_GRAPH_H
#define RXL_CSR_GRAPH_H
#include <cstddef>
#include <vector>

#include "types.h"

namespace rxl {

class CsrAdjacency {
 public:
  class Neighbors {
   public:
    using const_iterator = std::vector<Edge>::const_iterator;

    Neighbors() = default;
    Neighbors(const_iterator begin, const_iterator end)
        : begin_(begin), end_(end) {}

    const_iterator begin() const { return begin_; }
    const_iterator end() const { return end_; }
    std::size_t size() const { return static_cast<std::size_t>(end_ - begin_); }
    bool empty() const { return begin_ == end_; }
    const Edge& operator[](std::size_t i) const { return begin_[i]; }

   private:
    const_iterator begin_{};
    const_iterator end_{};
  };

  CsrAdjacency() : offsets_(1, 0) {}

  explicit CsrAdjacency(AdjacencyList lists);

  CsrAdjacency(std::vector<Edge> edges, std::vector<std::size_t> offsets);

  std::size_t num_vertices() const { return offsets_.size() - 1; }
  std::size_t size() const { return num_vertices(); }
  std::size_t num_edges() const { return edges_.size(); }

  Neighbors operator[](VertexId u) const {
    return Neighbors(
        edges_.begin() + static_cast<std::ptrdiff_t>(offsets_[u]),
        edges_.begin() + static_cast<std::ptrdiff_t>(offsets_[u + 1]));
  }

  CsrAdjacency reversed() const;

  const std::vector<Edge>& edges() const { return edges_; }
  const std::vector<std::size_t>& offsets() const { return offsets_; }

 private:
  std::vector<Edge> edges_;
  std::vector<std::size_t> offsets_;  // size num_vertices() + 1
};

}  // namespace rxl
#endif

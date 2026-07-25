#ifndef GRAPH_H
#define GRAPH_H

#include "types.h"
#include <cstddef>
#include <string>
#include <vector>

namespace rxl {
class Graph {
public:
  explicit Graph(const std::string &dimacs_file);
  Graph(AdjacencyList adjacency, AdjacencyList reverse);

  std::size_t num_vertices() const { return adjacency_.size(); }
  std::size_t num_edges() const;
  bool is_weighted() const { return weighted_; }
  const AdjacencyList &adjacency() const { return adjacency_; }
  const AdjacencyList &reverse_adjacency() const { return reverse_; }

  // Mutates the graph so rank_to_vertex[rank] receives ID rank. Returns the
  // old-ID -> new-ID permutation.
  std::vector<VertexId>
  reorder_by_rank(const std::vector<VertexId> &rank_to_vertex);

private:
  AdjacencyList adjacency_;
  AdjacencyList reverse_;
  bool weighted_ = false;
  void build_reverse();
  void update_weighted_flag();
};
} // namespace rxl
#endif

#ifndef GRAPH_H
#define GRAPH_H

#include <cstddef>
#include <string>
#include <vector>

#include "csr_graph.h"
#include "types.h"

namespace rxl {

enum class GraphFormat { Dimacs, Snap, Metis, EdgeList };

GraphFormat parse_graph_format(const std::string& name);

class Graph {
 public:
  explicit Graph(const std::string& file,
                 GraphFormat format = GraphFormat::Dimacs);
  Graph(AdjacencyList adjacency, AdjacencyList reverse);

  std::size_t num_vertices() const { return adjacency_.num_vertices(); }
  std::size_t num_edges() const { return adjacency_.num_edges(); }
  bool is_weighted() const { return weighted_; }
  bool is_zero_one_weighted() const;
  const CsrAdjacency& adjacency() const { return adjacency_; }
  const CsrAdjacency& reverse_adjacency() const { return reverse_; }

  std::vector<VertexId> reorder_by_rank(
      const std::vector<VertexId>& rank_to_vertex);

 private:
  CsrAdjacency adjacency_;
  CsrAdjacency reverse_;
  bool weighted_ = false;

  void finalize(AdjacencyList adjacency);
  void update_weighted_flag();

  AdjacencyList load_dimacs(const std::string& file);
  AdjacencyList load_snap(const std::string& file);
  AdjacencyList load_metis(const std::string& file);
  AdjacencyList load_edge_list(const std::string& file);
};
}  // namespace rxl
#endif

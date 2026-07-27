#ifndef GRAPH_H
#define GRAPH_H

#include <cstddef>
#include <string>
#include <vector>

#include "types.h"

namespace rxl {

enum class GraphFormat { Dimacs, Snap, Metis, EdgeList };

GraphFormat parse_graph_format(const std::string& name);

class Graph {
 public:
  explicit Graph(const std::string& file,
                 GraphFormat format = GraphFormat::Dimacs);
  Graph(AdjacencyList adjacency, AdjacencyList reverse);

  std::size_t num_vertices() const { return adjacency_.size(); }
  std::size_t num_edges() const;
  bool is_weighted() const { return weighted_; }
  const AdjacencyList& adjacency() const { return adjacency_; }
  const AdjacencyList& reverse_adjacency() const { return reverse_; }

  std::vector<VertexId> reorder_by_rank(
      const std::vector<VertexId>& rank_to_vertex);

 private:
  AdjacencyList adjacency_;
  AdjacencyList reverse_;
  bool weighted_ = false;
  void build_reverse();
  void update_weighted_flag();
  void load_dimacs(const std::string& file);
  void load_snap(const std::string& file);
  void load_metis(const std::string& file);
  void load_edge_list(const std::string& file);
};
}  // namespace rxl
#endif

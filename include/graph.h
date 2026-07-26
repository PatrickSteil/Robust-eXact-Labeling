#ifndef GRAPH_H
#define GRAPH_H

#include "types.h"
#include <cstddef>
#include <string>
#include <vector>

namespace rxl {

// Supported on-disk graph formats.
//   Dimacs  - DIMACS ".gr"/".graph" shortest-path format ('p sp n m' header,
//             'a from to weight' arcs, 1-based, directed, weighted).
//   Snap    - SNAP edge-list format ('#'-prefixed comments, one
//             'from to [weight]' pair per line, vertex IDs are arbitrary
//             integers and need not be contiguous or zero-based).
//   Metis   - METIS graph format ('% ' comments, 'n m [fmt]' header
//             followed by n adjacency lines, 1-based, fmt selects whether
//             edge weights are present).
//   EdgeList - simple CSV "from,to,weight" (weight optional, defaults to 1;
//             vertex IDs are arbitrary integers, same handling as Snap).
enum class GraphFormat { Dimacs, Snap, Metis, EdgeList };

// Parses a --format style CLI value ("dimacs", "snap", "metis", "csv"/
// "edgelist") into a GraphFormat. Throws std::invalid_argument on an
// unrecognized value.
GraphFormat parse_graph_format(const std::string &name);

class Graph {
public:
  explicit Graph(const std::string &file,
                 GraphFormat format = GraphFormat::Dimacs);
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
  void load_dimacs(const std::string &file);
  void load_snap(const std::string &file);
  void load_metis(const std::string &file);
  void load_edge_list(const std::string &file);
};
} // namespace rxl
#endif

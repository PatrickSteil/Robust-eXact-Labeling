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
  // Takes both directions as "builder" adjacency lists (see AdjacencyList
  // in types.h) and compacts each into CSR storage. Unlike the file-loading
  // constructor, this one trusts the caller's `reverse` instead of deriving
  // it from `adjacency`; the arc-count check below is a cheap sanity check
  // against a mismatched pair, not a guarantee that `reverse` is truly the
  // transpose.
  Graph(AdjacencyList adjacency, AdjacencyList reverse);

  std::size_t num_vertices() const { return adjacency_.num_vertices(); }
  std::size_t num_edges() const { return adjacency_.num_edges(); }
  bool is_weighted() const { return weighted_; }
  const CsrAdjacency& adjacency() const { return adjacency_; }
  const CsrAdjacency& reverse_adjacency() const { return reverse_; }

  std::vector<VertexId> reorder_by_rank(
      const std::vector<VertexId>& rank_to_vertex);

 private:
  CsrAdjacency adjacency_;
  CsrAdjacency reverse_;
  bool weighted_ = false;

  // Compacts `adjacency` into adjacency_, derives reverse_ from it, and
  // refreshes weighted_. Used by the file-loading constructor, once each
  // load_*() has finished populating a builder-style AdjacencyList.
  void finalize(AdjacencyList adjacency);
  void update_weighted_flag();

  // Each loader parses its format into a local "builder" AdjacencyList
  // (arc counts aren't known upfront, so per-vertex vectors grow via
  // emplace_back) and returns it for finalize() to compact into CSR.
  AdjacencyList load_dimacs(const std::string& file);
  AdjacencyList load_snap(const std::string& file);
  AdjacencyList load_metis(const std::string& file);
  AdjacencyList load_edge_list(const std::string& file);
};
}  // namespace rxl
#endif

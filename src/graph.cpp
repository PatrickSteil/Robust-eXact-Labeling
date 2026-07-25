#include "graph.h"
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace rxl {
Graph::Graph(const std::string &file) {
  std::ifstream input(file);
  if (!input) throw std::runtime_error("Could not open file: " + file);
  std::string line;
  std::uint64_t n = 0;
  bool saw_problem = false;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == 'c') continue;
    std::istringstream stream(line);
    char command = 0;
    stream >> command;
    if (command == 'p') {
      std::string format;
      std::uint64_t m;
      if (!(stream >> format >> n >> m) || n > kInvalidVertex)
        throw std::runtime_error("Malformed or oversized DIMACS problem line");
      adjacency_.assign(static_cast<std::size_t>(n), {});
      saw_problem = true;
    } else if (command == 'a') {
      std::uint64_t from, to, weight;
      if (!saw_problem || !(stream >> from >> to >> weight) || from == 0 ||
          to == 0 || from > n || to > n || weight == 0 || weight >= kInfinity)
        throw std::runtime_error("Malformed DIMACS arc: " + line);
      // DIMACS is 1-based; every internal API is 0-based.
      adjacency_[static_cast<VertexId>(from - 1)].emplace_back(
          static_cast<VertexId>(to - 1), static_cast<Distance>(weight));
    }
  }
  if (!saw_problem) throw std::runtime_error("DIMACS file has no problem line");
  build_reverse();
  update_weighted_flag();
}

Graph::Graph(AdjacencyList adjacency, AdjacencyList reverse)
    : adjacency_(std::move(adjacency)), reverse_(std::move(reverse)) {
  if (adjacency_.size() != reverse_.size())
    throw std::invalid_argument("Forward and reverse graph sizes differ");
  update_weighted_flag();
}

std::size_t Graph::num_edges() const {
  std::size_t result = 0;
  for (const auto &edges : adjacency_) result += edges.size();
  return result;
}

void Graph::build_reverse() {
  reverse_.assign(adjacency_.size(), {});
  // Count in-degrees first and reserve exactly, so filling reverse_[v] below
  // never triggers the repeated reallocate-and-copy growth a bare sequence
  // of emplace_back calls would otherwise cause.
  std::vector<std::size_t> in_degree(adjacency_.size(), 0);
  for (const auto &edges : adjacency_)
    for (const auto &[v, w] : edges) ++in_degree[v];
  for (VertexId v = 0; v < adjacency_.size(); ++v) reverse_[v].reserve(in_degree[v]);
  for (VertexId u = 0; u < adjacency_.size(); ++u)
    for (const auto &[v, w] : adjacency_[u]) reverse_[v].emplace_back(u, w);
}

void Graph::update_weighted_flag() {
  weighted_ = false;
  for (const auto &edges : adjacency_)
    for (const auto &[unused, weight] : edges)
      if (weight != 1) { weighted_ = true; return; }
}

std::vector<VertexId>
Graph::reorder_by_rank(const std::vector<VertexId> &rank_to_vertex) {
  const std::size_t n = num_vertices();
  if (rank_to_vertex.size() != n)
    throw std::invalid_argument("Rank permutation has the wrong size");
  std::vector<VertexId> old_to_new(n, kInvalidVertex);
  for (VertexId rank = 0; rank < n; ++rank) {
    const VertexId old = rank_to_vertex[rank];
    if (old >= n || old_to_new[old] != kInvalidVertex)
      throw std::invalid_argument("Invalid rank permutation");
    old_to_new[old] = rank;
  }
  AdjacencyList reordered(n);
  for (VertexId old_u = 0; old_u < n; ++old_u) {
    auto &target = reordered[old_to_new[old_u]];
    target.reserve(adjacency_[old_u].size());
    for (const auto &[old_v, weight] : adjacency_[old_u])
      target.emplace_back(old_to_new[old_v], weight);
  }
  adjacency_ = std::move(reordered);
  build_reverse();
  return old_to_new;
}
} // namespace rxl

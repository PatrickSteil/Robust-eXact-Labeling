#include "graph.h"

#include <algorithm>
#include <array>
#include <charconv>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace rxl {
namespace {
std::vector<std::string> split(const std::string& line,
                               const char* delimiters) {
  std::vector<std::string> fields;
  std::size_t pos = 0;
  while (pos < line.size()) {
    std::size_t next = line.find_first_of(delimiters, pos);
    if (next == std::string::npos) next = line.size();
    if (next > pos) fields.push_back(line.substr(pos, next - pos));
    pos = next + 1;
  }
  return fields;
}

std::uint64_t parse_uint(const std::string& field, const std::string& context) {
  std::uint64_t value = 0;
  const auto* begin = field.data();
  const auto* end = field.data() + field.size();
  const auto result = std::from_chars(begin, end, value);
  if (result.ec != std::errc() || result.ptr != end)
    throw std::runtime_error("Malformed integer in " + context + ": " + field);
  return value;
}
}  // namespace

GraphFormat parse_graph_format(const std::string& name) {
  if (name == "dimacs") return GraphFormat::Dimacs;
  if (name == "snap") return GraphFormat::Snap;
  if (name == "metis") return GraphFormat::Metis;
  if (name == "csv" || name == "edgelist" || name == "edge-list")
    return GraphFormat::EdgeList;
  throw std::invalid_argument("Unknown graph format '" + name +
                              "' (expected: dimacs, snap, metis, csv)");
}

Graph::Graph(const std::string& file, GraphFormat format) {
  switch (format) {
    case GraphFormat::Dimacs:
      finalize(load_dimacs(file));
      break;
    case GraphFormat::Snap:
      finalize(load_snap(file));
      break;
    case GraphFormat::Metis:
      finalize(load_metis(file));
      break;
    case GraphFormat::EdgeList:
      finalize(load_edge_list(file));
      break;
  }
}

AdjacencyList Graph::load_dimacs(const std::string& file) {
  std::ifstream input(file);
  if (!input) throw std::runtime_error("Could not open file: " + file);
  AdjacencyList adjacency;
  std::string line;
  std::uint64_t n = 0;
  bool saw_problem = false;
  while (std::getline(input, line)) {
    if (line.empty() || line[0] == 'c') continue;
    std::istringstream stream(line);
    char command = 0;
    stream >> command;
    if (command == 'p') {
      if (saw_problem)
        throw std::runtime_error("DIMACS file has more than one problem line");
      std::string format;
      std::uint64_t m;
      if (!(stream >> format >> n >> m) || n > kInvalidVertex)
        throw std::runtime_error("Malformed or oversized DIMACS problem line");
      if (format != "sp")
        throw std::runtime_error(
            "Unsupported DIMACS problem format (expected 'sp'): " + format);
      adjacency.assign(static_cast<std::size_t>(n), {});
      saw_problem = true;
    } else if (command == 'a') {
      std::uint64_t from, to, weight;
      if (!saw_problem || !(stream >> from >> to >> weight) || from == 0 ||
          to == 0 || from > n || to > n || weight >= kInfinity)
        throw std::runtime_error("Malformed DIMACS arc: " + line);
      // DIMACS is 1-based; every internal API is 0-based.
      adjacency[static_cast<VertexId>(from - 1)].emplace_back(
          static_cast<VertexId>(to - 1), static_cast<Distance>(weight));
    }
  }
  if (!saw_problem) throw std::runtime_error("DIMACS file has no problem line");
  return adjacency;
}

// SNAP edge lists (https://snap.stanford.edu/data/) list one directed edge
// per line as whitespace-separated "from to [weight]", with '#'-prefixed
// comment/header lines. Vertex IDs are arbitrary 64-bit integers that need
// not be contiguous or start at 0, so IDs are compacted on the fly in the
// order they're first seen.
AdjacencyList Graph::load_snap(const std::string& file) {
  std::ifstream input(file);
  if (!input) throw std::runtime_error("Could not open file: " + file);
  std::unordered_map<std::uint64_t, VertexId> id_map;
  std::vector<std::array<std::uint64_t, 3>> raw_edges;  // from, to, weight
  auto intern = [&](std::uint64_t raw) -> VertexId {
    auto [it, inserted] =
        id_map.try_emplace(raw, static_cast<VertexId>(id_map.size()));
    if (inserted && id_map.size() - 1 > kInvalidVertex)
      throw std::runtime_error("SNAP file has too many distinct vertex IDs");
    return it->second;
  };
  std::string line;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    const auto fields = split(line, " \t");
    if (fields.size() != 2 && fields.size() != 3)
      throw std::runtime_error("Malformed SNAP edge line: " + line);
    const std::uint64_t from = parse_uint(fields[0], "SNAP edge");
    const std::uint64_t to = parse_uint(fields[1], "SNAP edge");
    const std::uint64_t weight =
        fields.size() == 3 ? parse_uint(fields[2], "SNAP edge weight") : 1;
    if (weight == 0 || weight >= kInfinity)
      throw std::runtime_error("SNAP edge weight out of range: " + line);
    raw_edges.push_back({from, to, weight});
    intern(from);
    intern(to);
  }
  AdjacencyList adjacency(id_map.size());
  for (const auto& [from, to, weight] : raw_edges)
    adjacency[id_map.at(from)].emplace_back(id_map.at(to),
                                            static_cast<Distance>(weight));
  return adjacency;
}

// METIS graph format (https://github.com/KarypisLab/METIS manual, Section
// 5): a header line "n m [fmt]" (fmt's ones digit is 1 if edges carry
// weights) followed by exactly n lines, one per 1-based vertex, each
// listing that vertex's neighbors (and, if weighted, an interleaved weight
// after every neighbor). '%'-prefixed lines are comments. Since METIS
// stores undirected graphs by listing each edge in both endpoints' lines,
// parsing every vertex line as a set of outgoing arcs naturally reproduces
// both directions.
AdjacencyList Graph::load_metis(const std::string& file) {
  std::ifstream input(file);
  if (!input) throw std::runtime_error("Could not open file: " + file);
  AdjacencyList adjacency;
  std::string line;
  std::uint64_t n = 0, m = 0;
  int fmt = 0;
  bool saw_header = false;
  std::uint64_t vertex = 0;  // 0-based count of adjacency lines consumed
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '%') continue;
    if (!saw_header) {
      const auto fields = split(line, " \t");
      if (fields.size() < 2)
        throw std::runtime_error("Malformed METIS header line: " + line);
      n = parse_uint(fields[0], "METIS header");
      m = parse_uint(fields[1], "METIS header");
      if (n > kInvalidVertex)
        throw std::runtime_error("METIS graph has too many vertices");
      if (fields.size() >= 3)
        fmt = static_cast<int>(parse_uint(fields[2], "METIS header"));
      if (fmt != 0 && fmt != 1)
        throw std::runtime_error(
            "Unsupported METIS fmt (only unweighted '0' or edge-weighted "
            "'1' are supported): " +
            fields[2]);
      adjacency.assign(static_cast<std::size_t>(n), {});
      saw_header = true;
      continue;
    }
    if (vertex >= n)
      throw std::runtime_error(
          "METIS file has more adjacency lines than its header declares");
    const bool weighted = fmt == 1;
    const auto fields = split(line, " \t");
    if (weighted && fields.size() % 2 != 0)
      throw std::runtime_error("Malformed METIS weighted adjacency line: " +
                               line);
    for (std::size_t i = 0; i < fields.size(); i += weighted ? 2 : 1) {
      const std::uint64_t to = parse_uint(fields[i], "METIS adjacency line");
      const std::uint64_t weight =
          weighted ? parse_uint(fields[i + 1], "METIS adjacency line") : 1;
      if (to == 0 || to > n || weight == 0 || weight >= kInfinity)
        throw std::runtime_error("Malformed METIS neighbor: " + line);
      adjacency[static_cast<VertexId>(vertex)].emplace_back(
          static_cast<VertexId>(to - 1), static_cast<Distance>(weight));
    }
    ++vertex;
  }
  if (!saw_header) throw std::runtime_error("METIS file has no header line");
  if (vertex != n)
    throw std::runtime_error(
        "METIS file has fewer adjacency lines than its header declares");
  (void)m;  // m (declared arc count) isn't cross-checked against the body.
  return adjacency;
}

// Simple "from,to[,weight]" CSV edge list (weight optional, defaults to 1).
// An optional non-numeric header row ("from,to,weight") is skipped. As with
// SNAP, vertex IDs are arbitrary integers and get compacted on the fly.
AdjacencyList Graph::load_edge_list(const std::string& file) {
  std::ifstream input(file);
  if (!input) throw std::runtime_error("Could not open file: " + file);
  std::unordered_map<std::uint64_t, VertexId> id_map;
  std::vector<std::array<std::uint64_t, 3>> raw_edges;
  auto intern = [&](std::uint64_t raw) -> VertexId {
    auto [it, inserted] =
        id_map.try_emplace(raw, static_cast<VertexId>(id_map.size()));
    if (inserted && id_map.size() - 1 > kInvalidVertex)
      throw std::runtime_error("CSV file has too many distinct vertex IDs");
    return it->second;
  };
  std::string line;
  bool first_line = true;
  while (std::getline(input, line)) {
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line.empty() || line[0] == '#') continue;
    auto fields = split(line, ",");
    // Trim surrounding whitespace from each field so "from, to, weight"
    // (with spaces after the commas) parses the same as "from,to,weight".
    for (auto& field : fields) {
      const auto begin = field.find_first_not_of(" \t");
      const auto end = field.find_last_not_of(" \t");
      field = begin == std::string::npos ? ""
                                         : field.substr(begin, end - begin + 1);
    }
    if (first_line) {
      first_line = false;
      // A header like "from,to,weight" has non-numeric fields; skip it.
      std::uint64_t discard = 0;
      const bool numeric =
          !fields.empty() &&
          std::from_chars(fields[0].data(), fields[0].data() + fields[0].size(),
                          discard)
                  .ec == std::errc();
      if (!numeric) continue;
    }
    if (fields.size() != 2 && fields.size() != 3)
      throw std::runtime_error("Malformed CSV edge line: " + line);
    const std::uint64_t from = parse_uint(fields[0], "CSV edge");
    const std::uint64_t to = parse_uint(fields[1], "CSV edge");
    const std::uint64_t weight =
        fields.size() == 3 ? parse_uint(fields[2], "CSV edge weight") : 1;
    if (weight == 0 || weight >= kInfinity)
      throw std::runtime_error("CSV edge weight out of range: " + line);
    raw_edges.push_back({from, to, weight});
    intern(from);
    intern(to);
  }
  AdjacencyList adjacency(id_map.size());
  for (const auto& [from, to, weight] : raw_edges)
    adjacency[id_map.at(from)].emplace_back(id_map.at(to),
                                            static_cast<Distance>(weight));
  return adjacency;
}

Graph::Graph(AdjacencyList adjacency, AdjacencyList reverse) {
  if (adjacency.size() != reverse.size())
    throw std::invalid_argument("Forward and reverse graph sizes differ");
  // A true reverse always has exactly as many arcs as the forward graph
  // (each forward arc corresponds to exactly one reverse arc), so this is
  // a cheap, if partial, sanity check against a caller-assembled mismatch;
  // it won't catch a reverse with the right arc count but wrong endpoints.
  std::size_t forward_edges = 0, reverse_edges = 0;
  for (const auto& edges : adjacency) forward_edges += edges.size();
  for (const auto& edges : reverse) reverse_edges += edges.size();
  if (forward_edges != reverse_edges)
    throw std::invalid_argument("Forward and reverse graph arc counts differ");
  adjacency_ = CsrAdjacency(std::move(adjacency));
  reverse_ = CsrAdjacency(std::move(reverse));
  update_weighted_flag();
}

void Graph::finalize(AdjacencyList adjacency) {
  adjacency_ = CsrAdjacency(std::move(adjacency));
  reverse_ = adjacency_.reversed();
  update_weighted_flag();
}

void Graph::update_weighted_flag() {
  weighted_ = false;
  for (const Edge& edge : adjacency_.edges())
    if (edge.second != 1) {
      weighted_ = true;
      return;
    }
}

bool Graph::is_zero_one_weighted() const {
  for (const Edge& edge : adjacency_.edges())
    if (edge.second != 0 && edge.second != 1) return false;
  return true;
}

std::vector<VertexId> Graph::reorder_by_rank(
    const std::vector<VertexId>& rank_to_vertex) {
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
    const auto edges = adjacency_[old_u];
    auto& target = reordered[old_to_new[old_u]];
    target.reserve(edges.size());
    for (const auto& [old_v, weight] : edges)
      target.emplace_back(old_to_new[old_v], weight);
  }
  adjacency_ = CsrAdjacency(std::move(reordered));
  reverse_ = adjacency_.reversed();
  return old_to_new;
}
}  // namespace rxl

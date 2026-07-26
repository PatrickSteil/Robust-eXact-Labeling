#include "statistics.h"

#include <algorithm>
#include <limits>

namespace rxl {
GraphStatistics compute_graph_statistics(const Graph& graph) {
  GraphStatistics s;
  s.vertices = graph.num_vertices();
  s.arcs = graph.num_edges();
  if (s.vertices == 0) return s;
  s.min_out_degree = std::numeric_limits<std::size_t>::max();
  for (VertexId v = 0; v < s.vertices; ++v) {
    const auto degree = graph.adjacency()[v].size();
    s.min_out_degree = std::min(s.min_out_degree, degree);
    s.max_out_degree = std::max(s.max_out_degree, degree);
    if (degree == 0 && graph.reverse_adjacency()[v].empty())
      ++s.isolated_vertices;
  }
  s.average_out_degree = double(s.arcs) / s.vertices;
  return s;
}

LabelStatistics compute_label_statistics(const HubLabels& labels) {
  LabelStatistics s;
  for (const auto& l : labels) {
    s.forward_entries += l.forward.size();
    s.backward_entries += l.backward.size();
    s.max_forward_size = std::max(s.max_forward_size, l.forward.size());
    s.max_backward_size = std::max(s.max_backward_size, l.backward.size());
  }
  if (!labels.empty()) {
    s.average_forward_size = double(s.forward_entries) / labels.size();
    s.average_backward_size = double(s.backward_entries) / labels.size();
  }
  s.payload_bytes =
      (s.forward_entries + s.backward_entries) * sizeof(LabelEntry);
  return s;
}
}  // namespace rxl

#ifndef RXL_STATISTICS_H
#define RXL_STATISTICS_H
#include "graph.h"
#include "hub_label.h"
#include <cstddef>
#include <cstdint>
namespace rxl {
struct GraphStatistics {
  std::size_t vertices=0, arcs=0, isolated_vertices=0;
  std::size_t min_out_degree=0, max_out_degree=0;
  double average_out_degree=0.0;
};
struct LabelStatistics {
  std::uint64_t forward_entries=0, backward_entries=0;
  std::size_t max_forward_size=0, max_backward_size=0;
  double average_forward_size=0.0, average_backward_size=0.0;
  std::uint64_t payload_bytes=0;
};
GraphStatistics compute_graph_statistics(const Graph &);
LabelStatistics compute_label_statistics(const HubLabels &);
} // namespace rxl
#endif

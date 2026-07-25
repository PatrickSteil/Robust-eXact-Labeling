#include "query_support.h"

#include <algorithm>
#include <iostream>
namespace rxl {
Distance QuerySupport::distance(const HubLabels& labels, VertexId source,
                                VertexId target) {
  if (source >= labels.size() || target >= labels.size()) return kInfinity;
  const auto& out = labels[source].forward;
  const auto& in = labels[target].backward;
  Distance answer = kInfinity;
  // Both labels are DeltaLabel, which only supports a forward, ascending
  // sweep (decoding entry i costs O(i) in isolation, since hub ids are
  // stored as gaps from the previous one -- see delta_label.h). That is
  // exactly what this coordinated sweep needs anyway: the paper describes
  // the HL query as a mergesort-style walk over both labels (Section 2), so
  // we walk both with iterators instead of indices, advancing whichever
  // side currently has the smaller hub id.
  auto i = out.begin();
  const auto i_end = out.end();
  auto j = in.begin();
  const auto j_end = in.end();
  while (i != i_end && j != j_end) {
    const auto [hub_i, dist_i] = *i;
    const auto [hub_j, dist_j] = *j;
    if (hub_i < hub_j) {
      ++i;
    } else if (hub_j < hub_i) {
      ++j;
    } else {
      const std::uint64_t candidate = std::uint64_t(dist_i) + dist_j;
      if (candidate < answer) answer = static_cast<Distance>(candidate);
      ++i;
      ++j;
    }
  }
  return answer;
}
void QuerySupport::print(const HubLabels& labels) {
  for (VertexId u = 0; u < labels.size(); ++u) {
    std::cout << "Labels for " << u << ":\n  OUT: ";
    for (const auto& [h, d] : labels[u].forward)
      std::cout << '(' << h << ',' << d << ") ";
    std::cout << "\n  IN:  ";
    for (const auto& [h, d] : labels[u].backward)
      std::cout << '(' << h << ',' << d << ") ";
    std::cout << '\n';
  }
}
}  // namespace rxl

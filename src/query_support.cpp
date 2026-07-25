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
  std::size_t i = 0, j = 0;
  while (i < out.size() && j < in.size()) {
    if (out[i].first < in[j].first)
      ++i;
    else if (in[j].first < out[i].first)
      ++j;
    else {
      const std::uint64_t candidate =
          std::uint64_t(out[i].second) + in[j].second;
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

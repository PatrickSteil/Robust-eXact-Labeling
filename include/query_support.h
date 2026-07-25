#ifndef QUERY_SUPPORT_H
#define QUERY_SUPPORT_H
#include "hub_label.h"
namespace rxl {
class QuerySupport {
public:
  static Distance distance(const HubLabels &labels, VertexId source,
                           VertexId target);
  static void print(const HubLabels &labels);
};
} // namespace rxl
#endif

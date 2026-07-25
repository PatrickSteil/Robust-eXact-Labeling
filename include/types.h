#ifndef RXL_TYPES_H
#define RXL_TYPES_H

#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace rxl {
using VertexId = std::uint32_t;
using Distance = std::uint32_t;
using Rank = std::uint32_t;
using Score = std::uint64_t;

constexpr VertexId kInvalidVertex = std::numeric_limits<VertexId>::max();
constexpr Distance kInfinity = std::numeric_limits<Distance>::max();

using Edge = std::pair<VertexId, Distance>;
using AdjacencyList = std::vector<std::vector<Edge>>;
using LabelEntry = std::pair<VertexId, Distance>;
using Label = std::vector<LabelEntry>;
} // namespace rxl

#endif

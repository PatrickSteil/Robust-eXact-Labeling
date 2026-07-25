#include "index_io.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
namespace rxl {
namespace {
void write_u32(std::ostream& out, std::uint32_t value) {
  const std::array<char, 4> b{
      {char(value), char(value >> 8), char(value >> 16), char(value >> 24)}};
  out.write(b.data(), b.size());
}
std::uint32_t read_u32(std::istream& in) {
  std::array<unsigned char, 4> b{};
  in.read(reinterpret_cast<char*>(b.data()), 4);
  if (!in) throw std::runtime_error("Truncated RXL index");
  return std::uint32_t(b[0]) | std::uint32_t(b[1]) << 8 |
         std::uint32_t(b[2]) << 16 | std::uint32_t(b[3]) << 24;
}
void write_u8(std::ostream& out, std::uint8_t value) {
  out.write(reinterpret_cast<const char*>(&value), 1);
}
std::uint8_t read_u8(std::istream& in) {
  unsigned char b = 0;
  in.read(reinterpret_cast<char*>(&b), 1);
  if (!in) throw std::runtime_error("Truncated RXL index");
  return b;
}
// Writes/reads a single distance using `width` bytes (1, 2, or 4), the
// smallest width that fits every distance in the whole index (see
// choose_distance_width below).
void write_distance(std::ostream& out, Distance value, int width) {
  if (width == 1)
    write_u8(out, static_cast<std::uint8_t>(value));
  else if (width == 2) {
    const std::uint16_t v = static_cast<std::uint16_t>(value);
    const std::array<char, 2> b{{char(v), char(v >> 8)}};
    out.write(b.data(), b.size());
  } else
    write_u32(out, value);
}
Distance read_distance(std::istream& in, int width) {
  if (width == 1) return read_u8(in);
  if (width == 2) {
    std::array<unsigned char, 2> b{};
    in.read(reinterpret_cast<char*>(b.data()), 2);
    if (!in) throw std::runtime_error("Truncated RXL index");
    return std::uint32_t(b[0]) | std::uint32_t(b[1]) << 8;
  }
  return read_u32(in);
}
// The smallest of {1,2,4} bytes that can represent every distance stored
// anywhere in the labeling. Chosen once, globally, and recorded in the file
// header, per the paper's basic compression scheme (Section 4.1): "we
// represent distances with as few bits (8, 16, or 32) as needed for the
// largest distance stored in any label."
int choose_distance_width(const HubLabels& labels) {
  Distance max_distance = 0;
  for (const auto& entry : labels) {
    for (const auto& [hub, d] : entry.forward)
      max_distance = std::max(max_distance, d);
    for (const auto& [hub, d] : entry.backward)
      max_distance = std::max(max_distance, d);
  }
  if (max_distance < (1u << 8)) return 1;
  if (max_distance < (1u << 16)) return 2;
  return 4;
}
// Delta representation (Section 4.1): a label's hub ids h_1 < h_2 < ... are
// stored as gaps from the previous one, delta_i = h_i - h_{i-1} - 1 (with an
// implicit h_0 = -1, so delta_1 == h_1). Gaps are small whenever hub ids
// cluster densely -- which, since hub ids are now vertex *ranks* assigned at
// insertion time (see hub_label.h), is exactly the common case for
// frequently-used, important hubs. To keep decoding branch-free, deltas are
// split into (at most) two fixed-width blocks instead of a fully
// variable-length coding: a 1-byte-per-entry prefix for as long as deltas
// fit in a byte, then a 4-byte-per-entry suffix for the remainder.
void write_label(std::ostream& out, const Label& label, int distance_width) {
  if (label.size() > std::numeric_limits<std::uint32_t>::max())
    throw std::overflow_error("Label too large to export");
  const auto count = static_cast<std::uint32_t>(label.size());
  write_u32(out, count);
  std::vector<std::uint64_t> deltas(count);
  std::uint64_t previous_plus_one =
      0;  // h_{i-1} + 1; starts at 0 since h_0 = -1.
  for (std::uint32_t i = 0; i < count; ++i) {
    const VertexId hub = label[i].first;
    if (std::uint64_t(hub) < previous_plus_one)
      throw std::invalid_argument(
          "Label hub ids must be strictly increasing to delta-encode");
    deltas[i] = std::uint64_t(hub) - previous_plus_one;
    previous_plus_one = std::uint64_t(hub) + 1;
  }
  std::uint32_t narrow_count = 0;
  while (narrow_count < count && deltas[narrow_count] <= 0xFFu) ++narrow_count;
  write_u32(out, narrow_count);
  for (std::uint32_t i = 0; i < narrow_count; ++i)
    write_u8(out, static_cast<std::uint8_t>(deltas[i]));
  for (std::uint32_t i = narrow_count; i < count; ++i) {
    if (deltas[i] > std::numeric_limits<std::uint32_t>::max())
      throw std::overflow_error("Hub gap too large to export");
    write_u32(out, static_cast<std::uint32_t>(deltas[i]));
  }
  // Hubs first, then distances (in the same order): distances are only
  // needed once a hub id has already matched during a query, so grouping
  // them this way keeps the common (non-matching) scan cache-friendlier.
  for (const auto& [hub, distance] : label)
    write_distance(out, distance, distance_width);
}
Label read_label(std::istream& in, std::size_t n, int distance_width) {
  const auto count = read_u32(in);
  const auto narrow_count = read_u32(in);
  if (narrow_count > count)
    throw std::runtime_error("Invalid label in RXL index");
  std::vector<std::uint64_t> deltas(count);
  for (std::uint32_t i = 0; i < narrow_count; ++i) deltas[i] = read_u8(in);
  for (std::uint32_t i = narrow_count; i < count; ++i) deltas[i] = read_u32(in);
  std::vector<VertexId> hubs(count);
  std::uint64_t previous_plus_one = 0;
  VertexId previous = 0;
  bool first = true;
  for (std::uint32_t i = 0; i < count; ++i) {
    const std::uint64_t hub = previous_plus_one + deltas[i];
    if (hub >= n || (!first && hub <= previous))
      throw std::runtime_error("Invalid label in RXL index");
    hubs[i] = static_cast<VertexId>(hub);
    previous_plus_one = hub + 1;
    previous = hubs[i];
    first = false;
  }
  Label label;
  label.reserve(count);
  for (std::uint32_t i = 0; i < count; ++i) {
    const Distance distance = read_distance(in, distance_width);
    if (distance == kInfinity)
      throw std::runtime_error("Invalid label in RXL index");
    label.emplace_back(hubs[i], distance);
  }
  return label;
}
void validate_permutation(const std::vector<VertexId>& p) {
  std::vector<std::uint8_t> seen(p.size(), 0);
  for (auto v : p) {
    if (v >= p.size() || seen[v])
      throw std::runtime_error("Invalid rank permutation");
    seen[v] = 1;
  }
}
}  // namespace
void IndexIO::export_binary(const LabelingResult& result,
                            const std::string& path) {
  const std::size_t n = result.labels.size();
  if (n > std::numeric_limits<std::uint32_t>::max() ||
      result.rank_to_vertex.size() != n || result.vertex_to_rank.size() != n)
    throw std::invalid_argument("Incomplete or oversized labeling result");
  validate_permutation(result.rank_to_vertex);
  const int distance_width = choose_distance_width(result.labels);
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("Cannot open export file: " + path);
  out.write("RXLIDX\0\1", 8);
  write_u32(out, 2);
  write_u32(out, static_cast<std::uint32_t>(n));
  write_u32(out, result.rank_reordered ? 1u : 0u);
  write_u8(out, static_cast<std::uint8_t>(distance_width));
  for (auto v : result.rank_to_vertex) write_u32(out, v);
  for (auto v : result.vertex_to_rank) write_u32(out, v);
  for (const auto& entry : result.labels) {
    write_label(out, entry.forward, distance_width);
    write_label(out, entry.backward, distance_width);
  }
  if (!out) throw std::runtime_error("Failed while writing RXL index");
}
LabelingResult IndexIO::import_binary(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("Cannot open index: " + path);
  std::array<char, 8> magic{};
  in.read(magic.data(), 8);
  if (!in || std::string(magic.data(), 8) != std::string("RXLIDX\0\1", 8))
    throw std::runtime_error("Not an RXL index");
  if (read_u32(in) != 2)
    throw std::runtime_error("Unsupported RXL index version");
  const std::size_t n = read_u32(in);
  LabelingResult result;
  result.labels.resize(n);
  const std::uint32_t flags = read_u32(in);
  if (flags & ~1u) throw std::runtime_error("Unsupported RXL index flags");
  result.rank_reordered = (flags & 1u) != 0;
  const std::uint8_t width_byte = read_u8(in);
  if (width_byte != 1 && width_byte != 2 && width_byte != 4)
    throw std::runtime_error("Invalid distance width in RXL index");
  const int distance_width = width_byte;
  result.rank_to_vertex.resize(n);
  result.vertex_to_rank.resize(n);
  for (auto& v : result.rank_to_vertex) v = read_u32(in);
  for (auto& v : result.vertex_to_rank) v = read_u32(in);
  validate_permutation(result.rank_to_vertex);
  validate_permutation(result.vertex_to_rank);
  for (VertexId rank = 0; rank < n; ++rank)
    if (result.vertex_to_rank[result.rank_to_vertex[rank]] != rank)
      throw std::runtime_error("Rank maps are not inverses");
  for (auto& entry : result.labels) {
    entry.forward = read_label(in, n, distance_width);
    entry.backward = read_label(in, n, distance_width);
  }
  return result;
}
}  // namespace rxl

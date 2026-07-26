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
// Unsigned LEB128 varint: value is split into 7-bit groups, least
// significant group first, one group per byte. Every byte but the last has
// its top bit (0x80) set -- "one more byte follows" -- so decoding just
// keeps reading while that bit is set. A 32-bit value never needs more than
// 5 bytes (5*7 = 35 >= 32).
void write_varint(std::ostream& out, std::uint32_t value) {
  do {
    std::uint8_t byte = static_cast<std::uint8_t>(value & 0x7Fu);
    value >>= 7;
    if (value != 0) byte |= 0x80u;
    write_u8(out, byte);
  } while (value != 0);
}
std::uint32_t read_varint(std::istream& in) {
  std::uint32_t result = 0;
  for (int i = 0; i < 5; ++i) {
    const std::uint8_t byte = read_u8(in);
    const std::uint32_t payload = byte & 0x7Fu;
    // The 5th byte only has 4 spare bits of room (4*7 = 28, +4 = 32); a
    // well-formed encoder never sets more, so a wider payload here means
    // the file is corrupt rather than describing a value beyond uint32_t.
    if (i == 4 && payload > 0xFu)
      throw std::runtime_error("Malformed varint in RXL index (overflow)");
    result |= payload << (7 * i);
    if ((byte & 0x80u) == 0) return result;
  }
  throw std::runtime_error("Malformed varint in RXL index (unterminated)");
}
// The smallest of {1,2,4} bytes that can represent every distance stored
// anywhere in the labeling. Chosen once, globally, and recorded in the file
// header, per the paper's basic compression scheme (Section 4.1): "we
// represent distances with as few bits (8, 16, or 32) as needed for the
// largest distance stored in any label." Walking each label's public
// iterator (rather than its raw run array) is deliberate: the max distance
// is the same either way, and going through the iterator means this
// function doesn't need to know anything about the run-length
// representation at all.
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
// Label (== DeltaLabel, see delta_label.h) already *is* the on-disk
// representation in memory: hub ids as gaps from the previous hub id, and
// distances as (value, run length) pairs, both built up incrementally as
// hubs are added during PL/SamPG rather than derived at export time. So
// exporting is just serializing label.raw_deltas()/raw_run_values()/
// raw_run_lengths() in whichever byte scheme was requested -- no
// expand-to-flat-then-recompress step.
void write_label(std::ostream& out, const Label& label, LabelEncoding encoding,
                 int distance_width) {
  if (label.size() > std::numeric_limits<std::uint32_t>::max())
    throw std::overflow_error("Label too large to export");
  const auto count = static_cast<std::uint32_t>(label.size());
  write_u32(out, count);
  const std::vector<VertexId>& deltas = label.raw_deltas();
  // Hub-id deltas: two-block or varint, exactly as before -- distances
  // moving to a run-length scheme doesn't change how hub ids are packed.
  if (encoding == LabelEncoding::Varint) {
    for (std::uint32_t i = 0; i < count; ++i) write_varint(out, deltas[i]);
  } else {
    // TwoBlockDelta: split into (at most) two fixed-width blocks instead of
    // a fully variable-length coding, to keep decoding branch-free -- a
    // 1-byte-per-entry prefix for as long as deltas fit in a byte, then a
    // 4-byte-per-entry suffix for the remainder.
    std::uint32_t narrow_count = 0;
    while (narrow_count < count && deltas[narrow_count] <= 0xFFu)
      ++narrow_count;
    write_u32(out, narrow_count);
    for (std::uint32_t i = 0; i < narrow_count; ++i)
      write_u8(out, static_cast<std::uint8_t>(deltas[i]));
    for (std::uint32_t i = narrow_count; i < count; ++i)
      write_u32(out, static_cast<std::uint32_t>(deltas[i]));
  }
  // Distances: run-length encoded. Each run contributes one value (packed
  // the same way a single distance always was: fixed-width for
  // TwoBlockDelta, varint for Varint) plus a varint run length -- lengths
  // are small and heavily skewed toward 1, so they're not worth a fixed
  // width of their own regardless of which scheme the values use.
  const std::vector<Distance>& run_values = label.raw_run_values();
  const std::vector<std::uint32_t>& run_lengths = label.raw_run_lengths();
  const auto run_count = static_cast<std::uint32_t>(run_values.size());
  write_u32(out, run_count);
  for (std::uint32_t i = 0; i < run_count; ++i) {
    if (encoding == LabelEncoding::Varint)
      write_varint(out, static_cast<std::uint32_t>(run_values[i]));
    else
      write_distance(out, run_values[i], distance_width);
    write_varint(out, run_lengths[i]);
  }
}
Label read_label(std::istream& in, std::size_t n, LabelEncoding encoding,
                 int distance_width) {
  const auto count = read_u32(in);
  // A label's hub ids are unique and strictly increasing in [0, n), so it
  // can never legitimately hold more than n entries. Checking this before
  // allocating rejects a corrupt or truncated count with a clear error
  // instead of first attempting a (possibly huge, since count comes
  // straight from the file) allocation for `deltas`.
  if (count > n) throw std::runtime_error("Invalid label in RXL index");
  std::vector<VertexId> deltas(count);
  if (encoding == LabelEncoding::Varint) {
    for (std::uint32_t i = 0; i < count; ++i) deltas[i] = read_varint(in);
  } else {
    const auto narrow_count = read_u32(in);
    if (narrow_count > count)
      throw std::runtime_error("Invalid label in RXL index");
    for (std::uint32_t i = 0; i < narrow_count; ++i) deltas[i] = read_u8(in);
    for (std::uint32_t i = narrow_count; i < count; ++i)
      deltas[i] = read_u32(in);
  }
  // A run can never legitimately outnumber the entries it covers (each run
  // is at least length 1), so run_count > count is already corrupt --
  // reject it before allocating, same reasoning as the `count > n` check
  // above.
  const auto run_count = read_u32(in);
  if (run_count > count) throw std::runtime_error("Invalid label in RXL index");
  std::vector<Distance> run_values(run_count);
  std::vector<std::uint32_t> run_lengths(run_count);
  std::uint64_t total = 0;
  for (std::uint32_t i = 0; i < run_count; ++i) {
    run_values[i] = encoding == LabelEncoding::Varint
                       ? read_varint(in)
                       : read_distance(in, distance_width);
    if (run_values[i] == kInfinity)
      throw std::runtime_error("Invalid label in RXL index");
    run_lengths[i] = read_varint(in);
    if (run_lengths[i] == 0)
      throw std::runtime_error("Invalid label in RXL index");
    total += run_lengths[i];
  }
  if (total != count) throw std::runtime_error("Invalid label in RXL index");
  // DeltaLabel::from_runs re-derives and validates the hub ids (in range,
  // strictly increasing) while adopting the deltas/runs as-is -- no
  // separate decode-then-reencode step.
  try {
    return Label::from_runs(std::move(deltas), std::move(run_values),
                            std::move(run_lengths), n);
  } catch (const std::runtime_error&) {
    throw std::runtime_error("Invalid label in RXL index");
  } catch (const std::invalid_argument&) {
    throw std::runtime_error("Invalid label in RXL index");
  }
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
                            const std::string& path, LabelEncoding encoding) {
  const std::size_t n = result.labels.size();
  if (n > std::numeric_limits<std::uint32_t>::max() ||
      result.rank_to_vertex.size() != n || result.vertex_to_rank.size() != n)
    throw std::invalid_argument("Incomplete or oversized labeling result");
  validate_permutation(result.rank_to_vertex);
  const bool varint = encoding == LabelEncoding::Varint;
  // distance_width is meaningless for Varint (every distance is
  // self-delimiting) and simply isn't written for it; only computed here
  // for TwoBlockDelta.
  const int distance_width = varint ? 0 : choose_distance_width(result.labels);
  std::ofstream out(path, std::ios::binary);
  if (!out) throw std::runtime_error("Cannot open export file: " + path);
  out.write("RXLIDX\0\1", 8);
  // Version 3: distances are run-length encoded (delta_label.h); versions
  // 1-2's flat per-entry distance layout is no longer written or read.
  write_u32(out, 3);
  write_u32(out, static_cast<std::uint32_t>(n));
  // Bit 0: rank_reordered. Bit 1: varint hub-id/run-value encoding (vs.
  // TwoBlockDelta). Both flags are scoped to a single version-3 file; the
  // version bump above is what actually separates this format (run-length
  // encoded distances) from the older flat per-entry one, not these bits.
  write_u32(out, (result.rank_reordered ? 1u : 0u) | (varint ? 2u : 0u));
  if (!varint) write_u8(out, static_cast<std::uint8_t>(distance_width));
  for (auto v : result.rank_to_vertex) write_u32(out, v);
  for (auto v : result.vertex_to_rank) write_u32(out, v);
  for (const auto& entry : result.labels) {
    write_label(out, entry.forward, encoding, distance_width);
    write_label(out, entry.backward, encoding, distance_width);
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
  if (read_u32(in) != 3)
    throw std::runtime_error("Unsupported RXL index version");
  const std::size_t n = read_u32(in);
  LabelingResult result;
  result.labels.resize(n);
  const std::uint32_t flags = read_u32(in);
  if (flags & ~3u) throw std::runtime_error("Unsupported RXL index flags");
  result.rank_reordered = (flags & 1u) != 0;
  const LabelEncoding encoding =
      (flags & 2u) ? LabelEncoding::Varint : LabelEncoding::TwoBlockDelta;
  int distance_width = 0;  // unused (and never read) when encoding == Varint
  if (encoding == LabelEncoding::TwoBlockDelta) {
    const std::uint8_t width_byte = read_u8(in);
    if (width_byte != 1 && width_byte != 2 && width_byte != 4)
      throw std::runtime_error("Invalid distance width in RXL index");
    distance_width = width_byte;
  }
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
    entry.forward = read_label(in, n, encoding, distance_width);
    entry.backward = read_label(in, n, encoding, distance_width);
  }
  return result;
}
}  // namespace rxl

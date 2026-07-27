#ifndef RXL_INDEXED_MINHEAP
#define RXL_INDEXED_MINHEAP

#include <cassert>
#include <functional>
#include <limits>
#include <vector>

#include "types.h"

namespace rxl {
namespace dijkstra_detail {
struct NoOpOnPop {
  void operator()(VertexId, Distance) const {}
};
struct NeverPrune {
  bool operator()(VertexId, Distance) const { return false; }
};
struct NoOpOnSettle {
  void operator()(VertexId, Distance) const {}
};
struct NoOpOnRelax {
  void operator()(VertexId, VertexId, Distance) const {}
};
struct NoOpOnNonImprovingEdge {
  void operator()(VertexId, VertexId, std::uint64_t) const {}
};

static_assert(sizeof(Distance) == 4 && sizeof(VertexId) == 4,
              "packed heap item assumes 32-bit Distance/VertexId");
inline std::uint64_t pack(Distance d, VertexId v) {
  return (std::uint64_t(d) << 32) | std::uint64_t(v);
}

class IndexedMinHeap {
public:
  IndexedMinHeap() = default;
  explicit IndexedMinHeap(std::size_t n) { assign(n); }

  void assign(std::size_t n) {
    heap_.clear();
    heap_.reserve(n);
    position_.assign(n, kInvalidPos);
  }

  std::size_t capacity() const { return position_.size(); }
  bool empty() const { return heap_.empty(); }

  void decrease_key(VertexId v, Distance d) {
    const std::uint64_t item = pack(d, v);
    if (position_[v] == kInvalidPos) {
      const std::size_t i = heap_.size();
      heap_.push_back(item);
      position_[v] = i;
      sift_up(i);
    } else {
      heap_[position_[v]] = item;
      sift_up(position_[v]);
    }
  }

  std::pair<Distance, VertexId> pop_min() {
    const std::uint64_t top = heap_[0];
    position_[static_cast<VertexId>(top)] = kInvalidPos;
    const std::size_t last = heap_.size() - 1;
    if (last != 0) {
      heap_[0] = heap_[last];
      position_[static_cast<VertexId>(heap_[0])] = 0;
    }
    heap_.pop_back();
    if (!heap_.empty())
      sift_down(0);
    return {static_cast<Distance>(top >> 32), static_cast<VertexId>(top)};
  }

private:
  static constexpr std::size_t kInvalidPos =
      std::numeric_limits<std::size_t>::max();
  static constexpr std::size_t kArity = 4;

  void sift_up(std::size_t i) {
    const std::uint64_t val = heap_[i];
    while (i > 0) {
      const std::size_t parent = (i - 1) / kArity;
      if (heap_[parent] <= val)
        break;
      heap_[i] = heap_[parent];
      position_[static_cast<VertexId>(heap_[i])] = i;
      i = parent;
    }
    heap_[i] = val;
    position_[static_cast<VertexId>(val)] = i;
  }

  void sift_down(std::size_t i) {
    const std::size_t n = heap_.size();
    const std::uint64_t val = heap_[i];
    while (true) {
      const std::size_t first_child = kArity * i + 1;
      std::size_t smallest = i;
      std::uint64_t best = val;
      const std::size_t last_child = std::min(first_child + kArity, n);
      for (std::size_t c = first_child; c < last_child; ++c) {
        if (heap_[c] < best) {
          smallest = c;
          best = heap_[c];
        }
      }
      if (smallest == i)
        break;
      heap_[i] = heap_[smallest];
      position_[static_cast<VertexId>(heap_[i])] = i;
      i = smallest;
    }
    heap_[i] = val;
    position_[static_cast<VertexId>(val)] = i;
  }

  std::vector<std::uint64_t> heap_;
  std::vector<std::size_t> position_;
};
} // namespace dijkstra_detail
} // namespace rxl
#endif

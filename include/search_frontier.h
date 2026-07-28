#ifndef RXL_SEARCH_FRONTIER_H
#define RXL_SEARCH_FRONTIER_H
#include <cstddef>
#include <deque>
#include <utility>

#include "indexed_minheap.h"
#include "types.h"

namespace rxl {

class DijkstraFrontier {
 public:
  DijkstraFrontier() = default;
  explicit DijkstraFrontier(std::size_t n) : heap_(n) {}

  void assign(std::size_t n) { heap_.assign(n); }
  bool empty() const { return heap_.empty(); }

  void relax(VertexId v, Distance new_distance, Distance /*edge_weight*/) {
    heap_.decrease_key(v, new_distance);
  }
  std::pair<Distance, VertexId> pop() { return heap_.pop_min(); }

 private:
  dijkstra_detail::IndexedMinHeap heap_;
};

class ZeroOneBfsFrontier {
 public:
  ZeroOneBfsFrontier() = default;
  explicit ZeroOneBfsFrontier(std::size_t /*n*/) {}

  void assign(std::size_t /*n*/) { queue_.clear(); }
  bool empty() const { return queue_.empty(); }

  void relax(VertexId v, Distance new_distance, Distance edge_weight) {
    const std::uint64_t item = dijkstra_detail::pack(new_distance, v);
    if (edge_weight == 0)
      queue_.push_front(item);
    else
      queue_.push_back(item);
  }
  std::pair<Distance, VertexId> pop() {
    const std::uint64_t item = queue_.front();
    queue_.pop_front();
    return {static_cast<Distance>(item >> 32), static_cast<VertexId>(item)};
  }

 private:
  std::deque<std::uint64_t> queue_;
};

class BfsFrontier {
 public:
  BfsFrontier() = default;
  explicit BfsFrontier(std::size_t /*n*/) {}

  void assign(std::size_t /*n*/) { queue_.clear(); }
  bool empty() const { return queue_.empty(); }

  void relax(VertexId v, Distance new_distance, Distance /*edge_weight*/) {
    queue_.push_back(dijkstra_detail::pack(new_distance, v));
  }
  std::pair<Distance, VertexId> pop() {
    const std::uint64_t item = queue_.front();
    queue_.pop_front();
    return {static_cast<Distance>(item >> 32), static_cast<VertexId>(item)};
  }

 private:
  std::deque<std::uint64_t> queue_;
};

}  // namespace rxl
#endif

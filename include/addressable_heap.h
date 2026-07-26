#ifndef RXL_ADDRESSABLE_HEAP_H
#define RXL_ADDRESSABLE_HEAP_H
#include <cstddef>
#include <limits>
#include <vector>

#include "types.h"

namespace rxl {

struct HeapEntry {
  Score robust;
  Score total;
  std::size_t degree;
  VertexId v;
};

class AddressableHeap {
 public:
  explicit AddressableHeap(std::size_t n);

  bool empty() const { return heap_.empty(); }
  bool contains(VertexId v) const { return position_[v] != kInvalidPos; }

  void set(VertexId v, Score robust, Score total, std::size_t degree);

  VertexId pop_top();

 private:
  static constexpr std::size_t kInvalidPos =
      std::numeric_limits<std::size_t>::max();

  static bool less(const HeapEntry& a, const HeapEntry& b);
  void swap_at(std::size_t i, std::size_t j);
  bool sift_up(std::size_t i);
  void sift_down(std::size_t i);
  void remove_at(std::size_t i);

  std::vector<HeapEntry> heap_;
  std::vector<std::size_t> position_;
};

}  // namespace rxl
#endif

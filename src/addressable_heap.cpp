#include "addressable_heap.h"

#include <utility>

namespace rxl {

AddressableHeap::AddressableHeap(std::size_t n) : position_(n, kInvalidPos) {
  heap_.reserve(n);
}

bool AddressableHeap::less(const HeapEntry& a, const HeapEntry& b) {
  if (a.robust != b.robust) return a.robust < b.robust;
  if (a.total != b.total) return a.total < b.total;
  if (a.degree != b.degree) return a.degree < b.degree;
  return a.v > b.v;  // smaller vertex id wins ties, so it must sort higher.
}

void AddressableHeap::swap_at(std::size_t i, std::size_t j) {
  std::swap(heap_[i], heap_[j]);
  position_[heap_[i].v] = i;
  position_[heap_[j].v] = j;
}

bool AddressableHeap::sift_up(std::size_t i) {
  bool moved = false;
  while (i > 0) {
    const std::size_t parent = (i - 1) / 2;
    if (!less(heap_[parent], heap_[i])) break;
    swap_at(i, parent);
    i = parent;
    moved = true;
  }
  return moved;
}

void AddressableHeap::sift_down(std::size_t i) {
  const std::size_t size = heap_.size();
  while (true) {
    std::size_t largest = i;
    const std::size_t l = 2 * i + 1, r = 2 * i + 2;
    if (l < size && less(heap_[largest], heap_[l])) largest = l;
    if (r < size && less(heap_[largest], heap_[r])) largest = r;
    if (largest == i) break;
    swap_at(i, largest);
    i = largest;
  }
}

void AddressableHeap::set(VertexId v, Score robust, Score total,
                          std::size_t degree) {
  if (position_[v] == kInvalidPos) {
    const std::size_t i = heap_.size();
    heap_.push_back(HeapEntry{robust, total, degree, v});
    position_[v] = i;
    sift_up(i);
  } else {
    const std::size_t i = position_[v];
    heap_[i] = HeapEntry{robust, total, degree, v};
    if (!sift_up(i)) sift_down(i);
  }
}

void AddressableHeap::remove_at(std::size_t i) {
  const VertexId removed_v = heap_[i].v;
  position_[removed_v] = kInvalidPos;
  const std::size_t last = heap_.size() - 1;
  if (i != last) {
    heap_[i] = heap_[last];
    position_[heap_[i].v] = i;
    heap_.pop_back();
    if (!sift_up(i)) sift_down(i);
  } else {
    heap_.pop_back();
  }
}

VertexId AddressableHeap::pop_top() {
  const VertexId top = heap_[0].v;
  remove_at(0);
  return top;
}

}  // namespace rxl

#ifndef RXL_DELTA_LABEL_H
#define RXL_DELTA_LABEL_H
#include <cstdint>
#include <stdexcept>
#include <utility>
#include <vector>

#include "types.h"

namespace rxl {

class DeltaLabel {
 public:
  using Entry = std::pair<VertexId, Distance>;

  class const_iterator {
   public:
    using iterator_category = std::input_iterator_tag;
    using value_type = Entry;
    using difference_type = std::ptrdiff_t;
    using pointer = const Entry*;
    using reference = Entry;

    reference operator*() const { return {current_hub_, (*label_)[index_]}; }

    const_iterator& operator++() {
      ++index_;
      if (index_ < label_->deltas_.size())
        current_hub_ =
            static_cast<VertexId>(current_hub_ + 1 + label_->deltas_[index_]);
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator tmp = *this;
      ++(*this);
      return tmp;
    }
    bool operator==(const const_iterator& other) const {
      return label_ == other.label_ && index_ == other.index_;
    }
    bool operator!=(const const_iterator& other) const {
      return !(*this == other);
    }

   private:
    friend class DeltaLabel;
    const_iterator(const DeltaLabel* label, std::size_t index,
                   VertexId current_hub)
        : label_(label), index_(index), current_hub_(current_hub) {}
    const DeltaLabel* label_;
    std::size_t index_;
    VertexId current_hub_;
  };
  using iterator = const_iterator;

  DeltaLabel() = default;

  void push_back(VertexId hub, Distance distance) {
    if (hub < next_min_hub_)
      throw std::invalid_argument(
          "DeltaLabel: hub ids must be inserted in strictly increasing "
          "order");
    deltas_.push_back(hub - next_min_hub_);
    distances_.push_back(distance);
    next_min_hub_ = hub + 1;
  }

  void reserve(std::size_t n) {
    deltas_.reserve(n);
    distances_.reserve(n);
  }
  std::size_t size() const { return deltas_.size(); }
  bool empty() const { return deltas_.empty(); }

  const_iterator begin() const {
    return const_iterator(this, 0, deltas_.empty() ? 0 : deltas_[0]);
  }
  const_iterator end() const { return const_iterator(this, deltas_.size(), 0); }

  const std::vector<VertexId>& raw_deltas() const { return deltas_; }
  const std::vector<Distance>& raw_distances() const { return distances_; }

  static DeltaLabel from_deltas(std::vector<VertexId> deltas,
                                std::vector<Distance> distances,
                                std::size_t n) {
    if (deltas.size() != distances.size())
      throw std::invalid_argument(
          "DeltaLabel::from_deltas: mismatched array sizes");
    DeltaLabel label;
    std::uint64_t next_min = 0;
    for (std::size_t i = 0; i < deltas.size(); ++i) {
      const std::uint64_t hub = next_min + deltas[i];
      if (hub >= n)
        throw std::runtime_error(
            "DeltaLabel::from_deltas: hub id out of range");
      next_min = hub + 1;
    }
    label.deltas_ = std::move(deltas);
    label.distances_ = std::move(distances);
    label.next_min_hub_ = static_cast<VertexId>(next_min);
    return label;
  }

 private:
  Distance operator[](std::size_t index) const { return distances_[index]; }

  std::vector<VertexId> deltas_;
  std::vector<Distance> distances_;
  VertexId next_min_hub_ = 0;
};

using LabelEntry = DeltaLabel::Entry;
using Label = DeltaLabel;

}  // namespace rxl
#endif

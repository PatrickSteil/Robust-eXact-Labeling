#ifndef RXL_DELTA_LABEL_H
#define RXL_DELTA_LABEL_H
#include <cstdint>
#include <limits>
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

    reference operator*() const { return {current_hub_, current_distance_}; }

    const_iterator& operator++() {
      ++index_;
      if (index_ < label_->deltas_.size()) {
        current_hub_ =
            static_cast<VertexId>(current_hub_ + 1 + label_->deltas_[index_]);
        if (--run_remaining_ == 0) {
          ++run_index_;
          current_distance_ = label_->run_values_[run_index_];
          run_remaining_ = label_->run_lengths_[run_index_];
        }
      }
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
        : label_(label), index_(index), current_hub_(current_hub) {
      if (index_ < label_->deltas_.size()) {
        run_index_ = 0;
        current_distance_ = label_->run_values_[0];
        run_remaining_ = label_->run_lengths_[0];
      } else {
        run_index_ = 0;
        current_distance_ = 0;
        run_remaining_ = 0;
      }
    }
    const DeltaLabel* label_;
    std::size_t index_;
    VertexId current_hub_;
    std::size_t run_index_;
    Distance current_distance_;
    std::uint32_t run_remaining_;
  };
  using iterator = const_iterator;

  DeltaLabel() = default;

  void push_back(VertexId hub, Distance distance) {
    if (hub == kInvalidVertex)
      throw std::invalid_argument(
          "DeltaLabel: hub id must not be "
          "kInvalidVertex");
    if (hub < next_min_hub_)
      throw std::invalid_argument(
          "DeltaLabel: hub ids must be inserted in strictly increasing "
          "order");
    deltas_.push_back(hub - next_min_hub_);
    extend_or_start_run(distance);
    next_min_hub_ = hub + 1;
  }

  void reserve(std::size_t n) {
    deltas_.reserve(n);
    run_values_.reserve(n);
    run_lengths_.reserve(n);
  }
  std::size_t size() const { return deltas_.size(); }
  bool empty() const { return deltas_.empty(); }

  void prefetch() const {
#if defined(__GNUC__) || defined(__clang__)
    __builtin_prefetch(deltas_.data(), 0, 1);
    __builtin_prefetch(run_values_.data(), 0, 1);
    __builtin_prefetch(run_lengths_.data(), 0, 1);
#endif
  }

  const_iterator begin() const {
    return const_iterator(this, 0, deltas_.empty() ? 0 : deltas_[0]);
  }
  const_iterator end() const { return const_iterator(this, deltas_.size(), 0); }

  const std::vector<VertexId>& raw_deltas() const { return deltas_; }
  const std::vector<Distance>& raw_run_values() const { return run_values_; }
  const std::vector<std::uint32_t>& raw_run_lengths() const {
    return run_lengths_;
  }
  std::vector<Distance> raw_distances() const {
    std::vector<Distance> flat;
    flat.reserve(deltas_.size());
    for (std::size_t r = 0; r < run_values_.size(); ++r)
      for (std::uint32_t k = 0; k < run_lengths_[r]; ++k)
        flat.push_back(run_values_[r]);
    return flat;
  }

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
    for (const Distance d : distances) label.extend_or_start_run(d);
    label.next_min_hub_ = static_cast<VertexId>(next_min);
    return label;
  }

  static DeltaLabel from_runs(std::vector<VertexId> deltas,
                              std::vector<Distance> run_values,
                              std::vector<std::uint32_t> run_lengths,
                              std::size_t n) {
    if (run_values.size() != run_lengths.size())
      throw std::invalid_argument(
          "DeltaLabel::from_runs: mismatched run arrays");
    std::uint64_t total = 0;
    for (const std::uint32_t len : run_lengths) {
      if (len == 0)
        throw std::invalid_argument("DeltaLabel::from_runs: zero-length run");
      total += len;
    }
    if (total != deltas.size())
      throw std::invalid_argument(
          "DeltaLabel::from_runs: run lengths do not match delta count");
    DeltaLabel label;
    std::uint64_t next_min = 0;
    for (std::size_t i = 0; i < deltas.size(); ++i) {
      const std::uint64_t hub = next_min + deltas[i];
      if (hub >= n)
        throw std::runtime_error("DeltaLabel::from_runs: hub id out of range");
      next_min = hub + 1;
    }
    label.deltas_ = std::move(deltas);
    label.run_values_ = std::move(run_values);
    label.run_lengths_ = std::move(run_lengths);
    label.next_min_hub_ = static_cast<VertexId>(next_min);
    return label;
  }

 private:
  void extend_or_start_run(Distance distance) {
    if (!run_lengths_.empty() && run_values_.back() == distance &&
        run_lengths_.back() < std::numeric_limits<std::uint32_t>::max()) {
      ++run_lengths_.back();
    } else {
      run_values_.push_back(distance);
      run_lengths_.push_back(1);
    }
  }

  std::vector<VertexId> deltas_;
  std::vector<Distance> run_values_;
  std::vector<std::uint32_t> run_lengths_;
  VertexId next_min_hub_ = 0;
};

using LabelEntry = DeltaLabel::Entry;
using Label = DeltaLabel;

}  // namespace rxl
#endif

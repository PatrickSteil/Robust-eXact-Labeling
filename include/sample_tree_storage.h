#ifndef RXL_SAMPLE_TREE_STORAGE_H
#define RXL_SAMPLE_TREE_STORAGE_H
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sparsehash/sparse_hash_map>
#include <utility>
#include <vector>

#include "types.h"

namespace rxl {
class DenseBufferPool {
 public:
  struct Buffers {
    std::vector<VertexId> parent;
    std::vector<std::vector<VertexId>> children;
    std::vector<std::uint8_t> alive;
    std::vector<Score> subtree;
    std::vector<VertexId> touched;
  };

  static void clear_touched(Buffers& buf) {
    for (const VertexId v : buf.touched) {
      buf.parent[v] = kInvalidVertex;
      buf.alive[v] = 0;
      buf.subtree[v] = 0;
      buf.children[v].clear();
    }
    buf.touched.clear();
  }

  std::unique_ptr<Buffers> acquire(std::size_t n) {
    std::unique_ptr<Buffers> buf;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!free_.empty()) {
        buf = std::move(free_.back());
        free_.pop_back();
      }
    }
    if (!buf) buf = std::make_unique<Buffers>();
    if (buf->parent.size() == n) {
      clear_touched(*buf);
    } else {
      buf->parent.assign(n, kInvalidVertex);
      buf->children.assign(n, {});
      buf->alive.assign(n, 0);
      buf->subtree.assign(n, 0);
      buf->touched.clear();
    }
    return buf;
  }

  void release(std::unique_ptr<Buffers> buf) {
    std::lock_guard<std::mutex> lock(mutex_);
    free_.push_back(std::move(buf));
  }

 private:
  std::mutex mutex_;
  std::vector<std::unique_ptr<Buffers>> free_;
};

class SampleTreeStorage {
 public:
  static std::size_t dense_threshold(std::size_t n) { return n / 8; }

  SampleTreeStorage(std::size_t n, DenseBufferPool& pool)
      : n_(n), is_dense_(true), pool_(&pool), dense_(pool.acquire(n)) {}

  SampleTreeStorage(SampleTreeStorage&& other) noexcept
      : n_(other.n_),
        is_dense_(other.is_dense_),
        pool_(other.pool_),
        dense_(std::move(other.dense_)) {
    sparse_.swap(other.sparse_);
  }
  SampleTreeStorage& operator=(SampleTreeStorage&& other) noexcept {
    if (this == &other) return *this;
    n_ = other.n_;
    is_dense_ = other.is_dense_;
    pool_ = other.pool_;
    dense_ = std::move(other.dense_);
    sparse_.swap(other.sparse_);
    return *this;
  }
  SampleTreeStorage(const SampleTreeStorage&) = delete;
  SampleTreeStorage& operator=(const SampleTreeStorage&) = delete;

  bool is_dense() const { return is_dense_; }

  void reset(std::size_t n, DenseBufferPool& pool) {
    n_ = n;
    is_dense_ = true;
    pool_ = &pool;
    if (dense_ && dense_->parent.size() == n_) {
      DenseBufferPool::clear_touched(*dense_);
    } else {
      dense_ = pool.acquire(n_);
    }
    if (!sparse_.empty()) sparse_.clear();
  }

  bool alive(VertexId v) const {
    if (is_dense_) return dense_->alive[v] != 0;
    return sparse_.find(v) != sparse_.end();
  }
  VertexId parent_of(VertexId v) const {
    if (is_dense_) return dense_->parent[v];
    const auto it = sparse_.find(v);
    return it == sparse_.end() ? kInvalidVertex : it->second.parent;
  }
  Score subtree_of(VertexId v) const {
    if (is_dense_) return dense_->subtree[v];
    const auto it = sparse_.find(v);
    return it == sparse_.end() ? Score{0} : it->second.subtree;
  }
  const std::vector<VertexId>& children_of(VertexId v) const {
    if (is_dense_) return dense_->children[v];
    const auto it = sparse_.find(v);
    return it == sparse_.end() ? empty_children() : it->second.children;
  }

  void set_parent(VertexId v, VertexId parent) {
    if (is_dense_) {
      dense_->parent[v] = parent;
      dense_->touched.push_back(v);
    } else {
      sparse_[v].parent = parent;
    }
  }
  void mark_alive(VertexId v) {
    if (is_dense_) {
      dense_->alive[v] = 1;
      dense_->touched.push_back(v);
    } else {
      (void)sparse_[v];  // Ensures a node exists, parent already set above.
    }
  }
  void set_subtree(VertexId v, Score value) {
    if (is_dense_) {
      dense_->subtree[v] = value;
      dense_->touched.push_back(v);
    } else {
      sparse_[v].subtree = value;
    }
  }
  void add_subtree(VertexId v, Score delta) {
    if (is_dense_) {
      dense_->subtree[v] += delta;
      dense_->touched.push_back(v);
    } else {
      const auto it = sparse_.find(v);
      if (it != sparse_.end()) it->second.subtree += delta;
    }
  }
  void add_child(VertexId parent, VertexId child) {
    if (is_dense_) {
      dense_->children[parent].push_back(child);
      dense_->touched.push_back(parent);
    } else {
      sparse_[parent].children.push_back(child);
    }
  }
  void deactivate(VertexId v) {
    if (is_dense_)
      dense_->alive[v] = 0;
    else
      sparse_.erase(v);
  }

  bool maybe_downgrade(const std::vector<VertexId>& members,
                       std::size_t live_count) {
    if (!is_dense_ || live_count >= dense_threshold(n_)) return false;
    sparse_.set_deleted_key(kInvalidVertex);
    sparse_.resize(live_count * 2 + 1);  // avoid rehashing while filling.
    for (VertexId v : members) {
      if (!dense_->alive[v]) continue;
      Node node;
      node.parent = dense_->parent[v];
      node.subtree = dense_->subtree[v];
      node.children = std::move(dense_->children[v]);
      sparse_.insert({v, std::move(node)});
    }
    pool_->release(std::move(dense_));
    is_dense_ = false;
    return true;
  }

 private:
  struct Node {
    VertexId parent = kInvalidVertex;
    Score subtree = 0;
    std::vector<VertexId> children;
  };
  static const std::vector<VertexId>& empty_children() {
    static const std::vector<VertexId> kEmpty;
    return kEmpty;
  }

  std::size_t n_;
  bool is_dense_;
  DenseBufferPool* pool_ = nullptr;
  std::unique_ptr<DenseBufferPool::Buffers> dense_;
  google::sparse_hash_map<VertexId, Node> sparse_;
};

}  // namespace rxl
#endif

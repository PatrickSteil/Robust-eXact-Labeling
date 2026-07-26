#ifndef RXL_SAMPLE_TREE_STORAGE_H
#define RXL_SAMPLE_TREE_STORAGE_H
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <sparsehash/sparse_hash_map>
#include <utility>
#include <vector>

#include "types.h"

namespace rxl {

// Per-vertex storage for a single sampled tree (Section 3.3 / Appendix A.2).
//
// A sampled tree only ever needs, per vertex v: whether v is alive, v's
// parent, v's subtree count, and v's children. Appendix A.2 explains that
// representing this as four plain O(n)-sized arrays is fine (and fast) for
// *large* trees, but wasteful for *small* ones: with many trees live at
// once (SamplingOptions::counter_buckets and friends), n-sized arrays per
// tree dominate memory long before the trees themselves do any useful work,
// especially since trees created later in the algorithm are pruned by the
// current partial labels and are typically much smaller than n (see
// pruned_labeling.cpp's build_sample_tree). The paper's fix: keep large
// trees (>= n/8 live vertices) as dense arrays, and represent small trees
// (< n/8) as a hash table instead, so their footprint is proportional to
// their own size rather than to n.
//
// This class implements exactly that split. It always starts in dense mode
// (a tree's eventual size is only known once Dijkstra finishes growing it),
// and maybe_downgrade() converts it to the hash-map-backed ("sparse") mode
// once it has shrunk below the threshold -- which can happen either right
// after the tree is built, or later, as remove_subtree() consumes it over
// many subsequent PL/SamPG ranks. It never upgrades back to dense: trees
// only shrink over their lifetime (see pruned_labeling.cpp), so that
// direction is never needed.
//
// The sparse backend uses google::sparse_hash_map
// (https://github.com/sparsehash/sparsehash, vendored under third_party/),
// specifically chosen over dense_hash_map because it optimizes for memory
// (~2 bits of overhead per bucket) rather than lookup speed, which is the
// right trade-off here: small trees are numerous and mostly just sit there
// being decremented a little at a time by remove_subtree, so we care about
// their resting footprint, not raw lookup throughput.
class SampleTreeStorage {
 public:
  // A tree is "large" once it has at least n/8 live vertices (Appendix
  // A.2's own threshold) and "small" otherwise.
  static std::size_t dense_threshold(std::size_t n) { return n / 8; }

  explicit SampleTreeStorage(std::size_t n) : n_(n), is_dense_(true) {
    dense_parent_.assign(n_, kInvalidVertex);
    dense_children_.assign(n_, {});
    dense_alive_.assign(n_, 0);
    dense_subtree_.assign(n_, 0);
  }

  // Move-only, via swap. google::sparse_hash_map predates C++11 move
  // semantics: it only has a copy constructor/assignment (plus a swap()
  // it uses internally for resizing), so the *implicitly*-defined move
  // operations here would silently degrade to an O(size) copy of sparse_
  // for any tree already downgraded to the hash-map backend -- exactly the
  // cost this class exists to avoid, e.g. every time SampleTree objects get
  // reshuffled by std::remove_if()/erase() in the ordering loop. Defining
  // these explicitly with swap() keeps every move O(1) regardless of
  // backend, and makes the class non-copyable (fine: nothing ever needs to
  // copy a whole sampled tree, only move it around or read from it).
  SampleTreeStorage(SampleTreeStorage&& other) noexcept
      : n_(other.n_), is_dense_(other.is_dense_) {
    dense_parent_.swap(other.dense_parent_);
    dense_children_.swap(other.dense_children_);
    dense_alive_.swap(other.dense_alive_);
    dense_subtree_.swap(other.dense_subtree_);
    sparse_.swap(other.sparse_);
  }
  SampleTreeStorage& operator=(SampleTreeStorage&& other) noexcept {
    if (this == &other) return *this;
    n_ = other.n_;
    is_dense_ = other.is_dense_;
    dense_parent_.swap(other.dense_parent_);
    dense_children_.swap(other.dense_children_);
    dense_alive_.swap(other.dense_alive_);
    dense_subtree_.swap(other.dense_subtree_);
    sparse_.swap(other.sparse_);
    return *this;
  }
  SampleTreeStorage(const SampleTreeStorage&) = delete;
  SampleTreeStorage& operator=(const SampleTreeStorage&) = delete;

  bool is_dense() const { return is_dense_; }

  // Rewinds this storage to a fresh, empty, all-dense tree over the same n
  // vertices, reusing already-allocated capacity instead of releasing it
  // and reallocating (as replacing a retired SampleTree with a brand new
  // one would). If this storage was still dense, dense_parent_/
  // dense_alive_/dense_subtree_ are refilled in place via assign() (which
  // reuses the existing buffer whenever its capacity already fits n, the
  // overwhelmingly common case here since n never changes across a run),
  // and each per-vertex children_ list is cleared rather than destroyed --
  // so vertices that repeatedly end up with a handful of children across
  // many trees don't pay for that vector's growth every single time. If it
  // had already downgraded to sparse (its dense arrays freed for good, see
  // maybe_downgrade()), there is nothing dense left to reuse; it goes back
  // to plain O(n) dense arrays exactly as a newly-constructed
  // SampleTreeStorage would, and the sparse map is cleared (not
  // reallocated away) so its bucket array can be reused if this tree
  // downgrades again later.
  void reset(std::size_t n) {
    n_ = n;
    is_dense_ = true;
    if (dense_parent_.size() == n_) {
      std::fill(dense_parent_.begin(), dense_parent_.end(), kInvalidVertex);
      std::fill(dense_alive_.begin(), dense_alive_.end(), 0);
      std::fill(dense_subtree_.begin(), dense_subtree_.end(), 0);
      for (auto& children : dense_children_) children.clear();
    } else {
      dense_parent_.assign(n_, kInvalidVertex);
      dense_children_.assign(n_, {});
      dense_alive_.assign(n_, 0);
      dense_subtree_.assign(n_, 0);
    }
    // sparse_hash_map::clear() already empties the table without
    // deallocating its bucket array (see sparsehashtable.h), so a tree
    // that previously downgraded keeps that allocation available if it
    // downgrades again on its next life.
    if (!sparse_.empty()) sparse_.clear();
  }

  bool alive(VertexId v) const {
    if (is_dense_) return dense_alive_[v] != 0;
    return sparse_.find(v) != sparse_.end();
  }
  VertexId parent_of(VertexId v) const {
    if (is_dense_) return dense_parent_[v];
    const auto it = sparse_.find(v);
    return it == sparse_.end() ? kInvalidVertex : it->second.parent;
  }
  Score subtree_of(VertexId v) const {
    if (is_dense_) return dense_subtree_[v];
    const auto it = sparse_.find(v);
    return it == sparse_.end() ? Score{0} : it->second.subtree;
  }
  const std::vector<VertexId>& children_of(VertexId v) const {
    if (is_dense_) return dense_children_[v];
    const auto it = sparse_.find(v);
    return it == sparse_.end() ? empty_children() : it->second.children;
  }

  // Growth-time operations (see build_sample_tree). Dijkstra relaxes a
  // vertex's tentative parent (possibly repeatedly) before it is ever
  // settled/marked alive, so parent-setting and alive-marking are separate
  // calls -- mirroring exactly what the old plain-array code did with
  // `tree.parent[v] = u` (relaxation) vs. `tree.alive[u] = 1` (settling).
  void set_parent(VertexId v, VertexId parent) {
    if (is_dense_)
      dense_parent_[v] = parent;
    else
      sparse_[v].parent = parent;
  }
  void mark_alive(VertexId v) {
    if (is_dense_)
      dense_alive_[v] = 1;
    else
      (void)sparse_[v];  // Ensures a node exists, parent already set above.
  }
  void set_subtree(VertexId v, Score value) {
    if (is_dense_)
      dense_subtree_[v] = value;
    else
      sparse_[v].subtree = value;
  }
  void add_subtree(VertexId v, Score delta) {
    if (is_dense_) {
      dense_subtree_[v] += delta;
    } else {
      const auto it = sparse_.find(v);
      if (it != sparse_.end()) it->second.subtree += delta;
    }
  }
  void add_child(VertexId parent, VertexId child) {
    if (is_dense_)
      dense_children_[parent].push_back(child);
    else
      sparse_[parent].children.push_back(child);
  }
  void deactivate(VertexId v) {
    if (is_dense_)
      dense_alive_[v] = 0;
    else
      sparse_.erase(v);
  }

  // Converts from dense to sparse storage once `live_count` (the tree's
  // current number of alive vertices, i.e. SampleTree::remaining) has
  // dropped below dense_threshold(n). `members` is the tree's fixed
  // candidate list from construction (every vertex the search ever
  // settled), so this scan costs O(members.size()), not O(n): for trees
  // that were already pruned at birth, that is the tree's own small size;
  // for large seed trees it is a one-off cost paid exactly once, when they
  // first cross the threshold. No-op (returns false) if already sparse or
  // still large; returns true iff this call performed the conversion.
  bool maybe_downgrade(const std::vector<VertexId>& members,
                       std::size_t live_count) {
    if (!is_dense_ || live_count >= dense_threshold(n_)) return false;
    sparse_.set_deleted_key(kInvalidVertex);
    sparse_.resize(live_count * 2 + 1);  // avoid rehashing while filling.
    for (VertexId v : members) {
      if (!dense_alive_[v]) continue;
      Node node;
      node.parent = dense_parent_[v];
      node.subtree = dense_subtree_[v];
      node.children = std::move(dense_children_[v]);
      sparse_.insert({v, std::move(node)});
    }
    // Release the O(n) backing storage; it's the whole point.
    std::vector<VertexId>().swap(dense_parent_);
    std::vector<std::vector<VertexId>>().swap(dense_children_);
    std::vector<std::uint8_t>().swap(dense_alive_);
    std::vector<Score>().swap(dense_subtree_);
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
  // Dense backend: O(n) per array, indexed directly by vertex id. Used for
  // large trees, and transiently for every tree while it is being grown
  // (see the class comment).
  std::vector<VertexId> dense_parent_;
  std::vector<std::vector<VertexId>> dense_children_;
  std::vector<std::uint8_t> dense_alive_;
  std::vector<Score> dense_subtree_;
  // Sparse backend: O(live vertices) instead of O(n). Used for small trees,
  // which is the common case once the algorithm has made some progress
  // (see build_sample_tree's pruning).
  google::sparse_hash_map<VertexId, Node> sparse_;
};

}  // namespace rxl
#endif

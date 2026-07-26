#include "pruned_labeling.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <thread>

#include "abstract_dijkstra.h"
#include "parallel_for.h"
#include "sample_tree_storage.h"

namespace rxl {
namespace {
bool covered(const Label& label, const std::vector<Distance>& root_distance,
             Distance search_distance) {
  for (const auto& [hub, target_distance] : label) {
    const Distance root = root_distance[hub];
    if (root != kInfinity &&
        std::uint64_t(root) + target_distance <= search_distance)
      return true;
  }
  return false;
}

std::uint64_t pruned_dijkstra(const AdjacencyList& graph, VertexId root,
                              VertexId hub_id,
                              const std::vector<Distance>& root_distance,
                              HubLabels& labels, bool forward,
                              std::vector<Distance>& distance,
                              std::vector<VertexId>& touched) {
  std::uint64_t work = 0;
  AbstractDijkstra::search(
      root,
      [&graph](VertexId u) -> const std::vector<Edge>& { return graph[u]; },
      distance, touched,
      /*on_pop=*/[&](VertexId, Distance) { ++work; },
      /*should_prune=*/
      [&](VertexId u, Distance du) {
        const Label& query = forward ? labels[u].backward : labels[u].forward;
        return covered(query, root_distance, du);
      },
      /*on_settle=*/
      [&](VertexId u, Distance du) {
        Label& output = forward ? labels[u].backward : labels[u].forward;
        output.push_back(hub_id, du);
        work += graph[u].size();
      });
  for (VertexId v : touched) distance[v] = kInfinity;
  touched.clear();
  return work;
}

std::uint64_t add_hub(const Graph& graph, VertexId root, VertexId hub_id,
                      HubLabels& labels, std::vector<Distance>& root_out,
                      std::vector<Distance>& root_in,
                      std::vector<Distance>& distance,
                      std::vector<VertexId>& lookup_touched,
                      std::vector<VertexId>& search_touched) {
  for (const auto& [hub, d] : labels[root].forward) {
    root_out[hub] = d;
    lookup_touched.push_back(hub);
  }
  for (const auto& [hub, d] : labels[root].backward) {
    root_in[hub] = d;
    lookup_touched.push_back(hub);
  }
  std::uint64_t work =
      pruned_dijkstra(graph.adjacency(), root, hub_id, root_out, labels, true,
                      distance, search_touched);
  work += pruned_dijkstra(graph.reverse_adjacency(), root, hub_id, root_in,
                          labels, false, distance, search_touched);
  for (VertexId hub : lookup_touched) {
    root_out[hub] = kInfinity;
    root_in[hub] = kInfinity;
  }
  lookup_touched.clear();
  return work;
}

struct SampleTree {
  explicit SampleTree(std::size_t n) : storage(n) {}
  std::size_t bucket = 0;
  SampleTreeStorage storage;
  std::vector<VertexId> members;
  std::size_t remaining = 0;

  // Rewinds an already-retired SampleTree (remaining == 0) into a fresh,
  // empty one for a new root, reusing its storage's and members' existing
  // allocations instead of destroying this object and default-constructing
  // a new one -- see the tree_pool in sampled_order() below.
  void reset(std::size_t n, std::size_t new_bucket) {
    storage.reset(n);
    members.clear();  // keeps members' capacity for the new tree to refill.
    remaining = 0;
    bucket = new_bucket;
  }
};

struct TreeScratch {
  std::vector<Distance> root_lookup;
  std::vector<Distance> distance;
  std::vector<VertexId> lookup_touched;
  std::vector<VertexId> distance_touched;
};

void build_sample_tree(const Graph& graph, VertexId root,
                       const HubLabels& labels, std::size_t bucket,
                       std::uint64_t& work, TreeScratch& scratch,
                       SampleTree& tree) {
  const std::size_t n = graph.num_vertices();
  if (scratch.root_lookup.size() != n) {
    scratch.root_lookup.assign(n, kInfinity);
    scratch.distance.assign(n, kInfinity);
  }
  auto& root_lookup = scratch.root_lookup;
  auto& distance = scratch.distance;
  auto& lookup_touched = scratch.lookup_touched;
  auto& distance_touched = scratch.distance_touched;

  tree.reset(n, bucket);
  for (const auto& [hub, d] : labels[root].forward) {
    root_lookup[hub] = d;
    lookup_touched.push_back(hub);
  }
  // Grown directly into tree.members (already emptied by tree.reset() above,
  // but keeping whatever capacity it held from this SampleTree's previous
  // life) rather than a fresh local vector that gets move-assigned in at
  // the end.
  auto& settled = tree.members;
  AbstractDijkstra::search(
      root,
      [&graph](VertexId u) -> const std::vector<Edge>& {
        return graph.adjacency()[u];
      },
      distance, distance_touched,
      /*on_pop=*/[&](VertexId, Distance) { ++work; },
      /*should_prune=*/
      [&](VertexId u, Distance du) {
        return covered(labels[u].backward, root_lookup, du);
      },
      /*on_settle=*/
      [&](VertexId u, Distance) {
        tree.storage.mark_alive(u);
        settled.push_back(u);
        work += graph.adjacency()[u].size();
      },
      /*on_relax=*/
      [&](VertexId u, VertexId v, Distance) { tree.storage.set_parent(v, u); },
      /*on_non_improving_edge=*/
      [&](VertexId u, VertexId v, std::uint64_t candidate) {
        if (candidate == distance[v] && distance[v] != kInfinity &&
            u < tree.storage.parent_of(v))
          tree.storage.set_parent(v, u);
      });
  for (VertexId v : settled) {
    const VertexId parent = tree.storage.parent_of(v);
    if (parent != kInvalidVertex && tree.storage.alive(parent))
      tree.storage.add_child(parent, v);
  }
  for (auto it = settled.rbegin(); it != settled.rend(); ++it) {
    const VertexId v = *it;
    tree.storage.add_subtree(v, 1);
    const VertexId parent = tree.storage.parent_of(v);
    if (parent != kInvalidVertex && tree.storage.alive(parent))
      tree.storage.add_subtree(parent, tree.storage.subtree_of(v));
  }
  tree.remaining = settled.size();
  tree.storage.maybe_downgrade(tree.members, tree.remaining);

  for (VertexId v : distance_touched) distance[v] = kInfinity;
  distance_touched.clear();
  for (VertexId hub : lookup_touched) root_lookup[hub] = kInfinity;
  lookup_touched.clear();
}

// `touch(v)` is called for every vertex whose counters[v] just changed, so
// the caller (sampled_order()) can keep its selection heap in sync (see the
// HeapEntry/version machinery below). Templated on the callable type so
// this stays a plain inline call in the hot path, not a std::function
// indirection.
template <class TouchFn>
void add_tree_scores(const SampleTree& tree,
                     std::vector<std::vector<Score>>& counters,
                     TouchFn&& touch) {
  for (VertexId v : tree.members) {
    if (tree.storage.alive(v)) {
      counters[v][tree.bucket] += tree.storage.subtree_of(v);
      touch(v);
    }
  }
}

// Registers `tree` in the reverse index (Appendix A.2): for every vertex
// still alive in it, record that this tree is one of the (usually few)
// live trees currently containing that vertex. Called once, right after a
// tree finishes growing (see grow_batch's built-processing loop below) --
// not from inside build_sample_tree itself, since that runs concurrently
// across lanes and two different trees could settle the same vertex in the
// same batch, which would race on membership[v].
void register_membership(SampleTree* tree,
                         std::vector<std::vector<SampleTree*>>& membership) {
  for (VertexId v : tree->members)
    if (tree->storage.alive(v)) membership[v].push_back(tree);
}

template <class TouchFn>
std::size_t remove_subtree(SampleTree& tree, VertexId hub,
                           std::vector<std::vector<Score>>& counters,
                           bool& downgraded, TouchFn&& touch,
                           std::vector<std::vector<SampleTree*>>& membership) {
  downgraded = false;
  auto& storage = tree.storage;
  if (!storage.alive(hub)) return 0;
  const Score removed = storage.subtree_of(hub);
  VertexId ancestor = storage.parent_of(hub);
  while (ancestor != kInvalidVertex && storage.alive(ancestor)) {
    counters[ancestor][tree.bucket] -= removed;
    storage.set_subtree(ancestor, storage.subtree_of(ancestor) - removed);
    touch(ancestor);
    ancestor = storage.parent_of(ancestor);
  }
  std::vector<VertexId> stack{hub};
  while (!stack.empty()) {
    const VertexId v = stack.back();
    stack.pop_back();
    if (!storage.alive(v)) continue;
    counters[v][tree.bucket] -= storage.subtree_of(v);
    touch(v);
    const std::vector<VertexId> children = storage.children_of(v);
    storage.deactivate(v);
    // v is no longer alive in `tree`: drop it from the reverse index.
    // membership[v] is short (bounded by how many live trees currently
    // contain v), so a linear swap-and-pop is cheap and avoids pulling in
    // a set/hash-set just for this.
    auto& lst = membership[v];
    for (std::size_t i = 0; i < lst.size(); ++i) {
      if (lst[i] == &tree) {
        lst[i] = lst.back();
        lst.pop_back();
        break;
      }
    }
    for (VertexId child : children) stack.push_back(child);
  }
  tree.remaining -= static_cast<std::size_t>(removed);
  downgraded = storage.maybe_downgrade(tree.members, tree.remaining);
  return static_cast<std::size_t>(removed);
}

// Once a tree has shrunk to a handful of remaining vertices, its odds of
// ever hitting exactly 0 (which is the only thing that lets the ranking
// loop below actually drop it from `trees`) can be very poor: reaching 0
// requires every one of its surviving vertices to eventually be selected
// as a hub, which for a straggling fragment may not happen for a very long
// time. Meanwhile every *other* live tree pays a fixed O(1) tax for that
// fragment's continued existence every single rank (the remove_subtree
// sweep in sampled_order() below), so a large population of such stragglers
// turns each rank's bookkeeping into O(live trees) work that keeps growing
// instead of staying roughly proportional to `buckets`. This forces a
// shrunk tree fully empty -- as if every surviving vertex had just been
// selected as a hub, via the exact same remove_subtree() used for that --
// so it becomes eligible for the ordinary retirement sweep this same rank,
// instead of lingering indefinitely. `tree.members` is the tree's original
// (fixed, from construction) settle list, so this is O(that tree's own
// size), paid once, when it's evicted -- not on every subsequent rank.
template <class TouchFn>
void retire_remaining(SampleTree& tree,
                      std::vector<std::vector<Score>>& counters,
                      std::size_t& live_vertices, TouchFn&& touch,
                      std::vector<std::vector<SampleTree*>>& membership) {
  for (VertexId v : tree.members) {
    if (!tree.storage.alive(v)) continue;
    const VertexId parent = tree.storage.parent_of(v);
    // Skip anything whose parent is still alive: it'll be swept up when its
    // (still-alive) ancestor's fragment is removed below, so removing it
    // here too would double-subtract its contribution from `counters`.
    if (parent != kInvalidVertex && tree.storage.alive(parent)) continue;
    bool downgraded = false;
    live_vertices -=
        remove_subtree(tree, v, counters, downgraded, touch, membership);
  }
}

std::pair<Score, Score> priority(const std::vector<Score>& counter,
                                 std::size_t discard,
                                 std::vector<Score>& scratch) {
  scratch.assign(counter.begin(), counter.end());
  std::sort(scratch.begin(), scratch.end(), std::greater<Score>());
  discard = std::min(discard, scratch.size());
  Score robust = 0, total = 0;
  for (Score value : counter) total += value;
  for (std::size_t i = discard; i < scratch.size(); ++i) robust += scratch[i];
  return {robust, total};
}

// Lazy max-heap entry for vertex selection (Appendix A.2). Replaces the
// original O(n)-per-rank rescan ("for v in 0..n, if not selected, compute
// priority(v)") -- which makes the whole ordering algorithm O(n^2) and is
// the actual reason large inputs are impractical -- with an O(log n) push
// per counter update and an amortized O(log n) pop per rank.
//
// A vertex's priority can both increase (a new sampled tree adds to its
// counters) and decrease (an existing tree's subtree is removed), so this
// can't use the simple "priorities only decrease" lazy-deletion trick from
// Dijkstra's algorithm. Instead every counter change pushes a brand new
// entry stamped with the vertex's current version (see `touch()` in
// sampled_order()); an entry is valid exactly when its stamped version
// still matches the vertex's current version. Since the *last* push for any
// vertex always carries its true current priority, popping the heap in
// priority order and discarding stale (mismatched-version) or already-
// selected entries always yields the same "best remaining vertex" the
// original linear scan would have found -- with the same tie-break order:
// higher (robust, total) priority first, then higher static degree, then
// (to match the original scan's left-to-right, first-wins behavior on a
// full tie) smaller vertex id.
struct HeapEntry {
  Score robust;
  Score total;
  std::size_t degree;
  std::uint32_t version;
  VertexId v;
};
struct HeapEntryLess {
  bool operator()(const HeapEntry& a, const HeapEntry& b) const {
    if (a.robust != b.robust) return a.robust < b.robust;
    if (a.total != b.total) return a.total < b.total;
    if (a.degree != b.degree) return a.degree < b.degree;
    return a.v > b.v;  // smaller vertex id wins ties, so it must sort higher.
  }
};
using SelectionHeap =
    std::priority_queue<HeapEntry, std::vector<HeapEntry>, HeapEntryLess>;

std::vector<VertexId> sampled_order(const Graph& graph,
                                    const SamplingOptions& options,
                                    HubLabels& labels, BuildStatistics& stats) {
  const std::size_t n = graph.num_vertices();
  const std::size_t buckets = std::max<std::size_t>(1, options.counter_buckets);
  const std::size_t initial =
      std::min(n, std::max(options.initial_trees, buckets));
  std::mt19937_64 random(options.random_seed);
  std::vector<VertexId> roots(n);
  std::iota(roots.begin(), roots.end(), VertexId{0});
  std::shuffle(roots.begin(), roots.end(), random);

  // Trees are held behind unique_ptr so their addresses stay stable while
  // `trees` itself gets reordered/compacted (push_back, the erase-remove
  // below) -- required for the reverse-index `membership` map just below to
  // hold raw SampleTree* pointers safely across a tree's whole lifetime.
  std::vector<std::unique_ptr<SampleTree>> trees;
  trees.reserve(initial + n / 8 + 1);
  // Retired trees (remaining == 0, see the erase-remove below) are parked
  // here instead of being destroyed, so growing a new tree can rewind one
  // of these in place (SampleTree::reset()) rather than default-
  // constructing a brand new SampleTree(n) -- which would mean freshly
  // allocating its four O(n) dense arrays every single time a new sample
  // tree is grown (Section 3.3 / Appendix A.2: this happens repeatedly
  // throughout the run, not just for the `initial` trees).
  std::vector<std::unique_ptr<SampleTree>> tree_pool;
  auto acquire_tree = [&]() -> std::unique_ptr<SampleTree> {
    if (!tree_pool.empty()) {
      std::unique_ptr<SampleTree> tree = std::move(tree_pool.back());
      tree_pool.pop_back();
      return tree;
    }
    return std::make_unique<SampleTree>(n);
  };
  std::vector<std::vector<Score>> counters(n, std::vector<Score>(buckets, 0));

  // Reverse index (Appendix A.2): membership[v] lists the live trees that
  // currently have v alive, so the ranking loop below can find "which live
  // trees contain the vertex just picked as hub" directly, instead of
  // probing every live tree each rank.
  std::vector<std::vector<SampleTree*>> membership(n);

  // A vertex's static degree is a fixed tie-breaker for selection (see
  // priority()'s callers), so it's computed once here rather than
  // recomputed for every unselected vertex on every rank, as the original
  // O(n)-per-rank scan did.
  std::vector<std::size_t> degree(n);
  for (VertexId v = 0; v < n; ++v)
    degree[v] =
        graph.adjacency()[v].size() + graph.reverse_adjacency()[v].size();

  // Selection heap (Appendix A.2): see HeapEntry's comment above. version[v]
  // is bumped on every counters[v] change; touch(v) recomputes v's current
  // priority and pushes a freshly-versioned entry. priority_scratch is
  // shared scratch space for priority()'s internal sort.
  std::vector<std::uint32_t> version(n, 0);
  std::vector<Score> priority_scratch;
  SelectionHeap heap;
  auto touch = [&](VertexId v) {
    ++version[v];
    const auto p =
        priority(counters[v], options.discarded_max_buckets, priority_scratch);
    heap.push(HeapEntry{p.first, p.second, degree[v], version[v], v});
  };

  std::uint64_t tree_work = 0, label_work = 0;
  const std::size_t threads = std::max<std::size_t>(1, options.num_threads);
  std::size_t live_vertices = 0;
  std::vector<TreeScratch> scratch_pool(threads);
  auto grow_batch = [&](const std::vector<VertexId>& batch_roots,
                        std::size_t first_bucket) {
    using Built = std::pair<std::unique_ptr<SampleTree>, std::uint64_t>;
    std::vector<Built> built;
    built.reserve(batch_roots.size());
    if (threads == 1 || batch_roots.size() == 1) {
      for (std::size_t i = 0; i < batch_roots.size(); ++i) {
        std::uint64_t work = 0;
        std::unique_ptr<SampleTree> tree = acquire_tree();
        build_sample_tree(graph, batch_roots[i], labels,
                          (first_bucket + i) % buckets, work, scratch_pool[0],
                          *tree);
        built.emplace_back(std::move(tree), work);
      }
    } else {
      // tree_pool is only ever touched from the main thread: every lane's
      // tree is acquired here, before any std::async task is launched, and
      // handed to that lane by reference -- each async task only ever
      // touches its own lane_trees[i], so there is no concurrent access to
      // the pool itself or to any other lane's tree.
      std::vector<std::unique_ptr<SampleTree>> lane_trees;
      lane_trees.reserve(batch_roots.size());
      for (std::size_t i = 0; i < batch_roots.size(); ++i)
        lane_trees.push_back(acquire_tree());
      std::vector<std::future<std::uint64_t>> futures;
      futures.reserve(batch_roots.size());
      for (std::size_t i = 0; i < batch_roots.size(); ++i) {
        const VertexId root = batch_roots[i];
        const std::size_t bucket = (first_bucket + i) % buckets;
        TreeScratch& lane = scratch_pool[i];
        SampleTree& tree_ref = *lane_trees[i];
        futures.emplace_back(std::async(std::launch::async, [&graph, &labels,
                                                             &lane, &tree_ref,
                                                             root, bucket] {
          std::uint64_t work = 0;
          build_sample_tree(graph, root, labels, bucket, work, lane, tree_ref);
          return work;
        }));
      }
      for (std::size_t i = 0; i < futures.size(); ++i) {
        const std::uint64_t work = futures[i].get();
        built.emplace_back(std::move(lane_trees[i]), work);
      }
    }
    for (auto& item : built) {
      tree_work += item.second;
      stats.sampling_work += item.second;
      add_tree_scores(*item.first, counters, touch);
      register_membership(item.first.get(), membership);
      live_vertices += item.first->remaining;
      if (!item.first->storage.is_dense()) ++stats.sparse_downgrades;
      trees.push_back(std::move(item.first));
      ++stats.sampled_trees;
    }
  };
  for (std::size_t begin = 0; begin < initial; begin += threads) {
    const std::size_t end = std::min(initial, begin + threads);
    grow_batch(
        std::vector<VertexId>(roots.begin() + begin, roots.begin() + end),
        trees.size() % buckets);
  }
  tree_work = 0;

  // Seed the selection heap for every vertex the initial trees never
  // touched (their counters are still all-zero, but they still need a
  // valid, current heap entry to ever be selectable). Vertices the initial
  // trees already touched via add_tree_scores()'s touch() calls above
  // already have a live entry reflecting their real priority; re-touching
  // them here would just be a harmless but wasted extra push, so skip them
  // via the version check.
  for (VertexId v = 0; v < n; ++v)
    if (version[v] == 0) touch(v);

  std::vector<VertexId> order;
  order.reserve(n);
  std::vector<std::uint8_t> selected(n, 0);
  std::vector<Distance> root_out(n, kInfinity), root_in(n, kInfinity),
      distance(n, kInfinity);
  std::vector<VertexId> lookup_touched, search_touched;
  std::size_t next_root = initial;
  const std::size_t factor = 10 * std::max<std::size_t>(1, initial);
  const std::size_t max_tree_vertices =
      n > std::numeric_limits<std::size_t>::max() / factor
          ? std::numeric_limits<std::size_t>::max()
          : factor * n;

  for (std::size_t rank = 0; rank < n; ++rank) {
    // Lazy max-heap pop (Appendix A.2) instead of an O(n) rescan: keep
    // popping until we find an entry that is neither already selected nor
    // stale (see HeapEntry's comment). Every vertex always has at least one
    // valid entry in the heap until it's selected, so this always
    // terminates with a valid `best` before the heap empties.
    VertexId best = kInvalidVertex;
    while (true) {
      if (heap.empty())
        throw std::logic_error(
            "SamPG: selection heap exhausted before all vertices were "
            "ranked");
      const HeapEntry top = heap.top();
      heap.pop();
      if (selected[top.v] || top.version != version[top.v]) continue;
      best = top.v;
      break;
    }
    selected[best] = 1;
    order.push_back(best);
    label_work +=
        add_hub(graph, best, static_cast<VertexId>(rank), labels, root_out,
                root_in, distance, lookup_touched, search_touched);

    if (options.verbose &&
        (rank < 10 || (rank + 1) % 1000 == 0 || rank + 1 == n))
      std::cerr << "[rxl] rank " << (rank + 1) << '/' << n
                << ": selected vertex " << best << ", live trees "
                << trees.size() << "\n";

    // Reverse index (Appendix A.2): remove `best` only from the (typically
    // short) list of trees that actually contain it, instead of probing
    // every live tree. Copy the list first since remove_subtree() mutates
    // membership[best] as it deactivates `best` in each of these trees, and
    // mutating a container while ranging over it is undefined behavior.
    const std::vector<SampleTree*> containing_best = membership[best];
    for (SampleTree* tree_ptr : containing_best) {
      bool downgraded = false;
      live_vertices -= remove_subtree(*tree_ptr, best, counters, downgraded,
                                      touch, membership);
      if (downgraded) ++stats.sparse_downgrades;
    }
    // Every *live* tree's retirement eligibility is still re-checked every
    // rank, not just the ones `best` happened to be a member of: a newly
    // grown tree can already be at or below the threshold the moment it's
    // born (see grow_batch above), and this is also how long-lingering
    // stragglers (see retire_remaining's own comment) eventually get swept
    // -- neither case depends on `best` being one of their members. The
    // number of live trees is bounded by options.max_live_trees regardless
    // of n (default 1024), so this stays independent of graph size, unlike
    // the vertex-selection scan this change replaces.
    for (auto& tree_ptr : trees)
      if (tree_ptr->remaining > 0 &&
          tree_ptr->remaining <= options.min_tree_vertices)
        retire_remaining(*tree_ptr, counters, live_vertices, touch, membership);

    trees.erase(std::remove_if(trees.begin(), trees.end(),
                               [&](std::unique_ptr<SampleTree>& tree) {
                                 if (tree->remaining != 0) return false;
                                 // Move out this tree's guts (storage's
                                 // dense arrays, members' capacity) before
                                 // it's logically "removed" -- it may still
                                 // get overwritten in place by a later
                                 // surviving element's move-assignment as
                                 // remove_if compacts the vector, but that's
                                 // fine, we've already taken what we need.
                                 tree_pool.push_back(std::move(tree));
                                 return true;
                               }),
                trees.end());

    std::size_t attempts = 0;
    while ((trees.size() < buckets || tree_work <= label_work) &&
           trees.size() < options.max_live_trees &&
           live_vertices < max_tree_vertices && attempts < 2 * n) {
      std::vector<VertexId> batch;
      batch.reserve(threads);
      while (batch.size() < threads && attempts < 2 * n) {
        if (next_root == n) {
          std::shuffle(roots.begin(), roots.end(), random);
          next_root = 0;
        }
        const VertexId root = roots[next_root++];
        ++attempts;
        if (!selected[root]) batch.push_back(root);
      }
      if (batch.empty()) break;
      grow_batch(batch, trees.size() % buckets);
    }
    stats.peak_live_trees = std::max(stats.peak_live_trees, trees.size());
  }
  stats.labeling_work = label_work;
  return order;
}

LabelingResult finish(HubLabels labels, std::vector<VertexId> order) {
  std::vector<Rank> ranks(order.size());
  for (Rank rank = 0; rank < order.size(); ++rank) ranks[order[rank]] = rank;
  return {std::move(labels), std::move(order), std::move(ranks), {}};
}
}  // namespace

std::vector<VertexId> PrunedLabeling::degree_order(const Graph& graph) {
  std::vector<VertexId> order(graph.num_vertices());
  std::iota(order.begin(), order.end(), VertexId{0});
  std::sort(order.begin(), order.end(), [&](VertexId a, VertexId b) {
    const auto da =
        graph.adjacency()[a].size() + graph.reverse_adjacency()[a].size();
    const auto db =
        graph.adjacency()[b].size() + graph.reverse_adjacency()[b].size();
    return da != db ? da > db : a < b;
  });
  return order;
}

LabelingResult PrunedLabeling::compute_with_degree_order(const Graph& graph) {
  const std::size_t n = graph.num_vertices();
  HubLabels labels(n);
  auto order = degree_order(graph);
  std::vector<Distance> root_out(n, kInfinity), root_in(n, kInfinity),
      distance(n, kInfinity);
  std::vector<VertexId> lookup, touched;
  for (std::size_t rank = 0; rank < order.size(); ++rank)
    add_hub(graph, order[rank], static_cast<VertexId>(rank), labels, root_out,
            root_in, distance, lookup, touched);
  return finish(std::move(labels), std::move(order));
}

LabelingResult PrunedLabeling::compute(const Graph& graph,
                                       const SamplingOptions& options) {
  if (options.initial_trees == 0) return compute_with_degree_order(graph);
  if (options.discarded_max_buckets >= options.counter_buckets)
    throw std::invalid_argument(
        "SamPG must retain at least one counter bucket");
  HubLabels labels(graph.num_vertices());
  BuildStatistics statistics;
  const auto start = std::chrono::steady_clock::now();
  auto order = sampled_order(graph, options, labels, statistics);
  auto result = finish(std::move(labels), std::move(order));
  statistics.ordering_and_labeling_seconds =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
          .count();
  result.statistics = statistics;
  return result;
}

void PrunedLabeling::reorder_labels_by_rank(LabelingResult& result) {
  const std::size_t n = result.labels.size();
  if (result.rank_to_vertex.size() != n || result.vertex_to_rank.size() != n)
    throw std::invalid_argument("Incomplete rank information");
  HubLabels reordered(n);
  const std::size_t threads =
      std::max<std::size_t>(1, std::thread::hardware_concurrency());
  parallel_for(n, threads, [&](std::size_t lo, std::size_t hi) {
    for (std::size_t i = lo; i < hi; ++i) {
      const VertexId old_owner = static_cast<VertexId>(i);
      const VertexId new_owner = result.vertex_to_rank[old_owner];
      reordered[new_owner] = std::move(result.labels[old_owner]);
    }
  });
  result.labels = std::move(reordered);
  result.rank_reordered = true;
}
}  // namespace rxl

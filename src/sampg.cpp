#include "sampg.h"

#include <algorithm>
#include <array>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <random>
#include <stdexcept>
#include <thread>

#include "abstract_dijkstra.h"
#include "addressable_heap.h"
#include "indexed_minheap.h"
#include "pruned_dijkstra.h"
#include "sample_tree_storage.h"

namespace rxl {
namespace {
constexpr std::size_t kMaxCounterBuckets = 32;

struct SampleTree {
  SampleTree(std::size_t n, DenseBufferPool& pool) : storage(n, pool) {}
  std::size_t bucket = 0;
  SampleTreeStorage storage;
  std::vector<VertexId> members;
  std::size_t remaining = 0;
  std::size_t list_index = 0;

  void reset(std::size_t n, std::size_t new_bucket, DenseBufferPool& pool) {
    storage.reset(n, pool);
    members.clear();
    remaining = 0;
    bucket = new_bucket;
  }
};

struct TreeScratch {
  std::vector<Distance> root_lookup;
  std::vector<Distance> distance;
  std::vector<VertexId> lookup_touched;
  std::vector<VertexId> distance_touched;
  dijkstra_detail::IndexedMinHeap heap;
};

void build_sample_tree(const Graph& graph, VertexId root,
                       const HubLabels& labels, std::size_t bucket,
                       std::uint64_t& work, TreeScratch& scratch,
                       SampleTree& tree, DenseBufferPool& dense_pool) {
  const std::size_t n = graph.num_vertices();
  if (scratch.root_lookup.size() != n) {
    scratch.root_lookup.assign(n, kInfinity);
    scratch.distance.assign(n, kInfinity);
    scratch.heap.assign(n);
  }
  auto& root_lookup = scratch.root_lookup;
  auto& distance = scratch.distance;
  auto& lookup_touched = scratch.lookup_touched;
  auto& distance_touched = scratch.distance_touched;
  auto& heap = scratch.heap;

  tree.reset(n, bucket, dense_pool);
  for (const auto& [hub, d] : labels[root].forward) {
    root_lookup[hub] = d;
    lookup_touched.push_back(hub);
  }
  auto& settled = tree.members;
  AbstractDijkstra::search(
      root, [&graph](VertexId u) { return graph.adjacency()[u]; }, distance,
      distance_touched, heap,
      /*on_pop=*/[&](VertexId, Distance) { ++work; },
      /*should_prune=*/
      [&](VertexId u, Distance du) {
        return HubExpansion::covered(labels[u].backward, root_lookup, du);
      },
      /*on_settle=*/
      [&](VertexId u, Distance) {
        tree.storage.mark_alive(u);
        settled.push_back(u);
        work += graph.adjacency()[u].size();
      },
      /*on_relax=*/
      [&](VertexId u, VertexId v, Distance) {
        tree.storage.set_parent(v, u);
        // Same rationale as HubExpansion's pruned_dijkstra on_relax: warm
        // up v's backward label now, ahead of the should_prune call that
        // will read it once v reaches the front of the heap.
        labels[v].backward.prefetch();
      },
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

template <class TouchFn>
void add_tree_scores(const SampleTree& tree, std::size_t buckets,
                     std::vector<Score>& counters, TouchFn&& touch) {
  for (VertexId v : tree.members) {
    if (tree.storage.alive(v)) {
      counters[static_cast<std::size_t>(v) * buckets + tree.bucket] +=
          tree.storage.subtree_of(v);
      touch(v);
    }
  }
}

void register_membership(SampleTree* tree,
                         std::vector<std::vector<SampleTree*>>& membership) {
  for (VertexId v : tree->members)
    if (tree->storage.alive(v)) membership[v].push_back(tree);
}

template <class TouchFn>
std::size_t remove_subtree(SampleTree& tree, VertexId hub, std::size_t buckets,
                           std::vector<Score>& counters, bool& downgraded,
                           TouchFn&& touch,
                           std::vector<std::vector<SampleTree*>>& membership) {
  downgraded = false;
  auto& storage = tree.storage;
  if (!storage.alive(hub)) return 0;
  const Score removed = storage.subtree_of(hub);
  VertexId ancestor = storage.parent_of(hub);
  while (ancestor != kInvalidVertex && storage.alive(ancestor)) {
    counters[static_cast<std::size_t>(ancestor) * buckets + tree.bucket] -=
        removed;
    storage.set_subtree(ancestor, storage.subtree_of(ancestor) - removed);
    touch(ancestor);
    ancestor = storage.parent_of(ancestor);
  }
  std::vector<VertexId> stack{hub};
  while (!stack.empty()) {
    const VertexId v = stack.back();
    stack.pop_back();
    if (!storage.alive(v)) continue;
    counters[static_cast<std::size_t>(v) * buckets + tree.bucket] -=
        storage.subtree_of(v);
    touch(v);
    const std::vector<VertexId> children = storage.children_of(v);
    storage.deactivate(v);
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

template <class TouchFn>
void retire_remaining(SampleTree& tree, std::size_t buckets,
                      std::vector<Score>& counters, std::size_t& live_vertices,
                      TouchFn&& touch,
                      std::vector<std::vector<SampleTree*>>& membership) {
  for (VertexId v : tree.members) {
    if (!tree.storage.alive(v)) continue;
    const VertexId parent = tree.storage.parent_of(v);
    if (parent != kInvalidVertex && tree.storage.alive(parent)) continue;
    bool downgraded = false;
    live_vertices -= remove_subtree(tree, v, buckets, counters, downgraded,
                                    touch, membership);
  }
}

void retire_tree(std::size_t idx,
                 std::vector<std::unique_ptr<SampleTree>>& trees,
                 std::vector<std::unique_ptr<SampleTree>>& tree_pool) {
  tree_pool.push_back(std::move(trees[idx]));
  const std::size_t last = trees.size() - 1;
  if (idx != last) {
    trees[idx] = std::move(trees[last]);
    trees[idx]->list_index = idx;
  }
  trees.pop_back();
}

std::pair<Score, Score> priority(const Score* counter, std::size_t buckets,
                                 std::size_t discard) {
  std::array<Score, kMaxCounterBuckets> scratch;
  std::copy(counter, counter + buckets, scratch.begin());
  const Score total = std::accumulate(counter, counter + buckets, Score(0));

  discard = std::min(discard, buckets);
  if (discard > 0) {
    std::nth_element(scratch.begin(), scratch.begin() + (discard - 1),
                     scratch.begin() + buckets, std::greater<Score>());
  }
  const Score robust = std::accumulate(scratch.begin() + discard,
                                       scratch.begin() + buckets, Score(0));
  return {robust, total};
}

}  // namespace

std::vector<VertexId> SamPG::build_order(const Graph& graph,
                                         const SamplingOptions& options,
                                         HubLabels& labels,
                                         BuildStatistics& stats) {
  if (options.discarded_max_buckets >= options.counter_buckets)
    throw std::invalid_argument(
        "SamPG must retain at least one counter bucket");
  if (options.counter_buckets > kMaxCounterBuckets)
    throw std::invalid_argument(
        "SamPG: counter_buckets exceeds the supported maximum");

  const std::size_t n = graph.num_vertices();
  const std::size_t buckets = std::max<std::size_t>(1, options.counter_buckets);
  const std::size_t initial =
      std::min(n, std::max(options.initial_trees, buckets));
  std::mt19937_64 random(options.random_seed);
  std::vector<VertexId> roots(n);
  std::iota(roots.begin(), roots.end(), VertexId{0});
  std::shuffle(roots.begin(), roots.end(), random);
  std::vector<std::unique_ptr<SampleTree>> trees;
  trees.reserve(initial + n / 8 + 1);
  std::vector<std::unique_ptr<SampleTree>> tree_pool;
  DenseBufferPool dense_pool;
  auto acquire_tree = [&]() -> std::unique_ptr<SampleTree> {
    if (!tree_pool.empty()) {
      std::unique_ptr<SampleTree> tree = std::move(tree_pool.back());
      tree_pool.pop_back();
      return tree;
    }
    return std::make_unique<SampleTree>(n, dense_pool);
  };
  std::vector<Score> counters(n * buckets, Score(0));

  std::vector<std::vector<SampleTree*>> membership(n);

  std::vector<std::size_t> degree(n);
  for (VertexId v = 0; v < n; ++v)
    degree[v] =
        graph.adjacency()[v].size() + graph.reverse_adjacency()[v].size();
  std::vector<std::uint8_t> selected(n, 0);

  std::vector<Score> priority_scratch;
  AddressableHeap heap(n);

  auto touch = [&](VertexId v) {
    if (selected[v]) return;
    const auto p = priority(&counters[static_cast<std::size_t>(v) * buckets],
                            buckets, options.discarded_max_buckets);
    heap.set(v, p.first, p.second, degree[v]);
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
                          *tree, dense_pool);
        built.emplace_back(std::move(tree), work);
      }
    } else {
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
        futures.emplace_back(std::async(
            std::launch::async,
            [&graph, &labels, &lane, &tree_ref, root, bucket, &dense_pool] {
              std::uint64_t work = 0;
              build_sample_tree(graph, root, labels, bucket, work, lane,
                                tree_ref, dense_pool);
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
      add_tree_scores(*item.first, buckets, counters, touch);
      register_membership(item.first.get(), membership);
      live_vertices += item.first->remaining;
      if (!item.first->storage.is_dense()) ++stats.sparse_downgrades;
      trees.push_back(std::move(item.first));
      trees.back()->list_index = trees.size() - 1;
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

  for (VertexId v = 0; v < n; ++v)
    if (!heap.contains(v)) touch(v);

  std::vector<VertexId> order;
  order.reserve(n);
  std::vector<Distance> root_out(n, kInfinity), root_in(n, kInfinity),
      distance(n, kInfinity);
  std::vector<VertexId> lookup_touched, search_touched;
  dijkstra_detail::IndexedMinHeap dijkstra_heap(n);
  std::size_t next_root = initial;
  const std::size_t factor = 10 * std::max<std::size_t>(1, initial);
  const std::size_t max_tree_vertices =
      n > std::numeric_limits<std::size_t>::max() / factor
          ? std::numeric_limits<std::size_t>::max()
          : factor * n;

  for (std::size_t rank = 0; rank < n; ++rank) {
    if (heap.empty())
      throw std::logic_error(
          "SamPG: selection heap exhausted before all vertices were "
          "ranked");
    const VertexId best = heap.pop_top();
    selected[best] = 1;
    order.push_back(best);
    label_work += HubExpansion::add_hub(
        graph, best, static_cast<VertexId>(rank), labels, root_out, root_in,
        distance, lookup_touched, search_touched, dijkstra_heap);

    if (options.verbose &&
        (rank < 10 || (rank + 1) % 1000 == 0 || rank + 1 == n))
      std::cerr << "[rxl] rank " << (rank + 1) << '/' << n
                << ": selected vertex " << best << ", live trees "
                << trees.size() << "\n";

    const std::vector<SampleTree*> containing_best = membership[best];
    for (SampleTree* tree_ptr : containing_best) {
      bool downgraded = false;
      live_vertices -= remove_subtree(*tree_ptr, best, buckets, counters,
                                      downgraded, touch, membership);
      if (downgraded) ++stats.sparse_downgrades;
      if (tree_ptr->remaining > 0 &&
          tree_ptr->remaining <= options.min_tree_vertices)
        retire_remaining(*tree_ptr, buckets, counters, live_vertices, touch,
                         membership);
      if (tree_ptr->remaining == 0)
        retire_tree(tree_ptr->list_index, trees, tree_pool);
    }

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

}  // namespace rxl

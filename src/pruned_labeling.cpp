#include "pruned_labeling.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <functional>
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
#include "parallel_for.h"
#include "sample_tree_storage.h"

namespace rxl {
namespace {
constexpr std::size_t kMaxCounterBuckets = 32;

bool covered(const Label& label, const std::vector<Distance>& root_distance,
             Distance search_distance) {
  const auto& deltas = label.raw_deltas();
  const std::size_t m = deltas.size();
  if (m == 0) return false;

  constexpr std::size_t kPrefetchAhead = 6;
  VertexId ahead_hub = deltas[0];
  std::size_t ahead_index = 0;
  auto prefetch_next_hub = [&] {
    __builtin_prefetch(&root_distance[ahead_hub], /*rw=*/0, /*locality=*/1);
    if (++ahead_index < m)
      ahead_hub = static_cast<VertexId>(ahead_hub + 1 + deltas[ahead_index]);
  };
  for (std::size_t k = 0; k < kPrefetchAhead && k < m; ++k) prefetch_next_hub();

  for (const auto& [hub, target_distance] : label) {
    if (ahead_index < m) prefetch_next_hub();
    const Distance root = root_distance[hub];
    if (std::uint64_t(root) + target_distance <= search_distance) return true;
  }
  return false;
}

std::uint64_t pruned_dijkstra(const AdjacencyList& graph, VertexId root,
                              VertexId hub_id,
                              const std::vector<Distance>& root_distance,
                              HubLabels& labels, bool forward,
                              std::vector<Distance>& distance,
                              std::vector<VertexId>& touched,
                              dijkstra_detail::IndexedMinHeap& heap) {
  std::uint64_t work = 0;
  AbstractDijkstra::search(
      root,
      [&graph](VertexId u) -> const std::vector<Edge>& { return graph[u]; },
      distance, touched, heap,
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
      },
      /*on_relax=*/
      [&](VertexId, VertexId v, Distance) {
        (forward ? labels[v].backward : labels[v].forward).prefetch();
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
                      std::vector<VertexId>& search_touched,
                      dijkstra_detail::IndexedMinHeap& heap) {
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
                      distance, search_touched, heap);
  work += pruned_dijkstra(graph.reverse_adjacency(), root, hub_id, root_in,
                          labels, false, distance, search_touched, heap);
  for (VertexId hub : lookup_touched) {
    root_out[hub] = kInfinity;
    root_in[hub] = kInfinity;
  }
  lookup_touched.clear();
  return work;
}

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
      root,
      [&graph](VertexId u) -> const std::vector<Edge>& {
        return graph.adjacency()[u];
      },
      distance, distance_touched, heap,
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
      [&](VertexId u, VertexId v, Distance) {
        tree.storage.set_parent(v, u);
        // Same rationale as pruned_dijkstra's on_relax above: warm up
        // v's backward label now, ahead of the should_prune call that
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
    label_work += add_hub(graph, best, static_cast<VertexId>(rank), labels,
                          root_out, root_in, distance, lookup_touched,
                          search_touched, dijkstra_heap);

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

LabelingResult PrunedLabeling::compute_with_degree_order(const Graph& graph,
                                                         const bool verbose) {
  const std::size_t n = graph.num_vertices();
  HubLabels labels(n);
  auto order = degree_order(graph);
  std::vector<Distance> root_out(n, kInfinity), root_in(n, kInfinity),
      distance(n, kInfinity);
  std::vector<VertexId> lookup, touched;
  dijkstra_detail::IndexedMinHeap heap(n);
  for (std::size_t rank = 0; rank < order.size(); ++rank) {
    if (verbose && (rank < 10 || (rank + 1) % 1000 == 0 || rank + 1 == n)) {
      std::cerr << "[rxl] rank " << (rank + 1) << '/' << n
                << ": selected vertex " << order[rank] << "\n";
    }

    add_hub(graph, order[rank], static_cast<VertexId>(rank), labels, root_out,
            root_in, distance, lookup, touched, heap);
  }
  return finish(std::move(labels), std::move(order));
}

LabelingResult PrunedLabeling::compute(const Graph& graph,
                                       const SamplingOptions& options) {
  if (options.initial_trees == 0) return compute_with_degree_order(graph);
  if (options.discarded_max_buckets >= options.counter_buckets)
    throw std::invalid_argument(
        "SamPG must retain at least one counter bucket");
  if (options.counter_buckets > kMaxCounterBuckets)
    throw std::invalid_argument(
        "SamPG: counter_buckets exceeds the supported maximum");
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

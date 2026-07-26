#include "pruned_labeling.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <numeric>
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

void add_tree_scores(const SampleTree& tree,
                     std::vector<std::vector<Score>>& counters) {
  for (VertexId v : tree.members) {
    if (tree.storage.alive(v)) {
      counters[v][tree.bucket] += tree.storage.subtree_of(v);
    }
  }
}

std::size_t remove_subtree(SampleTree& tree, VertexId hub,
                           std::vector<std::vector<Score>>& counters,
                           bool& downgraded) {
  downgraded = false;
  auto& storage = tree.storage;
  if (!storage.alive(hub)) return 0;
  const Score removed = storage.subtree_of(hub);
  VertexId ancestor = storage.parent_of(hub);
  while (ancestor != kInvalidVertex && storage.alive(ancestor)) {
    counters[ancestor][tree.bucket] -= removed;
    storage.set_subtree(ancestor, storage.subtree_of(ancestor) - removed);
    ancestor = storage.parent_of(ancestor);
  }
  std::vector<VertexId> stack{hub};
  while (!stack.empty()) {
    const VertexId v = stack.back();
    stack.pop_back();
    if (!storage.alive(v)) continue;
    counters[v][tree.bucket] -= storage.subtree_of(v);
    const std::vector<VertexId> children = storage.children_of(v);
    storage.deactivate(v);
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
void retire_remaining(SampleTree& tree,
                      std::vector<std::vector<Score>>& counters,
                      std::size_t& live_vertices) {
  for (VertexId v : tree.members) {
    if (!tree.storage.alive(v)) continue;
    const VertexId parent = tree.storage.parent_of(v);
    // Skip anything whose parent is still alive: it'll be swept up when its
    // (still-alive) ancestor's fragment is removed below, so removing it
    // here too would double-subtract its contribution from `counters`.
    if (parent != kInvalidVertex && tree.storage.alive(parent)) continue;
    bool downgraded = false;
    live_vertices -= remove_subtree(tree, v, counters, downgraded);
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

  std::vector<SampleTree> trees;
  trees.reserve(initial + n / 8 + 1);
  // Retired trees (remaining == 0, see the erase-remove below) are parked
  // here instead of being destroyed, so growing a new tree can rewind one
  // of these in place (SampleTree::reset()) rather than default-
  // constructing a brand new SampleTree(n) -- which would mean freshly
  // allocating its four O(n) dense arrays every single time a new sample
  // tree is grown (Section 3.3 / Appendix A.2: this happens repeatedly
  // throughout the run, not just for the `initial` trees).
  std::vector<SampleTree> tree_pool;
  auto acquire_tree = [&]() -> SampleTree {
    if (!tree_pool.empty()) {
      SampleTree tree = std::move(tree_pool.back());
      tree_pool.pop_back();
      return tree;
    }
    return SampleTree(n);
  };
  std::vector<std::vector<Score>> counters(n, std::vector<Score>(buckets, 0));
  std::uint64_t tree_work = 0, label_work = 0;
  const std::size_t threads = std::max<std::size_t>(1, options.num_threads);
  std::size_t live_vertices = 0;
  std::vector<TreeScratch> scratch_pool(threads);
  auto grow_batch = [&](const std::vector<VertexId>& batch_roots,
                        std::size_t first_bucket) {
    using Built = std::pair<SampleTree, std::uint64_t>;
    std::vector<Built> built;
    built.reserve(batch_roots.size());
    if (threads == 1 || batch_roots.size() == 1) {
      for (std::size_t i = 0; i < batch_roots.size(); ++i) {
        std::uint64_t work = 0;
        SampleTree tree = acquire_tree();
        build_sample_tree(graph, batch_roots[i], labels,
                          (first_bucket + i) % buckets, work, scratch_pool[0],
                          tree);
        built.emplace_back(std::move(tree), work);
      }
    } else {
      // tree_pool is only ever touched from the main thread: every lane's
      // tree is acquired here, before any std::async task is launched, and
      // handed to that lane by reference -- each async task only ever
      // touches its own lane_trees[i], so there is no concurrent access to
      // the pool itself or to any other lane's tree.
      std::vector<SampleTree> lane_trees;
      lane_trees.reserve(batch_roots.size());
      for (std::size_t i = 0; i < batch_roots.size(); ++i)
        lane_trees.push_back(acquire_tree());
      std::vector<std::future<std::uint64_t>> futures;
      futures.reserve(batch_roots.size());
      for (std::size_t i = 0; i < batch_roots.size(); ++i) {
        const VertexId root = batch_roots[i];
        const std::size_t bucket = (first_bucket + i) % buckets;
        TreeScratch& lane = scratch_pool[i];
        SampleTree& tree = lane_trees[i];
        futures.emplace_back(std::async(
            std::launch::async, [&graph, &labels, &lane, &tree, root, bucket] {
              std::uint64_t work = 0;
              build_sample_tree(graph, root, labels, bucket, work, lane, tree);
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
      add_tree_scores(item.first, counters);
      live_vertices += item.first.remaining;
      if (!item.first.storage.is_dense()) ++stats.sparse_downgrades;
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

  std::vector<VertexId> order;
  order.reserve(n);
  std::vector<std::uint8_t> selected(n, 0);
  std::vector<Distance> root_out(n, kInfinity), root_in(n, kInfinity),
      distance(n, kInfinity);
  std::vector<VertexId> lookup_touched, search_touched;
  std::vector<Score> priority_scratch;
  std::size_t next_root = initial;
  const std::size_t factor = 10 * std::max<std::size_t>(1, initial);
  const std::size_t max_tree_vertices =
      n > std::numeric_limits<std::size_t>::max() / factor
          ? std::numeric_limits<std::size_t>::max()
          : factor * n;

  for (std::size_t rank = 0; rank < n; ++rank) {
    VertexId best = kInvalidVertex;
    std::pair<Score, Score> best_priority{0, 0};
    std::size_t best_degree = 0;
    for (VertexId v = 0; v < n; ++v)
      if (!selected[v]) {
        const auto p = priority(counters[v], options.discarded_max_buckets,
                                priority_scratch);
        const std::size_t degree =
            graph.adjacency()[v].size() + graph.reverse_adjacency()[v].size();
        if (best == kInvalidVertex || p > best_priority ||
            (p == best_priority && degree > best_degree)) {
          best = v;
          best_priority = p;
          best_degree = degree;
        }
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
    for (auto& tree : trees) {
      bool downgraded = false;
      live_vertices -= remove_subtree(tree, best, counters, downgraded);
      if (downgraded) ++stats.sparse_downgrades;
      // See retire_remaining()'s comment: don't let a tree that has
      // dwindled to a few stragglers stick around indefinitely just
      // because none of its survivors happen to get selected next.
      if (tree.remaining > 0 && tree.remaining <= options.min_tree_vertices)
        retire_remaining(tree, counters, live_vertices);
    }
    trees.erase(std::remove_if(trees.begin(), trees.end(),
                               [&](SampleTree& tree) {
                                 if (tree.remaining != 0) return false;
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

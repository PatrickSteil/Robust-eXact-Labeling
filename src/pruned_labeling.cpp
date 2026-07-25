#include "pruned_labeling.h"

#include <algorithm>
#include <chrono>
#include <functional>
#include <future>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <random>
#include <stdexcept>
#include <thread>

#include "parallel_for.h"

namespace rxl {
namespace {
using QueueItem = std::pair<Distance, VertexId>;
using PQ = std::priority_queue<QueueItem, std::vector<QueueItem>,
                               std::greater<QueueItem>>;

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
  PQ queue;
  distance[root] = 0;
  touched.push_back(root);
  queue.emplace(0, root);
  std::uint64_t work = 0;
  while (!queue.empty()) {
    const auto [du, u] = queue.top();
    queue.pop();
    if (du != distance[u]) continue;
    const Label& query = forward ? labels[u].backward : labels[u].forward;
    ++work;
    if (covered(query, root_distance, du)) continue;
    Label& output = forward ? labels[u].backward : labels[u].forward;
    // The graph traversal (queue/distance, adjacency lookups) stays in
    // original-vertex-id space, since the graph itself is never physically
    // reordered. Only the value recorded in the label is the hub's rank:
    // hub_id is root's processing rank, assigned by the caller. This is
    // what makes hub IDs inside labels small/clustered for important
    // (early-processed) vertices without renumbering any vertex.
    output.push_back(hub_id, du);
    work += graph[u].size();
    for (const auto& [v, weight] : graph[u]) {
      const std::uint64_t candidate = std::uint64_t(du) + weight;
      if (candidate < distance[v] && candidate < kInfinity) {
        if (distance[v] == kInfinity) touched.push_back(v);
        distance[v] = static_cast<Distance>(candidate);
        queue.emplace(distance[v], v);
      }
    }
  }
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
  // root_out/root_in are indexed by hub id (i.e. by rank), matching how
  // hub entries are now stored in labels[*].forward/backward.
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
  std::size_t bucket = 0;
  std::vector<VertexId> parent;
  std::vector<std::vector<VertexId>> children;
  std::vector<std::uint8_t> alive;
  std::vector<Score> subtree;
  // Vertices alive at construction time. Iterating this (instead of the
  // dense 0..n range) lets score seeding touch only what the search
  // actually reached.
  std::vector<VertexId> members;
  // Mirrors "how many entries of `alive` are still 1", updated incrementally
  // by remove_subtree so tree exhaustion is an O(1) check instead of an
  // O(n) scan of `alive`.
  std::size_t remaining = 0;
};

// Per-lane scratch reused across sample-tree builds so each build doesn't
// have to allocate and fully re-initialize O(n) buffers from scratch. Only
// the entries actually touched by a search are reset afterwards, the same
// touched-list trick pruned_dijkstra already uses for `distance`.
struct TreeScratch {
  std::vector<Distance> root_lookup;
  std::vector<Distance> distance;
  std::vector<VertexId> lookup_touched;
  std::vector<VertexId> distance_touched;
};

// A sampled tree represents currently uncovered source-target paths. The tree
// is grown with the current partial labels and pruned exactly as PL would be.
SampleTree build_sample_tree(const Graph& graph, VertexId root,
                             const HubLabels& labels, std::size_t bucket,
                             std::uint64_t& work, TreeScratch& scratch) {
  const std::size_t n = graph.num_vertices();
  if (scratch.root_lookup.size() != n) {
    scratch.root_lookup.assign(n, kInfinity);
    scratch.distance.assign(n, kInfinity);
  }
  auto& root_lookup = scratch.root_lookup;
  auto& distance = scratch.distance;
  auto& lookup_touched = scratch.lookup_touched;
  auto& distance_touched = scratch.distance_touched;

  SampleTree tree;
  tree.bucket = bucket;
  tree.parent.assign(n, kInvalidVertex);
  tree.children.resize(n);
  tree.alive.assign(n, 0);
  tree.subtree.assign(n, 0);
  for (const auto& [hub, d] : labels[root].forward) {
    root_lookup[hub] = d;
    lookup_touched.push_back(hub);
  }
  PQ queue;
  distance[root] = 0;
  distance_touched.push_back(root);
  queue.emplace(0, root);
  std::vector<VertexId> settled;
  while (!queue.empty()) {
    const auto [du, u] = queue.top();
    queue.pop();
    if (du != distance[u]) continue;
    ++work;
    if (covered(labels[u].backward, root_lookup, du)) continue;
    tree.alive[u] = 1;
    settled.push_back(u);
    work += graph.adjacency()[u].size();
    for (const auto& [v, w] : graph.adjacency()[u]) {
      const std::uint64_t candidate = std::uint64_t(du) + w;
      if (candidate < distance[v] && candidate < kInfinity) {
        if (distance[v] == kInfinity) distance_touched.push_back(v);
        distance[v] = static_cast<Distance>(candidate);
        tree.parent[v] = u;
        queue.emplace(distance[v], v);
      } else if (candidate == distance[v] && u < tree.parent[v]) {
        // Deterministic shortest-path-tree tie breaking.
        tree.parent[v] = u;
      }
    }
  }
  for (VertexId v : settled) {
    const VertexId parent = tree.parent[v];
    if (parent != kInvalidVertex && tree.alive[parent])
      tree.children[parent].push_back(v);
  }
  // Distances are nondecreasing in settled order, so children occur after
  // parents; reverse order computes descendant counts.
  for (auto it = settled.rbegin(); it != settled.rend(); ++it) {
    const VertexId v = *it;
    tree.subtree[v] += 1;
    const VertexId parent = tree.parent[v];
    if (parent != kInvalidVertex && tree.alive[parent])
      tree.subtree[parent] += tree.subtree[v];
  }
  tree.remaining = settled.size();
  tree.members = std::move(settled);

  for (VertexId v : distance_touched) distance[v] = kInfinity;
  distance_touched.clear();
  for (VertexId hub : lookup_touched) root_lookup[hub] = kInfinity;
  lookup_touched.clear();
  return tree;
}

void add_tree_scores(const SampleTree& tree,
                     std::vector<std::vector<Score>>& counters) {
  // Only members reached by the search need visiting, not every vertex.
  for (VertexId v : tree.members)
    if (tree.alive[v]) counters[v][tree.bucket] += tree.subtree[v];
}

// Remove the subtree rooted at hub. This deletes precisely the sampled paths
// newly covered when hub is selected. Ancestors lose the removed descendants.
// Returns how many vertices were removed, so callers can keep a running
// live-vertex total without rescanning `alive`.
std::size_t remove_subtree(SampleTree& tree, VertexId hub,
                           std::vector<std::vector<Score>>& counters) {
  if (!tree.alive[hub]) return 0;
  const Score removed = tree.subtree[hub];
  VertexId ancestor = tree.parent[hub];
  while (ancestor != kInvalidVertex && tree.alive[ancestor]) {
    counters[ancestor][tree.bucket] -= removed;
    tree.subtree[ancestor] -= removed;
    ancestor = tree.parent[ancestor];
  }
  std::vector<VertexId> stack{hub};
  while (!stack.empty()) {
    const VertexId v = stack.back();
    stack.pop_back();
    if (!tree.alive[v]) continue;
    counters[v][tree.bucket] -= tree.subtree[v];
    tree.alive[v] = 0;
    for (VertexId child : tree.children[v]) stack.push_back(child);
  }
  tree.remaining -= static_cast<std::size_t>(removed);
  return static_cast<std::size_t>(removed);
}

// Sums `counter` and its "robust" tail (after discarding the `discard`
// largest buckets) using a caller-supplied scratch buffer, instead of
// allocating and sorting a fresh vector on every call. This is by far the
// hottest call in the ordering loop (once per still-unselected vertex, per
// rank), so avoiding the allocation matters far more than the sort itself.
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
  std::vector<std::vector<Score>> counters(n, std::vector<Score>(buckets, 0));
  std::uint64_t tree_work = 0, label_work = 0;
  const std::size_t threads = std::max<std::size_t>(1, options.num_threads);
  // Running total of alive vertices across all live trees, maintained
  // incrementally by grow_batch (on insertion) and remove_subtree's return
  // value (on removal) instead of being recomputed with std::count every
  // rank, which used to cost O(n) per live tree per rank.
  std::size_t live_vertices = 0;
  // One reusable scratch buffer per concurrent lane; a batch never has more
  // entries than `threads`, so indexing by position in the batch is safe.
  std::vector<TreeScratch> scratch_pool(threads);
  auto grow_batch = [&](const std::vector<VertexId>& batch_roots,
                        std::size_t first_bucket) {
    using Built = std::pair<SampleTree, std::uint64_t>;
    std::vector<Built> built(batch_roots.size());
    if (threads == 1 || batch_roots.size() == 1) {
      for (std::size_t i = 0; i < batch_roots.size(); ++i) {
        std::uint64_t work = 0;
        built[i].first = build_sample_tree(graph, batch_roots[i], labels,
                                           (first_bucket + i) % buckets, work,
                                           scratch_pool[0]);
        built[i].second = work;
      }
    } else {
      std::vector<std::future<Built>> futures;
      futures.reserve(batch_roots.size());
      for (std::size_t i = 0; i < batch_roots.size(); ++i) {
        const VertexId root = batch_roots[i];
        const std::size_t bucket = (first_bucket + i) % buckets;
        TreeScratch& lane = scratch_pool[i];
        futures.emplace_back(std::async(std::launch::async, [&, root, bucket] {
          std::uint64_t work = 0;
          auto tree =
              build_sample_tree(graph, root, labels, bucket, work, lane);
          return Built{std::move(tree), work};
        }));
      }
      for (std::size_t i = 0; i < futures.size(); ++i)
        built[i] = futures[i].get();
    }
    for (auto& item : built) {
      tree_work += item.second;
      stats.sampling_work += item.second;
      add_tree_scores(item.first, counters);
      live_vertices += item.first.remaining;
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
  // The paper treats the initial k trees as free in balancing counters.
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
            (p == best_priority && degree > best_degree) ||
            (p == best_priority && degree == best_degree && v < best)) {
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
      live_vertices -= remove_subtree(tree, best, counters);
    }
    trees.erase(std::remove_if(
                    trees.begin(), trees.end(),
                    [](const SampleTree& tree) { return tree.remaining == 0; }),
                trees.end());

    std::size_t attempts = 0;
    while ((trees.size() < buckets || tree_work <= label_work) &&
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

LabelingResult finish(HubLabels labels, std::vector<VertexId> order,
                      std::size_t threads) {
  (void)threads;
  // No sorting step needed here anymore. Hub ids appended to a label are
  // already strictly increasing by construction: add_hub()/pruned_dijkstra()
  // always push_back() with hub_id equal to the *current* rank, and ranks
  // only ever increase across the single pass over 0..n-1 that drives both
  // compute() and compute_with_degree_order(). DeltaLabel::push_back() also
  // enforces this at build time (it throws on an out-of-order hub id), so
  // this invariant is checked, not just assumed.
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
  return finish(std::move(labels), std::move(order),
                std::max<std::size_t>(1, std::thread::hardware_concurrency()));
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
  auto result = finish(std::move(labels), std::move(order),
                       std::max<std::size_t>(1, options.num_threads));
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

#include "pruned_labeling.h"
#include <algorithm>
#include <chrono>
#include <future>
#include <iostream>
#include <functional>
#include <numeric>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>

namespace rxl {
namespace {
using QueueItem = std::pair<Distance, VertexId>;

bool covered(const Label &label, const std::vector<Distance> &root_distance,
             Distance search_distance) {
  for (const auto &[hub, target_distance] : label) {
    const Distance root = root_distance[hub];
    if (root != kInfinity && std::uint64_t(root) + target_distance <= search_distance)
      return true;
  }
  return false;
}

std::uint64_t pruned_dijkstra(const AdjacencyList &graph, VertexId root,
    const std::vector<Distance> &root_distance, HubLabels &labels,
    bool forward, std::vector<Distance> &distance,
    std::vector<VertexId> &touched) {
  std::priority_queue<QueueItem, std::vector<QueueItem>,
                      std::greater<QueueItem>> queue;
  distance[root] = 0; touched.push_back(root); queue.emplace(0, root);
  std::uint64_t work = 0;
  while (!queue.empty()) {
    const auto [du, u] = queue.top(); queue.pop();
    if (du != distance[u]) continue;
    const Label &query = forward ? labels[u].backward : labels[u].forward;
    ++work;
    if (covered(query, root_distance, du)) continue;
    Label &output = forward ? labels[u].backward : labels[u].forward;
    output.emplace_back(root, du);
    work += graph[u].size();
    for (const auto &[v, weight] : graph[u]) {
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

std::uint64_t add_hub(const Graph &graph, VertexId root, HubLabels &labels,
    std::vector<Distance> &root_out, std::vector<Distance> &root_in,
    std::vector<Distance> &distance, std::vector<VertexId> &lookup_touched,
    std::vector<VertexId> &search_touched) {
  for (const auto &[hub,d] : labels[root].forward) {
    root_out[hub] = d; lookup_touched.push_back(hub);
  }
  for (const auto &[hub,d] : labels[root].backward) {
    root_in[hub] = d; lookup_touched.push_back(hub);
  }
  std::uint64_t work = pruned_dijkstra(graph.adjacency(), root, root_out,
      labels, true, distance, search_touched);
  work += pruned_dijkstra(graph.reverse_adjacency(), root, root_in,
      labels, false, distance, search_touched);
  for (VertexId hub : lookup_touched) {
    root_out[hub] = kInfinity; root_in[hub] = kInfinity;
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
};

// A sampled tree represents currently uncovered source-target paths. The tree
// is grown with the current partial labels and pruned exactly as PL would be.
SampleTree build_sample_tree(const Graph &graph, VertexId root,
    const HubLabels &labels, std::size_t bucket, std::uint64_t &work) {
  const std::size_t n = graph.num_vertices();
  SampleTree tree;
  tree.bucket = bucket;
  tree.parent.assign(n, kInvalidVertex);
  tree.children.resize(n);
  tree.alive.assign(n, 0);
  tree.subtree.assign(n, 0);
  std::vector<Distance> root_lookup(n, kInfinity), distance(n, kInfinity);
  for (const auto &[hub,d] : labels[root].forward) root_lookup[hub] = d;
  std::priority_queue<QueueItem, std::vector<QueueItem>,
                      std::greater<QueueItem>> queue;
  distance[root] = 0; queue.emplace(0, root);
  std::vector<VertexId> settled;
  while (!queue.empty()) {
    const auto [du,u] = queue.top(); queue.pop();
    if (du != distance[u]) continue;
    ++work;
    if (covered(labels[u].backward, root_lookup, du)) continue;
    tree.alive[u] = 1; settled.push_back(u);
    work += graph.adjacency()[u].size();
    for (const auto &[v,w] : graph.adjacency()[u]) {
      const std::uint64_t candidate = std::uint64_t(du) + w;
      if (candidate < distance[v] && candidate < kInfinity) {
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
  return tree;
}

void add_tree_scores(const SampleTree &tree,
    std::vector<std::vector<Score>> &counters) {
  for (VertexId v = 0; v < tree.alive.size(); ++v)
    if (tree.alive[v]) counters[v][tree.bucket] += tree.subtree[v];
}

// Remove the subtree rooted at hub. This deletes precisely the sampled paths
// newly covered when hub is selected. Ancestors lose the removed descendants.
void remove_subtree(SampleTree &tree, VertexId hub,
    std::vector<std::vector<Score>> &counters) {
  if (!tree.alive[hub]) return;
  const Score removed = tree.subtree[hub];
  VertexId ancestor = tree.parent[hub];
  while (ancestor != kInvalidVertex && tree.alive[ancestor]) {
    counters[ancestor][tree.bucket] -= removed;
    tree.subtree[ancestor] -= removed;
    ancestor = tree.parent[ancestor];
  }
  std::vector<VertexId> stack{hub};
  while (!stack.empty()) {
    const VertexId v = stack.back(); stack.pop_back();
    if (!tree.alive[v]) continue;
    counters[v][tree.bucket] -= tree.subtree[v];
    tree.alive[v] = 0;
    for (VertexId child : tree.children[v]) stack.push_back(child);
  }
}

std::pair<Score,Score> priority(const std::vector<Score> &counter,
                               std::size_t discard) {
  std::vector<Score> sorted = counter;
  std::sort(sorted.begin(), sorted.end(), std::greater<Score>());
  discard = std::min(discard, sorted.size());
  Score robust = 0, total = 0;
  for (Score value : counter) total += value;
  for (std::size_t i = discard; i < sorted.size(); ++i) robust += sorted[i];
  return {robust, total};
}

std::vector<VertexId> sampled_order(const Graph &graph,
    const SamplingOptions &options, HubLabels &labels, BuildStatistics &stats) {
  const std::size_t n = graph.num_vertices();
  const std::size_t buckets = std::max<std::size_t>(1, options.counter_buckets);
  const std::size_t initial = std::min(n, std::max(options.initial_trees, buckets));
  std::mt19937_64 random(options.random_seed);
  std::vector<VertexId> roots(n);
  std::iota(roots.begin(), roots.end(), VertexId{0});
  std::shuffle(roots.begin(), roots.end(), random);

  std::vector<SampleTree> trees;
  trees.reserve(initial + n / 8 + 1);
  std::vector<std::vector<Score>> counters(n, std::vector<Score>(buckets, 0));
  std::uint64_t tree_work = 0, label_work = 0;
  const std::size_t threads = std::max<std::size_t>(1, options.num_threads);
  auto grow_batch = [&](const std::vector<VertexId> &batch_roots,
                        std::size_t first_bucket) {
    using Built = std::pair<SampleTree,std::uint64_t>;
    std::vector<Built> built(batch_roots.size());
    if (threads == 1 || batch_roots.size() == 1) {
      for (std::size_t i=0;i<batch_roots.size();++i) {
        std::uint64_t work=0;
        built[i].first=build_sample_tree(graph,batch_roots[i],labels,
            (first_bucket+i)%buckets,work);
        built[i].second=work;
      }
    } else {
      std::vector<std::future<Built>> futures;
      futures.reserve(batch_roots.size());
      for (std::size_t i=0;i<batch_roots.size();++i) {
        const VertexId root=batch_roots[i]; const std::size_t bucket=(first_bucket+i)%buckets;
        futures.emplace_back(std::async(std::launch::async,[&,root,bucket] {
          std::uint64_t work=0;
          auto tree=build_sample_tree(graph,root,labels,bucket,work);
          return Built{std::move(tree),work};
        }));
      }
      for (std::size_t i=0;i<futures.size();++i) built[i]=futures[i].get();
    }
    for (auto &item:built) {
      tree_work += item.second; stats.sampling_work += item.second;
      add_tree_scores(item.first,counters);
      trees.push_back(std::move(item.first)); ++stats.sampled_trees;
    }
  };
  for (std::size_t begin=0;begin<initial;begin+=threads) {
    const std::size_t end=std::min(initial,begin+threads);
    grow_batch(std::vector<VertexId>(roots.begin()+begin,roots.begin()+end),trees.size()%buckets);
  }
  // The paper treats the initial k trees as free in balancing counters.
  tree_work = 0;

  std::vector<VertexId> order;
  order.reserve(n);
  std::vector<std::uint8_t> selected(n, 0);
  std::vector<Distance> root_out(n,kInfinity), root_in(n,kInfinity),
                        distance(n,kInfinity);
  std::vector<VertexId> lookup_touched, search_touched;
  std::size_t next_root = initial;
  const std::size_t factor = 10 * std::max<std::size_t>(1, initial);
  const std::size_t max_tree_vertices =
      n > std::numeric_limits<std::size_t>::max() / factor
          ? std::numeric_limits<std::size_t>::max() : factor * n;

  for (std::size_t rank = 0; rank < n; ++rank) {
    VertexId best = kInvalidVertex;
    std::pair<Score,Score> best_priority{0,0};
    std::size_t best_degree = 0;
    for (VertexId v = 0; v < n; ++v) if (!selected[v]) {
      const auto p = priority(counters[v], options.discarded_max_buckets);
      const std::size_t degree = graph.adjacency()[v].size() +
                                 graph.reverse_adjacency()[v].size();
      if (best == kInvalidVertex || p > best_priority ||
          (p == best_priority && degree > best_degree) ||
          (p == best_priority && degree == best_degree && v < best)) {
        best = v; best_priority = p; best_degree = degree;
      }
    }
    selected[best] = 1; order.push_back(best);
    label_work += add_hub(graph, best, labels, root_out, root_in, distance,
                          lookup_touched, search_touched);
    if (options.verbose && (rank < 10 || (rank + 1) % 1000 == 0 || rank + 1 == n))
      std::cerr << "[rxl] rank " << (rank + 1) << '/' << n
                << ": selected vertex " << best << ", live trees "
                << trees.size() << "\n";
    for (auto &tree : trees) remove_subtree(tree, best, counters);
    trees.erase(std::remove_if(trees.begin(), trees.end(), [](const SampleTree &tree) {
      return std::none_of(tree.alive.begin(), tree.alive.end(),
                          [](std::uint8_t value) { return value != 0; });
    }), trees.end());

    // SamPG replenishes the sample as labeling work overtakes tree work and
    // keeps at least c live trees. Random roots may be reused after one pass.
    std::size_t live_vertices = 0;
    for (const auto &tree : trees)
      live_vertices += std::count(tree.alive.begin(), tree.alive.end(), 1);
    std::size_t attempts = 0;
    while ((trees.size() < buckets || tree_work <= label_work) &&
           live_vertices < max_tree_vertices && attempts < 2 * n) {
      std::vector<VertexId> batch;
      // Each batch sees the same immutable graph and partial-label snapshot;
      // only the subsequent score merge is serialized.
      batch.reserve(threads);
      while (batch.size() < threads && attempts < 2*n) {
        if (next_root == n) { std::shuffle(roots.begin(),roots.end(),random); next_root=0; }
        const VertexId root=roots[next_root++]; ++attempts;
        if (!selected[root]) batch.push_back(root);
      }
      if (batch.empty()) break;
      const std::size_t old_size=trees.size();
      grow_batch(batch,old_size%buckets);
      for (std::size_t i=old_size;i<trees.size();++i)
        live_vertices += std::count(trees[i].alive.begin(),trees[i].alive.end(),1);
    }
    stats.peak_live_trees=std::max(stats.peak_live_trees,trees.size());
  }
  stats.labeling_work = label_work;
  return order;
}

LabelingResult finish(HubLabels labels, std::vector<VertexId> order) {
  for (auto &label : labels) {
    std::sort(label.forward.begin(), label.forward.end());
    std::sort(label.backward.begin(), label.backward.end());
  }
  std::vector<Rank> ranks(order.size());
  for (Rank rank = 0; rank < order.size(); ++rank) ranks[order[rank]] = rank;
  return {std::move(labels), std::move(order), std::move(ranks), {}};
}
} // namespace

std::vector<VertexId> PrunedLabeling::degree_order(const Graph &graph) {
  std::vector<VertexId> order(graph.num_vertices());
  std::iota(order.begin(), order.end(), VertexId{0});
  std::sort(order.begin(), order.end(), [&](VertexId a, VertexId b) {
    const auto da = graph.adjacency()[a].size()+graph.reverse_adjacency()[a].size();
    const auto db = graph.adjacency()[b].size()+graph.reverse_adjacency()[b].size();
    return da != db ? da > db : a < b;
  });
  return order;
}

LabelingResult PrunedLabeling::compute_with_degree_order(const Graph &graph) {
  const std::size_t n = graph.num_vertices();
  HubLabels labels(n);
  auto order = degree_order(graph);
  std::vector<Distance> root_out(n,kInfinity),root_in(n,kInfinity),distance(n,kInfinity);
  std::vector<VertexId> lookup, touched;
  for (VertexId root : order)
    add_hub(graph, root, labels, root_out, root_in, distance, lookup, touched);
  return finish(std::move(labels), std::move(order));
}

LabelingResult PrunedLabeling::compute(const Graph &graph,
                                       const SamplingOptions &options) {
  if (options.initial_trees == 0) return compute_with_degree_order(graph);
  if (options.discarded_max_buckets >= options.counter_buckets)
    throw std::invalid_argument("SamPG must retain at least one counter bucket");
  HubLabels labels(graph.num_vertices());
  BuildStatistics statistics;
  const auto start=std::chrono::steady_clock::now();
  auto order = sampled_order(graph, options, labels, statistics);
  auto result=finish(std::move(labels), std::move(order));
  statistics.ordering_and_labeling_seconds=std::chrono::duration<double>(
      std::chrono::steady_clock::now()-start).count();
  result.statistics=statistics;
  return result;
}

void PrunedLabeling::reorder_labels_by_rank(LabelingResult &result) {
  const std::size_t n = result.labels.size();
  if (result.rank_to_vertex.size() != n || result.vertex_to_rank.size() != n)
    throw std::invalid_argument("Incomplete rank information");
  HubLabels reordered(n);
  for (VertexId old_owner = 0; old_owner < n; ++old_owner) {
    const VertexId new_owner = result.vertex_to_rank[old_owner];
    auto convert = [&](const Label &source, Label &target) {
      target.reserve(source.size());
      for (const auto &[old_hub,d] : source)
        target.emplace_back(result.vertex_to_rank[old_hub], d);
      std::sort(target.begin(), target.end());
    };
    convert(result.labels[old_owner].forward, reordered[new_owner].forward);
    convert(result.labels[old_owner].backward, reordered[new_owner].backward);
  }
  result.labels = std::move(reordered);
  result.rank_reordered = true;
  // Keep both permutations: they now map external/original IDs to the
  // rank-renumbered IDs used by labels and the reordered graph.
}
} // namespace rxl

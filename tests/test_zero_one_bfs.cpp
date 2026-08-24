#include <random>

#include "pruned_labeling.h"
#include "test_common.h"

namespace {

// A directed graph with every arc weight 0 or 1, including some 0-weight
// cycles and 0-weight shortcuts, to exercise both bucket directions of the
// deque frontier.
Graph make_zero_one_graph() {
  return make_graph(8, {{0, 1, 1},
                        {1, 2, 0},
                        {2, 0, 0},  // 0-weight cycle back to the root
                        {2, 3, 1},
                        {3, 4, 0},
                        {4, 3, 0},  // 0-weight cycle elsewhere
                        {1, 5, 1},
                        {5, 6, 0},
                        {6, 7, 1},
                        {7, 5, 0},
                        {0, 7, 1},
                        {3, 6, 0}});
}

Graph make_random_zero_one_graph(std::size_t n, std::size_t arcs,
                                 std::uint64_t seed) {
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<VertexId> vertex(0,
                                                 static_cast<VertexId>(n - 1));
  std::uniform_int_distribution<int> weight(0, 1);
  std::vector<std::tuple<VertexId, VertexId, Distance>> edges;
  edges.reserve(arcs);
  for (std::size_t i = 0; i < arcs; ++i)
    edges.emplace_back(vertex(rng), vertex(rng),
                       static_cast<Distance>(weight(rng)));
  return make_graph(n, edges);
}

}  // namespace

void test_zero_one_bfs_dijkstra_agreement() {
  auto g = make_zero_one_graph();
  CHECK(g.is_zero_one_weighted());
  std::cerr << "[dbg] fixed graph loop\n";
  for (VertexId s = 0; s < g.num_vertices(); ++s) {
    auto truth = Dijkstra::shortest_distances(g.adjacency(), s);
    auto fast = Dijkstra::shortest_distances_01bfs(g.adjacency(), s);
    CHECK(truth.size() == fast.size());
    for (VertexId t = 0; t < g.num_vertices(); ++t) CHECK(truth[t] == fast[t]);
  }

  std::cerr << "[dbg] random graph loop\n";
  auto random_graph = make_random_zero_one_graph(40, 200, 0xC0FFEEULL);
  CHECK(random_graph.is_zero_one_weighted());
  for (VertexId s = 0; s < random_graph.num_vertices(); ++s) {
    auto truth = Dijkstra::shortest_distances(random_graph.adjacency(), s);
    auto fast = Dijkstra::shortest_distances_01bfs(random_graph.adjacency(), s);
    for (VertexId t = 0; t < random_graph.num_vertices(); ++t)
      CHECK(truth[t] == fast[t]);
  }
  std::cerr << "[dbg] test_zero_one_bfs_dijkstra_agreement done\n";
}

void test_zero_one_bfs_index_matches_general_index() {
  auto g = make_zero_one_graph();

  std::cerr << "[dbg] degree_general\n";
  auto degree_general = PrunedLabeling::compute_with_degree_order(
      g, /*verbose=*/false, /*zero_one_bfs=*/false);
  std::cerr << "[dbg] degree_01\n";
  auto degree_01 = PrunedLabeling::compute_with_degree_order(
      g, /*verbose=*/false, /*zero_one_bfs=*/true);
  std::cerr << "[dbg] check_exact general\n";
  check_exact(g, degree_general.labels);
  std::cerr << "[dbg] check_exact 01\n";
  check_exact(g, degree_01.labels);

  SamplingOptions options;
  options.initial_trees = 4;
  options.counter_buckets = 2;
  options.discarded_max_buckets = 1;
  options.num_threads = 2;
  options.zero_one_bfs = true;
  std::cerr << "[dbg] sampled_01\n";
  auto sampled_01 = PrunedLabeling::compute(g, options);
  std::cerr << "[dbg] check_exact sampled_01\n";
  check_exact(g, sampled_01.labels);

  options.zero_one_bfs = false;
  std::cerr << "[dbg] sampled_general\n";
  auto sampled_general = PrunedLabeling::compute(g, options);
  std::cerr << "[dbg] check_exact sampled_general\n";
  check_exact(g, sampled_general.labels);
  std::cerr << "[dbg] done\n";
}

void test_zero_one_bfs_rejects_other_weights() {
  auto g = make_graph(3, {{0, 1, 2}, {1, 2, 1}});
  CHECK(!g.is_zero_one_weighted());

  bool threw = false;
  try {
    PrunedLabeling::compute_with_degree_order(g, /*verbose=*/false,
                                              /*zero_one_bfs=*/true);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  SamplingOptions options;
  options.zero_one_bfs = true;
  threw = false;
  try {
    PrunedLabeling::compute(g, options);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
}

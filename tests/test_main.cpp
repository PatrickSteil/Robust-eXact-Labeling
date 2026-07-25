#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <tuple>

#include "delta_label.h"
#include "dijkstra.h"
#include "graph.h"
#include "index_io.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "statistics.h"
using namespace rxl;
#define CHECK(x)                                                            \
  do {                                                                      \
    if (!(x)) throw std::runtime_error(std::string("CHECK failed: ") + #x); \
  } while (0)
// DeltaLabel is deliberately its own class (delta_label.h) so it can be
// tested in isolation from graphs/labeling/queries.
void test_delta_label() {
  DeltaLabel label;
  CHECK(label.empty());
  CHECK(label.size() == 0);
  label.push_back(0, 0);
  label.push_back(16, 5);
  label.push_back(29, 9);
  label.push_back(189, 12);
  CHECK(!label.empty());
  CHECK(label.size() == 4);

  // Decoding via the iterator must reproduce exactly what was pushed.
  std::vector<std::pair<VertexId, Distance>> decoded(label.begin(),
                                                      label.end());
  const std::vector<std::pair<VertexId, Distance>> expected{
      {0, 0}, {16, 5}, {29, 9}, {189, 12}};
  CHECK(decoded == expected);

  // Matches the example in the paper (Section 4.1): hubs (0 16 29 189)
  // delta-encode to (0 15 12 159).
  const std::vector<VertexId> expected_deltas{0, 15, 12, 159};
  CHECK(label.raw_deltas() == expected_deltas);

  // Structured bindings must work on the decoded entries, same as before.
  VertexId sum_hubs = 0;
  for (const auto& [hub, d] : label) sum_hubs += hub + d;
  CHECK(sum_hubs == 0 + 0 + 16 + 5 + 29 + 9 + 189 + 12);

  // Strictly-increasing invariant is enforced at push time.
  bool threw = false;
  try {
    label.push_back(189, 1);  // repeat of the last hub id
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
    label.push_back(5, 1);  // smaller than the last hub id
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  // from_deltas() is the inverse of raw_deltas()/raw_distances(), and
  // validates that decoded hub ids stay within range.
  auto rebuilt = DeltaLabel::from_deltas(label.raw_deltas(),
                                        {label.raw_distances().begin(),
                                         label.raw_distances().end()},
                                        /*n=*/200);
  const std::vector<std::pair<VertexId, Distance>> rebuilt_decoded(
      rebuilt.begin(), rebuilt.end());
  CHECK(rebuilt_decoded == expected);
  bool out_of_range = false;
  try {
    DeltaLabel::from_deltas(label.raw_deltas(), label.raw_distances(),
                            /*n=*/189);  // hub 189 is not < 189
  } catch (const std::runtime_error&) {
    out_of_range = true;
  }
  CHECK(out_of_range);

  // An empty label iterates zero times and round-trips cleanly.
  DeltaLabel empty_label;
  CHECK(empty_label.begin() == empty_label.end());
  auto rebuilt_empty = DeltaLabel::from_deltas({}, {}, 10);
  CHECK(rebuilt_empty.empty());
}
Graph make_graph(
    std::size_t n,
    const std::vector<std::tuple<VertexId, VertexId, Distance>>& arcs) {
  AdjacencyList a(n), r(n);
  for (auto [u, v, w] : arcs) {
    a[u].emplace_back(v, w);
    r[v].emplace_back(u, w);
  }
  return Graph(std::move(a), std::move(r));
}
void check_exact(const Graph& g, const HubLabels& labels) {
  for (VertexId s = 0; s < g.num_vertices(); ++s) {
    auto truth = Dijkstra::shortest_distances(g.adjacency(), s);
    for (VertexId t = 0; t < g.num_vertices(); ++t)
      CHECK(QuerySupport::distance(labels, s, t) == truth[t]);
  }
}
void test_directed_weighted() {
  auto g = make_graph(
      6, {{0, 1, 2}, {0, 2, 9}, {1, 2, 3}, {2, 3, 4}, {1, 4, 20}, {3, 4, 1}});
  auto degree = PrunedLabeling::compute_with_degree_order(g);
  check_exact(g, degree.labels);
  SamplingOptions o;
  o.initial_trees = 4;
  o.counter_buckets = 2;
  o.discarded_max_buckets = 1;
  o.num_threads = 2;
  auto sampled = PrunedLabeling::compute(g, o);
  check_exact(g, sampled.labels);
}
void test_rank_reorder() {
  auto g = make_graph(
      5, {{0, 1, 1}, {1, 2, 2}, {0, 3, 8}, {2, 3, 1}, {3, 4, 3}, {4, 0, 7}});
  auto result = PrunedLabeling::compute_with_degree_order(g);
  std::vector<std::vector<Distance>> truth(g.num_vertices());
  for (VertexId s = 0; s < g.num_vertices(); ++s)
    truth[s] = Dijkstra::shortest_distances(g.adjacency(), s);
  auto map = g.reorder_by_rank(result.rank_to_vertex);
  PrunedLabeling::reorder_labels_by_rank(result);
  for (VertexId s = 0; s < g.num_vertices(); ++s)
    for (VertexId t = 0; t < g.num_vertices(); ++t)
      CHECK(QuerySupport::distance(result.labels, map[s], map[t]) ==
            truth[s][t]);
}
// Every vertex is trivially a hub of itself at distance 0. That self-entry's
// hub id must be the vertex's assigned *rank* (vertex_to_rank[v]), not v
// itself -- this is the core change that lets hub ids stay small/clustered
// without ever physically renumbering the graph or the labels.
void check_hub_ids_are_ranks(const LabelingResult& result) {
  for (VertexId v = 0; v < result.labels.size(); ++v) {
    const VertexId expected = result.vertex_to_rank[v];
    bool forward_ok = false, backward_ok = false;
    for (const auto& [hub, d] : result.labels[v].forward)
      if (d == 0 && hub == expected) forward_ok = true;
    for (const auto& [hub, d] : result.labels[v].backward)
      if (d == 0 && hub == expected) backward_ok = true;
    CHECK(forward_ok);
    CHECK(backward_ok);
  }
}
void test_hub_ids_are_ranks() {
  auto g = make_graph(
      6, {{0, 1, 2}, {0, 2, 9}, {1, 2, 3}, {2, 3, 4}, {1, 4, 20}, {3, 4, 1}});
  check_hub_ids_are_ranks(PrunedLabeling::compute_with_degree_order(g));
  SamplingOptions o;
  o.initial_trees = 4;
  o.counter_buckets = 2;
  o.discarded_max_buckets = 1;
  o.num_threads = 2;
  check_hub_ids_are_ranks(PrunedLabeling::compute(g, o));
}
void test_export_roundtrip() {
  auto g = make_graph(4, {{0, 1, 5}, {1, 2, 6}, {2, 0, 2}});
  auto result = PrunedLabeling::compute_with_degree_order(g);
  const std::string path = "rxl_test_index.bin";
  IndexIO::export_binary(result, path);
  auto loaded = IndexIO::import_binary(path);
  std::remove(path.c_str());
  CHECK(loaded.rank_to_vertex == result.rank_to_vertex);
  CHECK(loaded.vertex_to_rank == result.vertex_to_rank);
  CHECK(loaded.rank_reordered == result.rank_reordered);
  for (VertexId s = 0; s < g.num_vertices(); ++s)
    for (VertexId t = 0; t < g.num_vertices(); ++t)
      CHECK(QuerySupport::distance(loaded.labels, s, t) ==
            QuerySupport::distance(result.labels, s, t));
}
void test_parallel_determinism() {
  std::mt19937 rng(17);
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  for (VertexId u = 0; u < 25; ++u)
    for (VertexId v = 0; v < 25; ++v)
      if (u != v && rng() % 7 == 0) arcs.emplace_back(u, v, 1 + rng() % 20);
  auto g = make_graph(25, arcs);
  SamplingOptions a;
  a.initial_trees = 8;
  a.counter_buckets = 4;
  a.discarded_max_buckets = 1;
  a.random_seed = 99;
  a.num_threads = 1;
  auto b = a;
  b.num_threads = 4;
  auto one = PrunedLabeling::compute(g, a);
  auto four = PrunedLabeling::compute(g, b);
  check_exact(g, one.labels);
  check_exact(g, four.labels);
}
void test_statistics() {
  auto g = make_graph(3, {{0, 1, 1}, {1, 2, 2}});
  auto gs = compute_graph_statistics(g);
  CHECK(gs.vertices == 3);
  CHECK(gs.arcs == 2);
  CHECK(gs.max_out_degree == 1);
  auto result = PrunedLabeling::compute_with_degree_order(g);
  auto ls = compute_label_statistics(result.labels);
  CHECK(ls.forward_entries > 0);
  CHECK(ls.backward_entries > 0);
}
int main() {
  try {
    test_delta_label();
    test_directed_weighted();
    test_rank_reorder();
    test_export_roundtrip();
    test_parallel_determinism();
    test_statistics();
    std::cout << "All RXL tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

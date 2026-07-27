#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <random>
#include <stdexcept>
#include <tuple>

#include "abstract_dijkstra.h"
#include "delta_label.h"
#include "dijkstra.h"
#include "graph.h"
#include "index_io.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "statistics.h"
using namespace rxl;
#define CHECK(x)                                                               \
  do {                                                                         \
    if (!(x))                                                                  \
      throw std::runtime_error(std::string("CHECK failed: ") + #x);            \
  } while (0)

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

  std::vector<std::pair<VertexId, Distance>> decoded(label.begin(),
                                                     label.end());
  const std::vector<std::pair<VertexId, Distance>> expected{
      {0, 0}, {16, 5}, {29, 9}, {189, 12}};
  CHECK(decoded == expected);

  const std::vector<VertexId> expected_deltas{0, 15, 12, 159};
  CHECK(label.raw_deltas() == expected_deltas);

  VertexId sum_hubs = 0;
  for (const auto &[hub, d] : label)
    sum_hubs += hub + d;
  CHECK(sum_hubs == 0 + 0 + 16 + 5 + 29 + 9 + 189 + 12);

  bool threw = false;
  try {
    label.push_back(189, 1); // repeat of the last hub id
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
    label.push_back(5, 1); // smaller than the last hub id
  } catch (const std::invalid_argument &) {
    threw = true;
  }
  CHECK(threw);

  const std::vector<Distance> flat_distances = label.raw_distances();
  auto rebuilt = DeltaLabel::from_deltas(label.raw_deltas(), flat_distances,
                                         /*n=*/200);
  const std::vector<std::pair<VertexId, Distance>> rebuilt_decoded(
      rebuilt.begin(), rebuilt.end());
  CHECK(rebuilt_decoded == expected);
  bool out_of_range = false;
  try {
    DeltaLabel::from_deltas(label.raw_deltas(), flat_distances,
                            /*n=*/189); // hub 189 is not < 189
  } catch (const std::runtime_error &) {
    out_of_range = true;
  }
  CHECK(out_of_range);

  DeltaLabel empty_label;
  CHECK(empty_label.begin() == empty_label.end());
  auto rebuilt_empty = DeltaLabel::from_deltas({}, {}, 10);
  CHECK(rebuilt_empty.empty());
}
Graph make_graph(
    std::size_t n,
    const std::vector<std::tuple<VertexId, VertexId, Distance>> &arcs) {
  AdjacencyList a(n), r(n);
  for (auto [u, v, w] : arcs) {
    a[u].emplace_back(v, w);
    r[v].emplace_back(u, w);
  }
  return Graph(std::move(a), std::move(r));
}
void check_exact(const Graph &g, const HubLabels &labels) {
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

void check_hub_ids_are_ranks(const LabelingResult &result) {
  for (VertexId v = 0; v < result.labels.size(); ++v) {
    const VertexId expected = result.vertex_to_rank[v];
    bool forward_ok = false, backward_ok = false;
    for (const auto &[hub, d] : result.labels[v].forward)
      if (d == 0 && hub == expected)
        forward_ok = true;
    for (const auto &[hub, d] : result.labels[v].backward)
      if (d == 0 && hub == expected)
        backward_ok = true;
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

void test_default_encoding_is_two_block() {
  auto g = make_graph(4, {{0, 1, 5}, {1, 2, 6}, {2, 0, 2}});
  auto result = PrunedLabeling::compute_with_degree_order(g);
  const std::string implicit_path = "rxl_test_encoding_default.bin";
  const std::string explicit_path = "rxl_test_encoding_two_block.bin";
  IndexIO::export_binary(result, implicit_path);
  IndexIO::export_binary(result, explicit_path, LabelEncoding::TwoBlockDelta);
  std::ifstream fa(implicit_path, std::ios::binary);
  std::ifstream fb(explicit_path, std::ios::binary);
  const std::vector<char> bytes_a((std::istreambuf_iterator<char>(fa)),
                                  std::istreambuf_iterator<char>());
  const std::vector<char> bytes_b((std::istreambuf_iterator<char>(fb)),
                                  std::istreambuf_iterator<char>());
  std::remove(implicit_path.c_str());
  std::remove(explicit_path.c_str());
  CHECK(bytes_a == bytes_b);
}

void test_varint_encoding_roundtrip() {
  std::mt19937 rng(4242);
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  for (VertexId u = 0; u < 60; ++u)
    for (VertexId v = 0; v < 60; ++v)
      if (u != v && rng() % 6 == 0)
        arcs.emplace_back(u, v, 1 + rng() % 500);
  auto g = make_graph(60, arcs);
  SamplingOptions o;
  o.initial_trees = 16;
  o.counter_buckets = 4;
  o.discarded_max_buckets = 1;
  o.random_seed = 4242;
  auto result = PrunedLabeling::compute(g, o);
  const std::string path = "rxl_test_varint_roundtrip.bin";
  IndexIO::export_binary(result, path, LabelEncoding::Varint);
  auto loaded = IndexIO::import_binary(path);
  std::remove(path.c_str());
  CHECK(loaded.rank_to_vertex == result.rank_to_vertex);
  CHECK(loaded.vertex_to_rank == result.vertex_to_rank);
  check_exact(g, loaded.labels);
}

void test_varint_encoding_byte_boundaries() {
  const std::vector<Distance> boundary_values = {
      0,     1,       126,     127,       128,       16383,
      16384, 2097151, 2097152, 268435455, 268435456, kInfinity - 1};
  const VertexId n = static_cast<VertexId>(boundary_values.size());
  LabelingResult result;
  result.labels.resize(n);
  result.rank_to_vertex.resize(n);
  result.vertex_to_rank.resize(n);
  for (VertexId i = 0; i < n; ++i)
    result.rank_to_vertex[i] = i;
  for (VertexId i = 0; i < n; ++i)
    result.vertex_to_rank[i] = i;
  const std::vector<VertexId> deltas(n, 0);
  const std::vector<Distance> distances(boundary_values.begin(),
                                        boundary_values.end());
  result.labels[0].forward = Label::from_deltas(deltas, distances, n);
  result.labels[0].backward = Label::from_deltas(deltas, distances, n);

  const std::string path = "rxl_test_varint_boundaries.bin";
  IndexIO::export_binary(result, path, LabelEncoding::Varint);
  auto loaded = IndexIO::import_binary(path);
  std::remove(path.c_str());

  std::vector<Distance> forward, backward;
  for (const auto &[hub, d] : loaded.labels[0].forward)
    forward.push_back(d);
  for (const auto &[hub, d] : loaded.labels[0].backward)
    backward.push_back(d);
  CHECK(forward == distances);
  CHECK(backward == distances);
}
void test_parallel_determinism() {
  std::mt19937 rng(17);
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  for (VertexId u = 0; u < 25; ++u)
    for (VertexId v = 0; v < 25; ++v)
      if (u != v && rng() % 7 == 0)
        arcs.emplace_back(u, v, 1 + rng() % 20);
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

void test_sparse_tree_storage() {
  std::mt19937 rng(2024);
  std::vector<std::tuple<VertexId, VertexId, Distance>> arcs;
  const VertexId n = 120;
  for (VertexId u = 0; u < n; ++u)
    for (VertexId v = 0; v < n; ++v)
      if (u != v && rng() % 9 == 0)
        arcs.emplace_back(u, v, 1 + rng() % 50);
  auto g = make_graph(n, arcs);
  SamplingOptions o;
  o.initial_trees = 32;
  o.counter_buckets = 8;
  o.discarded_max_buckets = 1;
  o.random_seed = 2024;
  auto result = PrunedLabeling::compute(g, o);
  check_exact(g, result.labels);
  CHECK(result.statistics.sparse_downgrades > 0);
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
    test_default_encoding_is_two_block();
    test_varint_encoding_roundtrip();
    test_varint_encoding_byte_boundaries();
    test_parallel_determinism();
    test_sparse_tree_storage();
    test_statistics();
    std::cout << "All RXL tests passed\n";
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}

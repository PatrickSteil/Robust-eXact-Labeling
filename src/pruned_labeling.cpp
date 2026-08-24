#include "pruned_labeling.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <thread>

#include "parallel_for.h"
#include "pruned_dijkstra.h"
#include "sampg.h"
#include "search_frontier.h"

namespace rxl {
namespace {

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

namespace {
template <typename Frontier>
LabelingResult compute_with_degree_order_impl(const Graph& graph,
                                              const bool verbose) {
  const std::size_t n = graph.num_vertices();
  HubLabels labels(n);
  auto order = PrunedLabeling::degree_order(graph);
  std::vector<Distance> root_out(n, kInfinity), root_in(n, kInfinity),
      distance(n, kInfinity);
  std::vector<VertexId> lookup, touched;
  Frontier frontier(n);
  for (std::size_t rank = 0; rank < order.size(); ++rank) {
    if (verbose && (rank < 10 || (rank + 1) % 1000 == 0 || rank + 1 == n)) {
      std::cerr << "[rxl] rank " << (rank + 1) << '/' << n
                << ": selected vertex " << order[rank] << "\n";
    }

    HubExpansion::add_hub(graph, order[rank], static_cast<VertexId>(rank),
                          labels, root_out, root_in, distance, lookup, touched,
                          frontier);
  }
  return finish(std::move(labels), std::move(order));
}
}  // namespace

LabelingResult PrunedLabeling::compute_with_degree_order(
    const Graph& graph, const bool verbose, const bool zero_one_bfs) {
  if (zero_one_bfs && !graph.is_zero_one_weighted())
    throw std::invalid_argument(
        "PrunedLabeling::compute_with_degree_order: zero_one_bfs requires "
        "every edge weight to be 0 or 1");
  return zero_one_bfs
             ? compute_with_degree_order_impl<ZeroOneBfsFrontier>(graph,
                                                                  verbose)
             : compute_with_degree_order_impl<DijkstraFrontier>(graph, verbose);
}

LabelingResult PrunedLabeling::compute(const Graph& graph,
                                       const SamplingOptions& options) {
  if (options.initial_trees == 0)
    return compute_with_degree_order(graph, options.verbose,
                                     options.zero_one_bfs);
  HubLabels labels(graph.num_vertices());
  BuildStatistics statistics;
  const auto start = std::chrono::steady_clock::now();
  auto order = SamPG::build_order(graph, options, labels, statistics);
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

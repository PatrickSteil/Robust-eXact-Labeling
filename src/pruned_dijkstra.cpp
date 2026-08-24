
#include "pruned_dijkstra.h"

#include "abstract_dijkstra.h"

namespace rxl {

bool HubExpansion::covered(const Label& label,
                           const std::vector<Distance>& root_distance,
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

template <typename Frontier>
std::uint64_t HubExpansion::pruned_dijkstra(
    const CsrAdjacency& graph, VertexId root, VertexId hub_id,
    const std::vector<Distance>& root_distance, HubLabels& labels, bool forward,
    std::vector<Distance>& distance, std::vector<VertexId>& touched,
    Frontier& frontier) {
  std::uint64_t work = 0;
  AbstractDijkstra::search(
      root, [&graph](VertexId u) { return graph[u]; }, distance, touched,
      frontier,
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

template <typename Frontier>
std::uint64_t HubExpansion::add_hub(
    const Graph& graph, VertexId root, VertexId hub_id, HubLabels& labels,
    std::vector<Distance>& root_out, std::vector<Distance>& root_in,
    std::vector<Distance>& distance, std::vector<VertexId>& lookup_touched,
    std::vector<VertexId>& search_touched, Frontier& frontier) {
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
                      distance, search_touched, frontier);
  work += pruned_dijkstra(graph.reverse_adjacency(), root, hub_id, root_in,
                          labels, false, distance, search_touched, frontier);
  for (VertexId hub : lookup_touched) {
    root_out[hub] = kInfinity;
    root_in[hub] = kInfinity;
  }
  lookup_touched.clear();
  return work;
}

template std::uint64_t HubExpansion::add_hub<DijkstraFrontier>(
    const Graph&, VertexId, VertexId, HubLabels&, std::vector<Distance>&,
    std::vector<Distance>&, std::vector<Distance>&, std::vector<VertexId>&,
    std::vector<VertexId>&, DijkstraFrontier&);
template std::uint64_t HubExpansion::add_hub<ZeroOneBfsFrontier>(
    const Graph&, VertexId, VertexId, HubLabels&, std::vector<Distance>&,
    std::vector<Distance>&, std::vector<Distance>&, std::vector<VertexId>&,
    std::vector<VertexId>&, ZeroOneBfsFrontier&);

}  // namespace rxl

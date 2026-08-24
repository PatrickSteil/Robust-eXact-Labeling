#include "batch_pruned_labeling.h"

#include <algorithm>
#include <cstddef>

#include "abstract_dijkstra.h"
#include "parallel_for.h"
#include "pruned_dijkstra.h"
#include "search_frontier.h"
#include "thread_pool.h"

namespace rxl {
namespace {

// Same traversal as HubExpansion's private pruned_dijkstra, except it
// records (target, distance) pairs into `updates` instead of writing
// straight into `labels` -- concurrently running lanes only ever read
// `labels` (frozen at the start of their wave), never write it, so no
// synchronization is needed until the later, per-wave commit phase.
template <typename Frontier>
std::uint64_t batch_pruned_search(
    const CsrAdjacency& adjacency, VertexId root,
    const std::vector<Distance>& root_distance, const HubLabels& labels,
    bool forward, std::vector<Distance>& distance,
    std::vector<VertexId>& touched, Frontier& frontier,
    std::vector<std::pair<VertexId, Distance>>& updates) {
  std::uint64_t work = 0;
  AbstractDijkstra::search(
      root, [&adjacency](VertexId u) { return adjacency[u]; }, distance,
      touched, frontier,
      /*on_pop=*/[&](VertexId, Distance) { ++work; },
      /*should_prune=*/
      [&](VertexId u, Distance du) {
        const Label& query = forward ? labels[u].backward : labels[u].forward;
        return HubExpansion::covered(query, root_distance, du);
      },
      /*on_settle=*/
      [&](VertexId u, Distance du) {
        updates.emplace_back(u, du);
        work += adjacency[u].size();
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
std::uint64_t run_lane(const Graph& graph, VertexId root,
                       const HubLabels& labels,
                       BatchLaneScratch<Frontier>& scratch) {
  const std::size_t n = graph.num_vertices();
  if (scratch.distance.size() != n) {
    scratch.root_out.assign(n, kInfinity);
    scratch.root_in.assign(n, kInfinity);
    scratch.distance.assign(n, kInfinity);
    scratch.frontier.assign(n);
  }
  scratch.backward_updates.clear();
  scratch.forward_updates.clear();

  for (const auto& [hub, d] : labels[root].forward) {
    scratch.root_out[hub] = d;
    scratch.lookup_touched.push_back(hub);
  }
  for (const auto& [hub, d] : labels[root].backward) {
    scratch.root_in[hub] = d;
    scratch.lookup_touched.push_back(hub);
  }

  std::uint64_t work = batch_pruned_search(
      graph.adjacency(), root, scratch.root_out, labels, /*forward=*/true,
      scratch.distance, scratch.search_touched, scratch.frontier,
      scratch.backward_updates);
  work += batch_pruned_search(
      graph.reverse_adjacency(), root, scratch.root_in, labels,
      /*forward=*/false, scratch.distance, scratch.search_touched,
      scratch.frontier, scratch.forward_updates);

  for (VertexId hub : scratch.lookup_touched) {
    scratch.root_out[hub] = kInfinity;
    scratch.root_in[hub] = kInfinity;
  }
  scratch.lookup_touched.clear();
  return work;
}

}  // namespace

template <typename Frontier>
std::uint64_t BatchHubExpansion::add_hub_batch(
    const Graph& graph, const std::vector<VertexId>& roots,
    VertexId first_hub_id, HubLabels& labels, std::size_t threads,
    std::vector<BatchLaneScratch<Frontier>>& lane_scratch,
    BatchCommitScratch& commit_scratch) {
  if (roots.empty()) return 0;
  const std::size_t n = graph.num_vertices();
  commit_scratch.ensure_size(n);
  threads = std::max<std::size_t>(1, threads);

  std::uint64_t total_work = 0;
  // Roots are processed in waves of up to `threads` concurrent lanes. The
  // search phase within a wave needs no locking at all (see file header),
  // but each wave's results are committed into `labels` -- serially
  // merged, then in-parallel written -- before the NEXT wave's lanes read
  // `labels` to build their own search state. This bounds the
  // cross-pruning blind spot to one wave (<= threads roots), regardless
  // of how many waves roots.size() requires.
  for (std::size_t wave_begin = 0; wave_begin < roots.size();
       wave_begin += threads) {
    const std::size_t wave_end = std::min(roots.size(), wave_begin + threads);
    const std::size_t wave_size = wave_end - wave_begin;
    if (lane_scratch.size() < wave_size) lane_scratch.resize(wave_size);

    std::vector<std::uint64_t> lane_work(wave_size, 0);
    if (wave_size == 1) {
      lane_work[0] =
          run_lane(graph, roots[wave_begin], labels, lane_scratch[0]);
    } else {
      ThreadPool::instance().run(wave_size, [&](std::size_t i) {
        lane_work[i] =
            run_lane(graph, roots[wave_begin + i], labels, lane_scratch[i]);
      });
    }

    // Serial merge of this wave's per-lane updates into the wave-local
    // pending buffers, keyed by the root's position within the wave (used
    // below as the hub-id tie-break for targets multiple lanes reached).
    for (std::size_t i = 0; i < wave_size; ++i) {
      total_work += lane_work[i];
      const std::uint32_t lane_index = static_cast<std::uint32_t>(i);
      for (const auto& [target, dist] : lane_scratch[i].backward_updates) {
        if (commit_scratch.pending_backward[target].empty())
          commit_scratch.touched_backward.push_back(target);
        commit_scratch.pending_backward[target].emplace_back(lane_index,
                                                              dist);
      }
      for (const auto& [target, dist] : lane_scratch[i].forward_updates) {
        if (commit_scratch.pending_forward[target].empty())
          commit_scratch.touched_forward.push_back(target);
        commit_scratch.pending_forward[target].emplace_back(lane_index, dist);
      }
    }

    // Commit phase for this wave only: every touched vertex's pending
    // list is independent of every other vertex's, so this runs in
    // parallel over vertices. Sorting by lane_index (ascending) is
    // equivalent to sorting by hub id (= wave_begin + lane_index), which
    // is what DeltaLabel::push_back requires.
    const VertexId wave_first_hub_id =
        static_cast<VertexId>(first_hub_id + wave_begin);
    parallel_for(
        commit_scratch.touched_backward.size(), threads,
        [&](std::size_t lo, std::size_t hi) {
          for (std::size_t i = lo; i < hi; ++i) {
            const VertexId v = commit_scratch.touched_backward[i];
            auto& pending = commit_scratch.pending_backward[v];
            std::sort(pending.begin(), pending.end());
            for (const auto& [lane_index, dist] : pending)
              labels[v].backward.push_back(
                  static_cast<VertexId>(wave_first_hub_id + lane_index), dist);
            pending.clear();
          }
        });
    parallel_for(
        commit_scratch.touched_forward.size(), threads,
        [&](std::size_t lo, std::size_t hi) {
          for (std::size_t i = lo; i < hi; ++i) {
            const VertexId v = commit_scratch.touched_forward[i];
            auto& pending = commit_scratch.pending_forward[v];
            std::sort(pending.begin(), pending.end());
            for (const auto& [lane_index, dist] : pending)
              labels[v].forward.push_back(
                  static_cast<VertexId>(wave_first_hub_id + lane_index), dist);
            pending.clear();
          }
        });
    commit_scratch.touched_backward.clear();
    commit_scratch.touched_forward.clear();
  }

  return total_work;
}

template std::uint64_t BatchHubExpansion::add_hub_batch<DijkstraFrontier>(
    const Graph&, const std::vector<VertexId>&, VertexId, HubLabels&,
    std::size_t, std::vector<BatchLaneScratch<DijkstraFrontier>>&,
    BatchCommitScratch&);
template std::uint64_t BatchHubExpansion::add_hub_batch<ZeroOneBfsFrontier>(
    const Graph&, const std::vector<VertexId>&, VertexId, HubLabels&,
    std::size_t, std::vector<BatchLaneScratch<ZeroOneBfsFrontier>>&,
    BatchCommitScratch&);

}  // namespace rxl

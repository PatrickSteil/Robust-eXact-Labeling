#include <chrono>
#include <cmath>
#include <cstdio>

#include "graph.h"
#include "pruned_labeling.h"

using namespace rxl;

// Square grid graph (like the paper's "gridi" class): side x side vertices,
// 4-neighbor connectivity, unit weights, undirected (both directions).
// These have small, slowly-growing hub-label sizes, unlike a uniform random
// digraph -- matching what SamPG is actually meant for (Table 1 / Figure 3).
Graph make_grid_graph(std::size_t side) {
  const std::size_t n = side * side;
  AdjacencyList a(n), r(n);
  auto id = [side](std::size_t x, std::size_t y) { return x * side + y; };
  for (std::size_t x = 0; x < side; ++x) {
    for (std::size_t y = 0; y < side; ++y) {
      const VertexId u = static_cast<VertexId>(id(x, y));
      if (x + 1 < side) {
        const VertexId v = static_cast<VertexId>(id(x + 1, y));
        a[u].emplace_back(v, 1);
        a[v].emplace_back(u, 1);
        r[v].emplace_back(u, 1);
        r[u].emplace_back(v, 1);
      }
      if (y + 1 < side) {
        const VertexId v = static_cast<VertexId>(id(x, y + 1));
        a[u].emplace_back(v, 1);
        a[v].emplace_back(u, 1);
        r[v].emplace_back(u, 1);
        r[u].emplace_back(v, 1);
      }
    }
  }
  return Graph(std::move(a), std::move(r));
}

int main(int argc, char** argv) {
  std::size_t side = argc > 1 ? std::stoul(argv[1]) : 100;
  auto g = make_grid_graph(side);
  const std::size_t n = side * side;
  SamplingOptions o;
  o.initial_trees = 64;
  o.counter_buckets = 16;
  o.discarded_max_buckets = 2;
  o.num_threads = 1;
  auto t0 = std::chrono::steady_clock::now();
  auto result = PrunedLabeling::compute(g, o);
  auto t1 = std::chrono::steady_clock::now();
  double secs = std::chrono::duration<double>(t1 - t0).count();
  double avg_label = 0;
  for (auto& l : result.labels)
    avg_label += l.forward.size() + l.backward.size();
  avg_label /= (2.0 * n);
  std::printf("side=%zu n=%zu  time=%.3fs  avg_label_size=%.2f\n", side, n,
              secs, avg_label);
  return 0;
}

#include <atomic>
#include <cctype>
#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <system_error>
#include <thread>

#include "graph.h"
#include "index_io.h"
#include "parallel_for.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "statistics.h"
using namespace rxl;
namespace {
void usage(const char* program) {
  std::cerr
      << "Usage: " << program
      << " [graph] [options]\n"
         "  --export <file>   export labels/rank maps to a binary RXL index\n"
         "  --label-encoding <two-block|varint>\n"
         "                    on-disk hub-id/distance byte packing used by "
         "--export\n"
         "                    (default: two-block). two-block is the "
         "paper's fixed\n"
         "                    1-byte/4-byte split (Section 4.1); varint "
         "packs every\n"
         "                    value as a LEB128 varint (7 payload bits per "
         "byte, top\n"
         "                    bit set to mean \"one more byte follows\"), "
         "often\n"
         "                    smaller but slower to decode. Ignored "
         "without --export.\n"
         "  --import <file>   load labels/rank maps from a binary RXL index\n"
         "                    instead of building them (graph argument then\n"
         "                    becomes optional)\n"
         "  --benchmark [n]   run n (default 10000) random vertex-to-vertex\n"
         "                    queries and report the average query runtime "
         "and\n"
         "                    the number of reachable (found) pairs\n"
         "  --verbose         print graph, sampling, and label statistics\n"
         "  --threads <n>     parallel sample-tree workers (default: 1)\n"
         "  --seed <n>        seed for the query benchmark (default: 42)\n"
         "  --degree          use degree ordering instead of SamPG\n"
         "  --no-reorder      keep original internal zero-based IDs\n";
}
bool is_number(const std::string& s) {
  if (s.empty()) return false;
  for (char c : s)
    if (!std::isdigit(static_cast<unsigned char>(c))) return false;
  return true;
}
void print_graph_statistics(const Graph& graph) {
  const auto s = compute_graph_statistics(graph);
  std::cout << "Graph: vertices=" << s.vertices << ", arcs=" << s.arcs
            << ", weighted=" << (graph.is_weighted() ? "yes" : "no")
            << ", out-degree[min/avg/max]=" << s.min_out_degree << '/'
            << s.average_out_degree << '/' << s.max_out_degree
            << ", isolated=" << s.isolated_vertices << '\n';
}

void run_benchmark(const HubLabels& labels, std::size_t num_queries,
                   std::size_t threads, const int seed = 42) {
  const std::size_t n = labels.size();
  if (n == 0) throw std::invalid_argument("cannot benchmark an empty index");
  std::mt19937_64 rng(seed);
  std::uniform_int_distribution<VertexId> dist(0, static_cast<VertexId>(n - 1));

  std::vector<std::pair<VertexId, VertexId>> queries(num_queries);
  for (auto& q : queries) q = {dist(rng), dist(rng)};

  // Query generation stays serial (so the exact same queries are asked
  // regardless of --threads), and only the timed portion -- independent,
  // read-only lookups against `labels` -- is handed to parallel_for. Each
  // worker tallies hits in a local (uncontended) counter and folds it into
  // `found` once per chunk rather than per query, so the shared atomic is
  // touched O(threads) times, not O(num_queries) times.
  std::atomic<std::size_t> found{0};
  const auto start = std::chrono::steady_clock::now();
  parallel_for(num_queries, threads, [&](std::size_t lo, std::size_t hi) {
    std::size_t local = 0;
    for (std::size_t i = lo; i < hi; ++i) {
      const auto& [s, t] = queries[i];
      if (QuerySupport::distance(labels, s, t) != kInfinity) ++local;
    }
    found += local;
  });
  const auto end = std::chrono::steady_clock::now();
  const double total_us =
      std::chrono::duration<double, std::micro>(end - start).count();
  const double avg_us = total_us / static_cast<double>(num_queries);
  std::cout << "Benchmark: queries=" << num_queries << ", threads=" << threads
            << ", average-runtime-us=" << avg_us << ", found=" << found << '/'
            << num_queries << '\n';
}
}  // namespace
int main(int argc, char** argv) {
  if (argc < 2) {
    usage(argv[0]);
    return 1;
  }
  try {
    bool degree = false, reorder = true, verbose = false, do_benchmark = false;
    std::string export_path, import_path;
    std::size_t threads = 1, benchmark_queries = 10000;
    int seed = 42;
    LabelEncoding encoding = LabelEncoding::TwoBlockDelta;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--degree")
        degree = true;
      else if (arg == "--no-reorder")
        reorder = false;
      else if (arg == "--verbose")
        verbose = true;
      else if (arg == "--export") {
        if (++i >= argc) throw std::invalid_argument("--export needs a path");
        export_path = argv[i];
      } else if (arg == "--import") {
        if (++i >= argc) throw std::invalid_argument("--import needs a path");
        import_path = argv[i];
      } else if (arg == "--label-encoding") {
        if (++i >= argc)
          throw std::invalid_argument("--label-encoding needs a value");
        const std::string value = argv[i];
        if (value == "two-block")
          encoding = LabelEncoding::TwoBlockDelta;
        else if (value == "varint")
          encoding = LabelEncoding::Varint;
        else
          throw std::invalid_argument(
              "--label-encoding must be 'two-block' or 'varint', got: " +
              value);
      } else if (arg == "--threads") {
        if (++i >= argc) throw std::invalid_argument("--threads needs a count");
        threads = std::stoull(argv[i]);
        if (!threads)
          throw std::invalid_argument("thread count must be positive");
      } else if (arg == "--benchmark") {
        do_benchmark = true;
        benchmark_queries = 10000;
        if (i + 1 < argc && is_number(argv[i + 1]))
          benchmark_queries = std::stoull(argv[++i]);
        if (!benchmark_queries)
          throw std::invalid_argument("benchmark query count must be positive");
      } else if (arg == "--seed") {
        if (i + 1 < argc && is_number(argv[i + 1]))
          seed = std::stoi(argv[++i]);
        else
          throw std::invalid_argument("--seed needs an integer");
      } else if (!arg.empty() && arg[0] == '-')
        throw std::invalid_argument("unknown option: " + arg);
      else
        positional.push_back(arg);
    }
    if (positional.size() > 1)
      throw std::invalid_argument("provide at most one graph file");
    if (positional.empty() && import_path.empty())
      throw std::invalid_argument(
          "provide a graph file (or --import a saved index)");

    LabelingResult result;
    if (!import_path.empty()) {
      result = IndexIO::import_binary(import_path);
      if (verbose) {
        if (!positional.empty()) print_graph_statistics(Graph(positional[0]));
        std::cout << "Imported index from " << import_path
                  << ", vertices=" << result.labels.size()
                  << ", rank-reordered="
                  << (result.rank_reordered ? "yes" : "no") << '\n';
      }
    } else {
      Graph graph(positional[0]);
      if (verbose) print_graph_statistics(graph);
      SamplingOptions options;
      options.num_threads = threads;
      options.verbose = verbose;
      result = degree ? PrunedLabeling::compute_with_degree_order(graph)
                      : PrunedLabeling::compute(graph, options);
      if (reorder) {
        graph.reorder_by_rank(result.rank_to_vertex);
        PrunedLabeling::reorder_labels_by_rank(result);
      }
      if (verbose) {
        const auto s = compute_label_statistics(result.labels);
        std::cout << "Labels: forward=" << s.forward_entries
                  << ", backward=" << s.backward_entries
                  << ", average[fwd/back]=" << s.average_forward_size << '/'
                  << s.average_backward_size
                  << ", maximum[fwd/back]=" << s.max_forward_size << '/'
                  << s.max_backward_size
                  << ", raw-entry-bytes=" << s.payload_bytes << '\n';
        std::cout << "Build: ordering=" << (degree ? "degree" : "SamPG")
                  << ", threads=" << threads << ", seconds="
                  << result.statistics.ordering_and_labeling_seconds
                  << ", sampled-trees=" << result.statistics.sampled_trees
                  << ", peak-live-trees=" << result.statistics.peak_live_trees
                  << ", sparse-downgrades="
                  << result.statistics.sparse_downgrades
                  << ", sampling-work=" << result.statistics.sampling_work
                  << ", labeling-work=" << result.statistics.labeling_work
                  << '\n';
      }
    }

    if (!export_path.empty()) {
      IndexIO::export_binary(result, export_path, encoding);
      if (verbose) {
        std::error_code ec;
        const auto bytes = std::filesystem::file_size(export_path, ec);
        std::cout << "Exported index to " << export_path << ", encoding="
                  << (encoding == LabelEncoding::Varint ? "varint"
                                                        : "two-block");
        if (!ec) std::cout << ", bytes=" << bytes;
        std::cout << '\n';
      }
    }
    if (do_benchmark) {
      run_benchmark(result.labels, benchmark_queries, threads, seed);
    }
  } catch (const std::exception& e) {
    std::cerr << "Error: " << e.what() << '\n';
    return 2;
  }
}

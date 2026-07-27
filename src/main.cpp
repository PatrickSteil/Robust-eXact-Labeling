#include <atomic>
#include <chrono>
#include <exception>
#include <filesystem>
#include <iostream>
#include <random>
#include <string>
#include <system_error>
#include <thread>

#include "cmdparser.hpp"
#include "graph.h"
#include "index_io.h"
#include "parallel_for.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "statistics.h"
using namespace rxl;
namespace {
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

  std::size_t found{0};
  const auto start = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < queries.size(); ++i) {
    const auto& [s, t] = queries[i];
    if (QuerySupport::distance(labels, s, t) != kInfinity) ++found;
  }
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
  try {
    cli::Parser parser(argc, argv);
    parser.set_default<std::string>(
        false, "Input graph file (see --format); optional only with --import",
        "");
    parser.set_optional<std::string>(
        "f", "format", "dimacs",
        "Input graph file format: dimacs, snap, metis, or csv");
    parser.set_optional<std::string>("e", "export", "",
                                     "Export labels/rank maps to a binary "
                                     "RXL index");
    parser.set_optional<std::string>(
        "l", "label-encoding", "two-block",
        "On-disk hub-id/distance byte packing used by --export "
        "(two-block|varint)");
    parser.set_optional<std::string>(
        "i", "import", "",
        "Load labels/rank maps from a binary RXL index instead of "
        "building them");
    parser.set_optional<bool>(
        "b", "benchmark", false,
        "Run a benchmark of random vertex-to-vertex queries");
    parser.set_optional<unsigned long long>(
        "q", "benchmark-queries", 10000,
        "Number of queries to run with --benchmark");
    parser.set_optional<bool>("v", "verbose", false,
                              "Print graph, sampling, and label statistics");
    parser.set_optional<unsigned long long>("t", "threads", 1,
                                            "Parallel sample-tree workers");
    parser.set_optional<int>("s", "seed", 42, "Seed for the query benchmark");
    parser.set_optional<bool>("d", "degree", false,
                              "Use degree ordering instead of SamPG");
    parser.set_optional<bool>("r", "no-reorder", false,
                              "Keep original internal zero-based IDs");
    parser.set_optional<unsigned long long>(
        "m", "min-tree-vertices", 8,
        "Force-retire a sample tree once it has shrunk to at most this "
        "many remaining vertices (0 disables early retirement)");

    if (!parser.run()) return 1;

    const std::string positional = parser.get_default<std::string>();
    const std::string format_name = parser.get<std::string>("f");
    const std::string export_path = parser.get<std::string>("e");
    const std::string label_encoding_name = parser.get<std::string>("l");
    const std::string import_path = parser.get<std::string>("i");
    const bool do_benchmark = parser.get<bool>("b");
    const std::size_t benchmark_queries =
        static_cast<std::size_t>(parser.get<unsigned long long>("q"));
    const bool verbose = parser.get<bool>("v");
    const std::size_t threads =
        static_cast<std::size_t>(parser.get<unsigned long long>("t"));
    const int seed = parser.get<int>("s");
    const bool degree = parser.get<bool>("d");
    const bool reorder = !parser.get<bool>("r");
    const std::size_t min_tree_vertices =
        static_cast<std::size_t>(parser.get<unsigned long long>("m"));

    if (!benchmark_queries)
      throw std::invalid_argument("--benchmark-queries must be positive");
    if (!threads) throw std::invalid_argument("--threads must be positive");

    const GraphFormat format = parse_graph_format(format_name);

    LabelEncoding encoding;
    if (label_encoding_name == "two-block")
      encoding = LabelEncoding::TwoBlockDelta;
    else if (label_encoding_name == "varint")
      encoding = LabelEncoding::Varint;
    else
      throw std::invalid_argument(
          "--label-encoding must be 'two-block' or 'varint', got: " +
          label_encoding_name);

    if (positional.empty() && import_path.empty())
      throw std::invalid_argument(
          "provide a graph file (or --import a saved index)");

    LabelingResult result;
    if (!import_path.empty()) {
      result = IndexIO::import_binary(import_path);
      if (verbose) {
        if (!positional.empty())
          print_graph_statistics(Graph(positional, format));
        std::cout << "Imported index from " << import_path
                  << ", vertices=" << result.labels.size()
                  << ", rank-reordered="
                  << (result.rank_reordered ? "yes" : "no") << '\n';
      }
    } else {
      Graph graph(positional, format);
      if (verbose) print_graph_statistics(graph);
      SamplingOptions options;
      options.num_threads = threads;
      options.verbose = verbose;
      options.min_tree_vertices = min_tree_vertices;
      result = degree
                   ? PrunedLabeling::compute_with_degree_order(graph, verbose)
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

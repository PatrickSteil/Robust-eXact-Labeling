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
    parser.set_optional<bool>(
        "z", "zero-one-bfs", false,
        "Build with a deque-based 0-1 BFS instead of Dijkstra's "
        "binary-heap search. Faster, but only correct if every edge "
        "weight in the graph is 0 or 1; errors out otherwise");
    parser.set_optional<unsigned long long>(
        "batch-initial-size", "batch-initial-size", 1,
        "Number of hubs picked at once (without letting sample-tree "
        "scores react in between) at the start of the run. 1 keeps the "
        "original fully-sequential SamPG algorithm");
    parser.set_optional<unsigned long long>(
        "batch-max-size", "batch-max-size", 1,
        "Upper bound the adaptive-phase batch size may grow to (see "
        "--batch-growth). Ignored while <= --batch-initial-size");
    parser.set_optional<double>(
        "batch-growth", "batch-growth", 1.0,
        "Growth factor applied to the batch size after every batch "
        "during the adaptive phase, rounded up and clamped to "
        "--batch-max-size. 1.0 keeps the batch size fixed");
    parser.set_optional<bool>(
        "no-batch-diversity", "no-batch-diversity", false,
        "Disable the tree-membership diversity filter that requeues a "
        "batch candidate sharing a live sample tree with one already "
        "accepted this round (filter is on by default)");
    parser.set_optional<double>(
        "sampling-fraction", "sampling-fraction", 1.0,
        "Fraction of vertices (by rank) ordered via full adaptive SamPG "
        "sampling; the remaining tail is ordered once by freezing "
        "current SamPG priority scores and labeled via the same batched "
        "kernel. 1.0 samples the entire order");
    parser.set_optional<unsigned long long>(
        "tail-batch-size", "tail-batch-size", 0,
        "Batch size used once --sampling-fraction has been reached. "
        "0 (default) reuses --batch-max-size");

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
    const bool zero_one_bfs = parser.get<bool>("z");
    const std::size_t batch_initial_size = static_cast<std::size_t>(
        parser.get<unsigned long long>("batch-initial-size"));
    const std::size_t batch_max_size = static_cast<std::size_t>(
        parser.get<unsigned long long>("batch-max-size"));
    const double batch_growth = parser.get<double>("batch-growth");
    const bool batch_diversity_filter =
        !parser.get<bool>("no-batch-diversity");
    const double sampling_fraction = parser.get<double>("sampling-fraction");
    const std::size_t tail_batch_size = static_cast<std::size_t>(
        parser.get<unsigned long long>("tail-batch-size"));

    if (!benchmark_queries)
      throw std::invalid_argument("--benchmark-queries must be positive");
    if (!threads) throw std::invalid_argument("--threads must be positive");
    if (!batch_initial_size)
      throw std::invalid_argument("--batch-initial-size must be positive");
    if (!batch_max_size)
      throw std::invalid_argument("--batch-max-size must be positive");
    if (batch_growth < 1.0)
      throw std::invalid_argument("--batch-growth must be >= 1.0");
    if (sampling_fraction < 0.0 || sampling_fraction > 1.0)
      throw std::invalid_argument(
          "--sampling-fraction must be between 0.0 and 1.0");

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
      options.zero_one_bfs = zero_one_bfs;
      options.initial_batch_size = batch_initial_size;
      options.max_batch_size = batch_max_size;
      options.batch_growth_factor = batch_growth;
      options.batch_diversity_filter = batch_diversity_filter;
      options.sampling_fraction = sampling_fraction;
      options.tail_batch_size = tail_batch_size;
      if (degree &&
          (batch_initial_size != 1 || batch_max_size != 1 ||
           batch_growth != 1.0 || sampling_fraction != 1.0))
        std::cerr << "Warning: --batch-* and --sampling-fraction only "
                     "apply to SamPG ordering and are ignored with "
                     "--degree\n";
      result = degree ? PrunedLabeling::compute_with_degree_order(
                            graph, verbose, zero_one_bfs)
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
                  << ", frontier=" << (zero_one_bfs ? "0-1-bfs" : "dijkstra")
                  << ", threads=" << threads;
        if (!degree)
          std::cout << ", batch=[" << batch_initial_size << ".."
                     << batch_max_size << "]x" << batch_growth
                     << ", diversity="
                     << (batch_diversity_filter ? "on" : "off")
                     << ", sampling-fraction=" << sampling_fraction;
        std::cout << ", seconds="
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

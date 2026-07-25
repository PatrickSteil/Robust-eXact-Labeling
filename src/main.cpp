#include "graph.h"
#include "index_io.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "statistics.h"
#include <cctype>
#include <chrono>
#include <exception>
#include <iostream>
#include <random>
#include <string>
#include <thread>
using namespace rxl;
namespace {
void usage(const char *program) {
  std::cerr << "Usage: " << program << " [graph] [options]\n"
    "  --export <file>   export labels/rank maps to a binary RXL index\n"
    "  --import <file>   load labels/rank maps from a binary RXL index\n"
    "                    instead of building them (graph argument then\n"
    "                    becomes optional)\n"
    "  --benchmark [n]   run n (default 10000) random vertex-to-vertex\n"
    "                    queries and report the average query runtime and\n"
    "                    the number of reachable (found) pairs\n"
    "  --verbose         print graph, sampling, and label statistics\n"
    "  --threads <n>     parallel sample-tree workers (default: 1)\n"
    "  --degree          use degree ordering instead of SamPG\n"
    "  --no-reorder      keep original internal zero-based IDs\n";
}
bool is_number(const std::string &s) {
  if (s.empty()) return false;
  for (char c : s) if (!std::isdigit(static_cast<unsigned char>(c))) return false;
  return true;
}
void print_graph_statistics(const Graph &graph) {
  const auto s=compute_graph_statistics(graph);
  std::cout<<"Graph: vertices="<<s.vertices<<", arcs="<<s.arcs
   <<", weighted="<<(graph.is_weighted()?"yes":"no")
   <<", out-degree[min/avg/max]="<<s.min_out_degree<<'/'<<s.average_out_degree<<'/'<<s.max_out_degree
   <<", isolated="<<s.isolated_vertices<<'\n';
}
void run_benchmark(const HubLabels &labels, std::size_t num_queries) {
  const std::size_t n=labels.size();
  if(n==0) throw std::invalid_argument("cannot benchmark an empty index");
  std::mt19937_64 rng(std::random_device{}());
  std::uniform_int_distribution<VertexId> dist(0, static_cast<VertexId>(n-1));
  std::vector<std::pair<VertexId,VertexId>> queries(num_queries);
  for(auto &q:queries) q={dist(rng),dist(rng)};
  std::size_t found=0;
  const auto start=std::chrono::steady_clock::now();
  for(const auto &[s,t]:queries)
    if(QuerySupport::distance(labels,s,t)!=kInfinity) ++found;
  const auto end=std::chrono::steady_clock::now();
  const double total_us=std::chrono::duration<double,std::micro>(end-start).count();
  const double avg_us=total_us/static_cast<double>(num_queries);
  std::cout<<"Benchmark: queries="<<num_queries
   <<", average-runtime-us="<<avg_us
   <<", found="<<found<<'/'<<num_queries<<'\n';
}
} // namespace
int main(int argc,char **argv) {
  if(argc<2) { usage(argv[0]); return 1; }
  try {
    bool degree=false,reorder=true,verbose=false,do_benchmark=false;
    std::string export_path,import_path;
    std::size_t threads=1,benchmark_queries=10000;
    std::vector<std::string> positional;
    for(int i=1;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--degree") degree=true;
      else if(arg=="--no-reorder") reorder=false;
      else if(arg=="--verbose") verbose=true;
      else if(arg=="--export") { if(++i>=argc) throw std::invalid_argument("--export needs a path"); export_path=argv[i]; }
      else if(arg=="--import") { if(++i>=argc) throw std::invalid_argument("--import needs a path"); import_path=argv[i]; }
      else if(arg=="--threads") { if(++i>=argc) throw std::invalid_argument("--threads needs a count"); threads=std::stoull(argv[i]); if(!threads) throw std::invalid_argument("thread count must be positive"); }
      else if(arg=="--benchmark") {
        do_benchmark=true;
        benchmark_queries=10000;
        if(i+1<argc && is_number(argv[i+1])) benchmark_queries=std::stoull(argv[++i]);
        if(!benchmark_queries) throw std::invalid_argument("benchmark query count must be positive");
      }
      else if(!arg.empty() && arg[0]=='-') throw std::invalid_argument("unknown option: "+arg);
      else positional.push_back(arg);
    }
    if(positional.size()>1) throw std::invalid_argument("provide at most one graph file");
    if(positional.empty() && import_path.empty()) throw std::invalid_argument("provide a graph file (or --import a saved index)");

    LabelingResult result;
    if(!import_path.empty()) {
      result=IndexIO::import_binary(import_path);
      if(verbose) {
        if(!positional.empty()) print_graph_statistics(Graph(positional[0]));
        std::cout<<"Imported index from "<<import_path<<", vertices="<<result.labels.size()
         <<", rank-reordered="<<(result.rank_reordered?"yes":"no")<<'\n';
      }
    } else {
      Graph graph(positional[0]);
      if(verbose) print_graph_statistics(graph);
      SamplingOptions options; options.num_threads=threads; options.verbose=verbose;
      result=degree?PrunedLabeling::compute_with_degree_order(graph):PrunedLabeling::compute(graph,options);
      if(reorder) {
        graph.reorder_by_rank(result.rank_to_vertex);
        PrunedLabeling::reorder_labels_by_rank(result);
      }
      if(verbose) {
        const auto s=compute_label_statistics(result.labels);
        std::cout<<"Labels: forward="<<s.forward_entries<<", backward="<<s.backward_entries
         <<", average[fwd/back]="<<s.average_forward_size<<'/'<<s.average_backward_size
         <<", maximum[fwd/back]="<<s.max_forward_size<<'/'<<s.max_backward_size
         <<", raw-entry-bytes="<<s.payload_bytes<<'\n';
        std::cout<<"Build: ordering="<<(degree?"degree":"SamPG")<<", threads="<<threads
         <<", seconds="<<result.statistics.ordering_and_labeling_seconds
         <<", sampled-trees="<<result.statistics.sampled_trees
         <<", peak-live-trees="<<result.statistics.peak_live_trees
         <<", sampling-work="<<result.statistics.sampling_work
         <<", labeling-work="<<result.statistics.labeling_work<<'\n';
      }
    }

    if(!export_path.empty()) { IndexIO::export_binary(result,export_path); if(verbose) std::cout<<"Exported index to "<<export_path<<'\n'; }
    if(do_benchmark) run_benchmark(result.labels,benchmark_queries);
  } catch(const std::exception &e) { std::cerr<<"Error: "<<e.what()<<'\n'; return 2; }
}

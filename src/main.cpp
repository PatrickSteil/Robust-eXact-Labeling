#include "graph.h"
#include "index_io.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "statistics.h"
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <thread>
using namespace rxl;
namespace {
void usage(const char *program) {
  std::cerr << "Usage: " << program << " <graph> [source target] [options]\n"
    "  --export <file>  export labels/rank maps to a binary RXL index\n"
    "  --verbose        print graph, sampling, and label statistics\n"
    "  --threads <n>    parallel sample-tree workers (default: 1)\n"
    "  --degree         use degree ordering instead of SamPG\n"
    "  --no-reorder     keep original internal zero-based IDs\n";
}
}
int main(int argc,char **argv) {
  if(argc<2) { usage(argv[0]); return 1; }
  try {
    bool degree=false,reorder=true,verbose=false; std::string export_path;
    std::size_t threads=1; std::vector<std::string> positional;
    for(int i=2;i<argc;++i) {
      const std::string arg=argv[i];
      if(arg=="--degree") degree=true;
      else if(arg=="--no-reorder") reorder=false;
      else if(arg=="--verbose") verbose=true;
      else if(arg=="--export") { if(++i>=argc) throw std::invalid_argument("--export needs a path"); export_path=argv[i]; }
      else if(arg=="--threads") { if(++i>=argc) throw std::invalid_argument("--threads needs a count"); threads=std::stoull(argv[i]); if(!threads) throw std::invalid_argument("thread count must be positive"); }
      else if(!arg.empty() && arg[0]=='-') throw std::invalid_argument("unknown option: "+arg);
      else positional.push_back(arg);
    }
    if(positional.size()!=0 && positional.size()!=2) throw std::invalid_argument("provide both source and target");
    Graph graph(argv[1]);
    if(verbose) {
      const auto s=compute_graph_statistics(graph);
      std::cout<<"Graph: vertices="<<s.vertices<<", arcs="<<s.arcs
       <<", weighted="<<(graph.is_weighted()?"yes":"no")
       <<", out-degree[min/avg/max]="<<s.min_out_degree<<'/'<<s.average_out_degree<<'/'<<s.max_out_degree
       <<", isolated="<<s.isolated_vertices<<'\n';
    }
    SamplingOptions options; options.num_threads=threads; options.verbose=verbose;
    LabelingResult result=degree?PrunedLabeling::compute_with_degree_order(graph):PrunedLabeling::compute(graph,options);
    VertexId source=0,target=graph.num_vertices()>1?1:0;
    if(positional.size()==2) {
      const auto a=std::stoull(positional[0]),b=std::stoull(positional[1]);
      if(!a||!b||a>graph.num_vertices()||b>graph.num_vertices()) throw std::out_of_range("DIMACS query IDs must be in 1..n");
      source=static_cast<VertexId>(a-1);target=static_cast<VertexId>(b-1);
    }
    if(reorder) {
      const auto old_to_new=graph.reorder_by_rank(result.rank_to_vertex);
      PrunedLabeling::reorder_labels_by_rank(result);
      source=old_to_new[source];target=old_to_new[target];
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
    if(!export_path.empty()) { IndexIO::export_binary(result,export_path); if(verbose) std::cout<<"Exported index to "<<export_path<<'\n'; }
    const Distance answer=QuerySupport::distance(result.labels,source,target);
    std::cout<<"Query distance: "; if(answer==kInfinity) std::cout<<"unreachable\n"; else std::cout<<answer<<'\n';
  } catch(const std::exception &e) { std::cerr<<"Error: "<<e.what()<<'\n'; return 2; }
}

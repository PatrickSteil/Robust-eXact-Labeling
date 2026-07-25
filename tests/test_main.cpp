#include "dijkstra.h"
#include "graph.h"
#include "index_io.h"
#include "pruned_labeling.h"
#include "query_support.h"
#include "statistics.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <tuple>
using namespace rxl;
#define CHECK(x) do { if(!(x)) throw std::runtime_error(std::string("CHECK failed: ")+ #x); } while(0)
Graph make_graph(std::size_t n,const std::vector<std::tuple<VertexId,VertexId,Distance>>& arcs) {
  AdjacencyList a(n),r(n); for(auto [u,v,w]:arcs){a[u].emplace_back(v,w);r[v].emplace_back(u,w);} return Graph(std::move(a),std::move(r));
}
void check_exact(const Graph &g,const HubLabels &labels) {
  for(VertexId s=0;s<g.num_vertices();++s) { auto truth=Dijkstra::shortest_distances(g.adjacency(),s); for(VertexId t=0;t<g.num_vertices();++t) CHECK(QuerySupport::distance(labels,s,t)==truth[t]); }
}
void test_directed_weighted() {
  auto g=make_graph(6,{{0,1,2},{0,2,9},{1,2,3},{2,3,4},{1,4,20},{3,4,1}});
  auto degree=PrunedLabeling::compute_with_degree_order(g); check_exact(g,degree.labels);
  SamplingOptions o;o.initial_trees=4;o.counter_buckets=2;o.discarded_max_buckets=1;o.num_threads=2;
  auto sampled=PrunedLabeling::compute(g,o);check_exact(g,sampled.labels);
}
void test_rank_reorder() {
  auto g=make_graph(5,{{0,1,1},{1,2,2},{0,3,8},{2,3,1},{3,4,3},{4,0,7}});
  auto result=PrunedLabeling::compute_with_degree_order(g);
  std::vector<std::vector<Distance>> truth(g.num_vertices());
  for(VertexId s=0;s<g.num_vertices();++s) truth[s]=Dijkstra::shortest_distances(g.adjacency(),s);
  auto map=g.reorder_by_rank(result.rank_to_vertex);PrunedLabeling::reorder_labels_by_rank(result);
  for(VertexId s=0;s<g.num_vertices();++s) for(VertexId t=0;t<g.num_vertices();++t)
    CHECK(QuerySupport::distance(result.labels,map[s],map[t])==truth[s][t]);
}
void test_export_roundtrip() {
  auto g=make_graph(4,{{0,1,5},{1,2,6},{2,0,2}});auto result=PrunedLabeling::compute_with_degree_order(g);
  const std::string path="rxl_test_index.bin";IndexIO::export_binary(result,path);auto loaded=IndexIO::import_binary(path);std::remove(path.c_str());
  CHECK(loaded.rank_to_vertex==result.rank_to_vertex);CHECK(loaded.vertex_to_rank==result.vertex_to_rank);CHECK(loaded.rank_reordered==result.rank_reordered);
  for(VertexId s=0;s<g.num_vertices();++s) for(VertexId t=0;t<g.num_vertices();++t) CHECK(QuerySupport::distance(loaded.labels,s,t)==QuerySupport::distance(result.labels,s,t));
}
void test_parallel_determinism() {
  std::mt19937 rng(17);std::vector<std::tuple<VertexId,VertexId,Distance>> arcs;
  for(VertexId u=0;u<25;++u)for(VertexId v=0;v<25;++v)if(u!=v&&rng()%7==0)arcs.emplace_back(u,v,1+rng()%20);
  auto g=make_graph(25,arcs);SamplingOptions a;a.initial_trees=8;a.counter_buckets=4;a.discarded_max_buckets=1;a.random_seed=99;a.num_threads=1;
  auto b=a;b.num_threads=4;auto one=PrunedLabeling::compute(g,a);auto four=PrunedLabeling::compute(g,b);
  check_exact(g,one.labels);check_exact(g,four.labels);
}
void test_statistics() {
  auto g=make_graph(3,{{0,1,1},{1,2,2}});auto gs=compute_graph_statistics(g);CHECK(gs.vertices==3);CHECK(gs.arcs==2);CHECK(gs.max_out_degree==1);
  auto result=PrunedLabeling::compute_with_degree_order(g);auto ls=compute_label_statistics(result.labels);CHECK(ls.forward_entries>0);CHECK(ls.backward_entries>0);
}
int main(){try{test_directed_weighted();test_rank_reorder();test_export_roundtrip();test_parallel_determinism();test_statistics();std::cout<<"All RXL tests passed\n";}catch(const std::exception&e){std::cerr<<e.what()<<'\n';return 1;}}

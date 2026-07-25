#include "index_io.h"
#include <array>
#include <fstream>
#include <limits>
#include <stdexcept>
namespace rxl { namespace {
void write_u32(std::ostream &out, std::uint32_t value) {
  const std::array<char,4> b{{char(value),char(value>>8),char(value>>16),char(value>>24)}};
  out.write(b.data(), b.size());
}
std::uint32_t read_u32(std::istream &in) {
  std::array<unsigned char,4> b{}; in.read(reinterpret_cast<char*>(b.data()),4);
  if (!in) throw std::runtime_error("Truncated RXL index");
  return std::uint32_t(b[0]) | std::uint32_t(b[1])<<8 |
         std::uint32_t(b[2])<<16 | std::uint32_t(b[3])<<24;
}
void write_label(std::ostream &out, const Label &label) {
  if (label.size() > std::numeric_limits<std::uint32_t>::max())
    throw std::overflow_error("Label too large to export");
  write_u32(out, static_cast<std::uint32_t>(label.size()));
  for (const auto &[hub,distance] : label) { write_u32(out,hub); write_u32(out,distance); }
}
Label read_label(std::istream &in, std::size_t n) {
  const auto size=read_u32(in); Label label; label.reserve(size);
  VertexId previous=0; bool first=true;
  for (std::uint32_t i=0;i<size;++i) {
    const VertexId hub=read_u32(in); const Distance distance=read_u32(in);
    if (hub>=n || distance==kInfinity || (!first && hub<=previous))
      throw std::runtime_error("Invalid label in RXL index");
    label.emplace_back(hub,distance); previous=hub; first=false;
  }
  return label;
}
void validate_permutation(const std::vector<VertexId> &p) {
  std::vector<std::uint8_t> seen(p.size(),0);
  for (auto v:p) { if(v>=p.size() || seen[v]) throw std::runtime_error("Invalid rank permutation"); seen[v]=1; }
}
} // namespace
void IndexIO::export_binary(const LabelingResult &result,const std::string &path) {
  const std::size_t n=result.labels.size();
  if(n>std::numeric_limits<std::uint32_t>::max() || result.rank_to_vertex.size()!=n || result.vertex_to_rank.size()!=n)
    throw std::invalid_argument("Incomplete or oversized labeling result");
  validate_permutation(result.rank_to_vertex);
  std::ofstream out(path,std::ios::binary); if(!out) throw std::runtime_error("Cannot open export file: "+path);
  out.write("RXLIDX\0\1",8); write_u32(out,1); write_u32(out,static_cast<std::uint32_t>(n));
  write_u32(out,result.rank_reordered ? 1u : 0u);
  for(auto v:result.rank_to_vertex) write_u32(out,v);
  for(auto v:result.vertex_to_rank) write_u32(out,v);
  for(const auto &entry:result.labels) { write_label(out,entry.forward); write_label(out,entry.backward); }
  if(!out) throw std::runtime_error("Failed while writing RXL index");
}
LabelingResult IndexIO::import_binary(const std::string &path) {
  std::ifstream in(path,std::ios::binary); if(!in) throw std::runtime_error("Cannot open index: "+path);
  std::array<char,8> magic{}; in.read(magic.data(),8);
  if(!in || std::string(magic.data(),8)!=std::string("RXLIDX\0\1",8)) throw std::runtime_error("Not an RXL index");
  if(read_u32(in)!=1) throw std::runtime_error("Unsupported RXL index version");
  const std::size_t n=read_u32(in); LabelingResult result; result.labels.resize(n);
  const std::uint32_t flags=read_u32(in);
  if(flags & ~1u) throw std::runtime_error("Unsupported RXL index flags");
  result.rank_reordered=(flags & 1u)!=0;
  result.rank_to_vertex.resize(n); result.vertex_to_rank.resize(n);
  for(auto &v:result.rank_to_vertex) v=read_u32(in);
  for(auto &v:result.vertex_to_rank) v=read_u32(in);
  validate_permutation(result.rank_to_vertex); validate_permutation(result.vertex_to_rank);
  for(VertexId rank=0;rank<n;++rank)
    if(result.vertex_to_rank[result.rank_to_vertex[rank]]!=rank)
      throw std::runtime_error("Rank maps are not inverses");
  for(auto &entry:result.labels) { entry.forward=read_label(in,n); entry.backward=read_label(in,n); }
  return result;
}
} // namespace rxl

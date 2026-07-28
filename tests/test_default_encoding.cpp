#include <cstdio>
#include <fstream>
#include <iterator>

#include "index_io.h"
#include "pruned_labeling.h"
#include "test_common.h"

void test_default_encoding_is_two_block() {
  auto g = make_graph(4, {{0, 1, 5}, {1, 2, 6}, {2, 0, 2}});
  auto result = PrunedLabeling::compute_with_degree_order(g);
  const std::string implicit_path = "rxl_test_encoding_default.bin";
  const std::string explicit_path = "rxl_test_encoding_two_block.bin";
  IndexIO::export_binary(result, implicit_path);
  IndexIO::export_binary(result, explicit_path, LabelEncoding::TwoBlockDelta);
  std::ifstream fa(implicit_path, std::ios::binary);
  std::ifstream fb(explicit_path, std::ios::binary);
  const std::vector<char> bytes_a((std::istreambuf_iterator<char>(fa)),
                                  std::istreambuf_iterator<char>());
  const std::vector<char> bytes_b((std::istreambuf_iterator<char>(fb)),
                                  std::istreambuf_iterator<char>());
  std::remove(implicit_path.c_str());
  std::remove(explicit_path.c_str());
  CHECK(bytes_a == bytes_b);
}

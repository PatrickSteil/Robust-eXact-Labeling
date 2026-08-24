#include <iostream>

// Declare all test functions
void test_delta_label();
void test_directed_weighted();
void test_rank_reorder();
void test_hub_ids_are_ranks();
void test_export_roundtrip();
void test_default_encoding_is_two_block();
void test_varint_encoding_roundtrip();
void test_varint_encoding_byte_boundaries();
void test_parallel_determinism();
void test_sparse_tree_storage();
void test_statistics();
void test_batch_size_one_matches_sequential();
void test_batched_labeling_is_exact();
void test_batched_labeling_thread_count_agrees();

int main() {
  try {
    test_delta_label();
    test_directed_weighted();
    test_rank_reorder();
    test_hub_ids_are_ranks();
    test_export_roundtrip();
    test_default_encoding_is_two_block();
    test_varint_encoding_roundtrip();
    test_varint_encoding_byte_boundaries();
    test_parallel_determinism();
    test_sparse_tree_storage();
    test_statistics();
    test_batch_size_one_matches_sequential();
    test_batched_labeling_is_exact();
    test_batched_labeling_thread_count_agrees();
    std::cout << "All RXL tests passed\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
  return 0;
}

#include <stdexcept>
#include <vector>

#include "delta_label.h"
#include "test_common.h"

using namespace rxl;

void test_delta_label() {
  DeltaLabel label;
  CHECK(label.empty());
  CHECK(label.size() == 0);
  label.push_back(0, 0);
  label.push_back(16, 5);
  label.push_back(29, 9);
  label.push_back(189, 12);
  CHECK(!label.empty());
  CHECK(label.size() == 4);

  std::vector<std::pair<VertexId, Distance>> decoded(label.begin(),
                                                     label.end());
  const std::vector<std::pair<VertexId, Distance>> expected{
      {0, 0}, {16, 5}, {29, 9}, {189, 12}};
  CHECK(decoded == expected);

  const std::vector<VertexId> expected_deltas{0, 15, 12, 159};
  CHECK(label.raw_deltas() == expected_deltas);

  VertexId sum_hubs = 0;
  for (const auto& [hub, d] : label) sum_hubs += hub + d;
  CHECK(sum_hubs == 0 + 0 + 16 + 5 + 29 + 9 + 189 + 12);

  bool threw = false;
  try {
    label.push_back(189, 1);  // repeat of the last hub id
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
    label.push_back(5, 1);  // smaller than the last hub id
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  CHECK(threw);

  const std::vector<Distance> flat_distances = label.raw_distances();
  auto rebuilt = DeltaLabel::from_deltas(label.raw_deltas(), flat_distances,
                                         /*n=*/200);
  const std::vector<std::pair<VertexId, Distance>> rebuilt_decoded(
      rebuilt.begin(), rebuilt.end());
  CHECK(rebuilt_decoded == expected);
  bool out_of_range = false;
  try {
    DeltaLabel::from_deltas(label.raw_deltas(), flat_distances,
                            /*n=*/189);  // hub 189 is not < 189
  } catch (const std::runtime_error&) {
    out_of_range = true;
  }
  CHECK(out_of_range);

  DeltaLabel empty_label;
  CHECK(empty_label.begin() == empty_label.end());
  auto rebuilt_empty = DeltaLabel::from_deltas({}, {}, 10);
  CHECK(rebuilt_empty.empty());
}

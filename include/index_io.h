#ifndef RXL_INDEX_IO_H
#define RXL_INDEX_IO_H
#include "hub_label.h"
#include <string>
namespace rxl {
class IndexIO {
public:
  // Portable little-endian binary format. Includes labels and both rank maps.
  static void export_binary(const LabelingResult &, const std::string &path);
  static LabelingResult import_binary(const std::string &path);
};
} // namespace rxl
#endif

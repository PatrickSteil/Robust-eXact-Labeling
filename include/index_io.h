#ifndef RXL_INDEX_IO_H
#define RXL_INDEX_IO_H
#include "hub_label.h"
#include <string>
namespace rxl {
// How a label's hub-id deltas and distances are byte-packed on disk.
enum class LabelEncoding {
  // Section 4.1's scheme: deltas as a fixed-width 1-byte block followed by
  // a fixed-width 4-byte block (split at the first delta that doesn't fit
  // in a byte), distances at a whole-file fixed width (1/2/4 bytes). Fast,
  // branch-light decoding; the paper's own choice over full variable-length
  // coding for exactly that reason.
  TwoBlockDelta,
  // Every delta and every distance individually byte-packed as an unsigned
  // LEB128 varint: 7 payload bits per byte, MSB set to say "one more byte
  // follows". No fixed width to pick, and no need for two size classes --
  // small values (the common case for both deltas and, on many graphs,
  // distances) cost one byte regardless of how large other entries in the
  // same label are. Trades a per-entry length-dependent decode loop for
  // usually-smaller files; TwoBlockDelta remains the default because query
  // time, not export size, is this project's priority.
  Varint,
};
class IndexIO {
public:
  // Portable little-endian binary format. Includes labels and both rank
  // maps. Files written with encoding == TwoBlockDelta (the default) are
  // byte-for-byte what this function always produced; import_binary()
  // detects the encoding from the file itself, so callers never pass it in.
  static void export_binary(const LabelingResult &, const std::string &path,
                            LabelEncoding encoding = LabelEncoding::TwoBlockDelta);
  static LabelingResult import_binary(const std::string &path);
};
} // namespace rxl
#endif

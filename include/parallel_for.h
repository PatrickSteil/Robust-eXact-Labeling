#ifndef RXL_PARALLEL_FOR_H
#define RXL_PARALLEL_FOR_H
#include <algorithm>
#include <cstddef>

#include "thread_pool.h"

namespace rxl {

// Splits [0, n) into `requested_threads` contiguous chunks and runs body(lo,
// hi) for each on the shared process-wide ThreadPool, instead of spawning
// new OS threads per call.
template <typename Body>
void parallel_for(std::size_t n, std::size_t requested_threads, Body body) {
  const std::size_t threads =
      std::max<std::size_t>(1, std::min(requested_threads, n));
  if (threads <= 1) {
    body(std::size_t{0}, n);
    return;
  }
  const std::size_t chunk = (n + threads - 1) / threads;
  ThreadPool::instance().run(threads, [&](std::size_t t) {
    const std::size_t lo = t * chunk, hi = std::min(n, lo + chunk);
    if (lo < hi)
      body(lo, hi);
  });
}
} // namespace rxl
#endif

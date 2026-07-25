#ifndef RXL_PARALLEL_FOR_H
#define RXL_PARALLEL_FOR_H
#include <algorithm>
#include <cstddef>
#include <thread>
#include <vector>
namespace rxl {

template <typename Body>
void parallel_for(std::size_t n, std::size_t requested_threads, Body body) {
  const std::size_t threads =
      std::max<std::size_t>(1, std::min(requested_threads, n));
  if (threads <= 1) {
    body(std::size_t{0}, n);
    return;
  }
  const std::size_t chunk = (n + threads - 1) / threads;
  std::vector<std::thread> workers;
  workers.reserve(threads - 1);
  for (std::size_t t = 1; t < threads; ++t) {
    const std::size_t lo = t * chunk, hi = std::min(n, lo + chunk);
    if (lo < hi) workers.emplace_back([body, lo, hi] { body(lo, hi); });
  }
  body(std::size_t{0}, std::min(n, chunk));
  for (auto& w : workers) w.join();
}
}  // namespace rxl
#endif

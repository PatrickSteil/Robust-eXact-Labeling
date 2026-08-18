#ifndef RXL_THREAD_POOL_H
#define RXL_THREAD_POOL_H
#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <exception>
#include <functional>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace rxl {

// A small persistent worker-thread pool. Workers are created once (process
// lifetime) and parked on a condition variable between jobs, which avoids
// the OS thread-creation/teardown cost of spawning std::thread/std::async
// on every parallel region -- important here because SamPG dispatches many
// short-lived parallel batches over the course of building an order.
//
// ThreadPool::instance() returns a process-wide pool sized to
// hardware_concurrency(). run(count, f) dispatches f(i) for i in
// [0, count) across the pool using a shared atomic cursor, so imbalanced
// per-task costs (e.g. sample trees of very different sizes) get
// dynamically load-balanced instead of statically striped. The calling
// thread also participates as a worker, so run() with count == 1 or when
// called with no other pool activity has no extra synchronization cost
// beyond a single atomic increment.
class ThreadPool {
public:
  static ThreadPool &instance() {
    static ThreadPool pool(
        std::max<unsigned>(1, std::thread::hardware_concurrency()));
    return pool;
  }

  explicit ThreadPool(std::size_t num_threads) {
    workers_.reserve(num_threads);
    for (std::size_t i = 0; i < num_threads; ++i)
      workers_.emplace_back([this] { worker_loop(); });
  }

  ~ThreadPool() {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      stop_ = true;
    }
    queue_cv_.notify_all();
    for (auto &w : workers_)
      if (w.joinable())
        w.join();
  }

  ThreadPool(const ThreadPool &) = delete;
  ThreadPool &operator=(const ThreadPool &) = delete;

  std::size_t size() const { return workers_.size(); }

  // Runs f(i) for every i in [0, count), blocking until all calls finish.
  // f may be called from the invoking thread and/or pool worker threads.
  template <class F> void run(std::size_t count, F &&f) {
    if (count == 0)
      return;
    if (count == 1 || workers_.empty()) {
      for (std::size_t i = 0; i < count; ++i)
        f(i);
      return;
    }

    const std::size_t helpers = std::min(count, workers_.size());
    // active counts in-flight *participants* (job() invocations), not
    // completed tasks: a participant only stops touching `next`/`count`
    // once its loop below observes idx >= count and exits. Waiting on task
    // completion instead (e.g. a "remaining tasks" counter) would let run()
    // return -- and destroy `next`/`count`/f's captures -- while another
    // participant thread is still mid-loop about to touch them, which is a
    // real stack-use-after-return race, not just a theoretical one.
    std::atomic<std::size_t> active{helpers + 1};
    std::atomic<std::size_t> next{0};
    std::mutex done_mutex;
    std::condition_variable done_cv;
    std::mutex error_mutex;
    std::exception_ptr error;

    auto job = [&]() {
      for (;;) {
        const std::size_t idx = next.fetch_add(1, std::memory_order_relaxed);
        if (idx >= count)
          break;
        try {
          f(idx);
        } catch (...) {
          std::lock_guard<std::mutex> lock(error_mutex);
          if (!error)
            error = std::current_exception();
        }
      }
      if (active.fetch_sub(1, std::memory_order_acq_rel) == 1) {
        std::lock_guard<std::mutex> lock(done_mutex);
        done_cv.notify_all();
      }
    };

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      for (std::size_t i = 0; i < helpers; ++i)
        tasks_.push(job);
    }
    queue_cv_.notify_all();

    // The calling thread also drains work instead of just waiting, which
    // means small jobs (e.g. count <= workers_.size()) complete without
    // ever blocking on the condition variable below.
    job();

    {
      std::unique_lock<std::mutex> lock(done_mutex);
      done_cv.wait(lock, [&] { return active.load() == 0; });
    }
    if (error)
      std::rethrow_exception(error);
  }

private:
  void worker_loop() {
    for (;;) {
      std::function<void()> task;
      {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        queue_cv_.wait(lock, [this] { return stop_ || !tasks_.empty(); });
        if (stop_ && tasks_.empty())
          return;
        task = std::move(tasks_.front());
        tasks_.pop();
      }
      task();
    }
  }

  std::vector<std::thread> workers_;
  std::queue<std::function<void()>> tasks_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  bool stop_ = false;
};

} // namespace rxl
#endif

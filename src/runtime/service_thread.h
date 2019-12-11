#pragma once

#include <atomic>
#include <chrono>
#include <functional>
#include <thread>

namespace robocar {
namespace runtime {

/**
 * A thread that loops at a fixed rate.
 */
class ServiceThread {
 public:
  /**
   * Constructs a service thread.
   *
   * @param func      The function to be called.
   * @param rateHz    The frequency at which the function should be called.
   * @param callNow   Whether the function should be called right away (before
   *                  sleeping).
   */
  ServiceThread(std::function<void()> func, size_t rateHz,
                bool callImmediately = false);

  // On destruction, the thread is stopped and joined.
  ~ServiceThread();

  // Non-copyable and non-movable
  ServiceThread(const ServiceThread&) = delete;
  ServiceThread(ServiceThread&&) = delete;
  ServiceThread& operator=(const ServiceThread&) = delete;
  ServiceThread& operator=(ServiceThread&&) = delete;

 private:
  std::thread thread_;
  std::function<void()> func_;
  const std::chrono::microseconds sleepDuration_;
  std::atomic<bool> running_{true};
  const bool callImmediately_{false};

  void run();
  void callAndCatch();
};

}  // namespace runtime
}  // namespace robocar

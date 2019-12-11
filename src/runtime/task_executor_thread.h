#pragma once

#include <functional>

#include "runtime/mpsc_queue.h"
#include "runtime/service_thread.h"

namespace robocar {
namespace runtime {

/**
 * Thread that provides the capability of executing arbitrary functions.
 * Note: The thread only wakes up at the provided rate.
 */
class TaskExecutorThread {
 public:
  static constexpr size_t kDefaultQueueSize = 1000;

  /**
   * Constructs the task executor thread.
   *
   * @param rateHz      The rate (in hertz) at which the thread will wake up.
   * @param queueSize   The size of the queue powering the thread.
   */
  explicit TaskExecutorThread(size_t rateHz,
                              size_t queueSize = kDefaultQueueSize);

  /**
   * Adds a task to be executed by this executor.
   *
   * @param task  The task to be executed.
   *
   * @return True iff the task was successfully scheduled.
   */
  bool addTask(std::function<void()> task) noexcept;

 private:
  MPSCQueue<std::function<void()>> queue_;
  ServiceThread thread_;

  void run();
};

}  // namespace runtime
}  // namespace robocar

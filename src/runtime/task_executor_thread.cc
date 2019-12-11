#include "runtime/task_executor_thread.h"

#include <functional>
#include <iostream>

#include "runtime/mpsc_queue.h"
#include "runtime/service_thread.h"

namespace robocar {
namespace runtime {

TaskExecutorThread::TaskExecutorThread(size_t rateHz, size_t queueSize)
    : queue_(queueSize), thread_([this]() { run(); }, rateHz) {}

bool TaskExecutorThread::addTask(std::function<void()> task) noexcept {
  return queue_.push(std::move(task));
}

void TaskExecutorThread::run() {
  std::function<void()> task;
  while (queue_.pop(task)) {
    try {
      task();
    } catch (const std::exception& ex) {
      std::cerr
          << "Exception caught when executing a task in TaskExecutorThread:"
          << ex.what() << std::endl;
    }
  }
}

}  // namespace runtime
}  // namespace robocar

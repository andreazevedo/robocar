#include "runtime/service_thread.h"

#include <atomic>
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>

namespace robocar {
namespace runtime {

namespace {
constexpr size_t kMicrosInSec = 1000000;
}

ServiceThread::ServiceThread(std::function<void()> func, size_t rateHz,
                             bool callNow)
    : thread_{[this]() { run(); }},
      func_{std::move(func)},
      sleepDuration_{kMicrosInSec / rateHz},
      callImmediately_{callNow} {}

ServiceThread::~ServiceThread() {
  running_.store(false, std::memory_order_relaxed);
  thread_.join();
}

void ServiceThread::callAndCatch() {
  try {
    func_();
  } catch (const std::exception& ex) {
    std::cerr << "Exception caught in ServiceThread callback: " << ex.what()
              << std::endl;
  }
}

void ServiceThread::run() {
  if (!callImmediately_) {
    std::this_thread::sleep_for(sleepDuration_);
  }

  while (running_.load(std::memory_order_relaxed)) {
    auto start = std::chrono::steady_clock::now();
    callAndCatch();
    auto end = std::chrono::steady_clock::now();

    auto actualSleepDuration =
        sleepDuration_ -
        std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::this_thread::sleep_for(actualSleepDuration);
  }
}
}  // namespace runtime
}  // namespace robocar

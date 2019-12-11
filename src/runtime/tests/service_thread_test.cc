#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "runtime/service_thread.h"

using robocar::runtime::ServiceThread;

TEST(ServiceThreadTest, basic) {
  std::atomic<size_t> callCount = 0;
  {
    ServiceThread st([&callCount]() { callCount++; }, 1000 /* hz */);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  EXPECT_GT(callCount, 0);
}

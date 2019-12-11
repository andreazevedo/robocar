#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "runtime/task_executor_thread.h"

using robocar::runtime::TaskExecutorThread;

TEST(TaskExecutorThreadTest, basic) {
  std::atomic<size_t> val1{0};
  std::atomic<size_t> val2{0};

  TaskExecutorThread executor(1000 /* rateHz */);
  executor.addTask([&val1]() { val1 = 17; });
  executor.addTask([&val2]() { val2 = 71; });

  // Give some time for the thread to run
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  EXPECT_EQ(17, val1.load());
  EXPECT_EQ(71, val2.load());
}


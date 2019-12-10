#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "runtime/mpsc_queue.h"

using robocar::runtime::MPSCQueue;

namespace {
struct Item {
  uint64_t int1;
  std::string str1;
  uint64_t int2;
};

void expectEqual(const Item& lhs, const Item& rhs) {
  EXPECT_EQ(lhs.int1, rhs.int1);
  EXPECT_EQ(lhs.str1, rhs.str1);
  EXPECT_EQ(lhs.int2, rhs.int2);
}

}  // namespace

TEST(MPSCQueueTest, basic) {
  MPSCQueue<Item> queue(5 /* capacity */);

  EXPECT_TRUE(queue.push(Item{1, "100", 10}));
  EXPECT_TRUE(queue.push(Item{2, "200", 20}));
  EXPECT_TRUE(queue.push(Item{3, "300", 30}));
  EXPECT_TRUE(queue.push(Item{4, "400", 40}));
  EXPECT_TRUE(queue.push(Item{5, "500", 50}));

  EXPECT_FALSE(queue.push(Item{6, "600", 60}));

  Item item;
  EXPECT_TRUE(queue.pop(item));
  expectEqual(Item{1, "100", 10}, item);
  EXPECT_TRUE(queue.pop(item));
  expectEqual(Item{2, "200", 20}, item);

  EXPECT_TRUE(queue.push(Item{7, "700", 70}));
  EXPECT_TRUE(queue.push(Item{8, "800", 80}));

  EXPECT_TRUE(queue.pop(item));
  expectEqual(Item{3, "300", 30}, item);
  EXPECT_TRUE(queue.pop(item));
  expectEqual(Item{4, "400", 40}, item);
  EXPECT_TRUE(queue.pop(item));
  expectEqual(Item{5, "500", 50}, item);
  EXPECT_TRUE(queue.pop(item));
  expectEqual(Item{7, "700", 70}, item);
  EXPECT_TRUE(queue.pop(item));
  expectEqual(Item{8, "800", 80}, item);

  EXPECT_FALSE(queue.pop(item));
}

TEST(MPSCQueueTest, threads) {
  constexpr size_t kNumProducerIterations = 5000;
  MPSCQueue<Item> queue(kNumProducerIterations * 2 /* capacity */);

  auto producerFn = [&queue](size_t initialVal1) {
    size_t val1 = initialVal1;
    for (int i = 0; i < kNumProducerIterations; ++i) {
      Item item{val1, "my custom really long string that is not a small string",
                val1 * 10};
      EXPECT_TRUE(queue.push(std::move(item)));
      val1 += 2;
    }
  };

  size_t numItemsPopped = 0;
  auto consumerFn = [&queue, &numItemsPopped]() {
    Item item;
    while (numItemsPopped < kNumProducerIterations * 2) {
      if (!queue.pop(item)) {
        continue;
      }
      ++numItemsPopped;
      EXPECT_EQ("my custom really long string that is not a small string",
                item.str1);
      EXPECT_EQ(item.int2, item.int1 * 10);
    }
  };

  std::thread oddProducer([&producerFn]() { producerFn(1); });
  std::thread consumer(consumerFn);
  std::thread evenProducer([&producerFn]() { producerFn(2); });

  oddProducer.join();
  evenProducer.join();
  consumer.join();

  EXPECT_EQ(kNumProducerIterations * 2, numItemsPopped);
}

#pragma once

#include <atomic>
#include <cstdlib>
#include <new>
#include <vector>

namespace robocar {
namespace runtime {

constexpr size_t kHardwareDestructiveInterferenceSize = 64;

/**
 * A lock-free multi produce, single consumer queue implementation.
 */
template <class T>
class MPSCQueue {
 public:
  explicit MPSCQueue(size_t capacity) : queue_{capacity + 1} {}

  /**
   * Inserts an item into the queue.
   *
   * @param item  Item to insert.
   *
   * @return True if insertion was successful. False otherwise.
   */
  bool push(T item) noexcept {
    size_t back = back_.load(std::memory_order_relaxed);

    while (true) {
      size_t front = front_.load(std::memory_order_relaxed);
      size_t dist = getDistance(front, back);

      if (dist >= capacity()) {
        // Queue is full
        return false;
      }

      if (!back_.compare_exchange_strong(back, back + 1,
                                         std::memory_order_relaxed)) {
        // Try again
        continue;
      }

      // We we successfully reserved our spot on the queue, add the item.
      size_t idx = getIndex(back + 1);
      queue_[idx].data = std::move(item);
      queue_[idx].ready.store(true, std::memory_order_release);
      return true;
    }
  }

  bool pop(T& item) noexcept {
    size_t front = front_.load(std::memory_order_relaxed);
    size_t back = back_.load(std::memory_order_relaxed);
    size_t dist = getDistance(front, back);

    if (dist == 0) {
      // Queue is empty
      return false;
    }

    size_t idx = getIndex(front + 1);
    if (!queue_[idx].ready.load(std::memory_order_acquire)) {
      return false;
    }
    queue_[idx].ready.store(false, std::memory_order_relaxed);
    item = std::move(queue_[idx].data);
    front_.store(front + 1, std::memory_order_relaxed);
    return true;
  }

  const size_t capacity() const noexcept { return queue_.size() - 1; }

 private:
  struct Item {
    T data;
    std::atomic<bool> ready{false};
  };

  // The actual queue
  std::vector<Item> queue_;

  // Last index read from
  std::atomic<size_t> front_{0};

  // Last index written to
  alignas(kHardwareDestructiveInterferenceSize) std::atomic<size_t> back_{0};

  size_t getDistance(size_t front, size_t back) const {
    int64_t distance = back - front;
    if (distance < 0) {
      return 0;
    }
    return size_t(distance);
  }

  size_t getIndex(size_t absoluteIndex) const noexcept {
    return absoluteIndex % capacity();
  }
};

}  // namespace runtime
}  // namespace robocar

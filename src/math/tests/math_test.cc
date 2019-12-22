#include <gtest/gtest.h>

#include "math/floating_point.h"

using namespace robocar;

TEST(Math, equals) {
  EXPECT_TRUE(math::equals(2.75, 2.75));
  EXPECT_FALSE(math::equals(2.75, 2.76));
}

TEST(Math, isZero) {
  EXPECT_TRUE(math::isZero(0.0));
  EXPECT_FALSE(math::isZero(0.001));
}

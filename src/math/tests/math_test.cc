#include <gtest/gtest.h>

#include "math/floating_point.h"
#include "math/statistics.h"

using namespace robocar;

TEST(Math, equals) {
  EXPECT_TRUE(math::equals(2.75, 2.75));
  EXPECT_FALSE(math::equals(2.75, 2.76));
}

TEST(Math, isZero) {
  EXPECT_TRUE(math::isZero(0.0));
  EXPECT_FALSE(math::isZero(0.001));
}

TEST(Math, stdDeviation) {
  constexpr double kEpsilon = 1e-6;

  std::vector<double> list = {1.0, 1.0, 1.0};
  double stdDev = math::stdDeviation(list.begin(), list.end());
  EXPECT_NEAR(0.0, stdDev, kEpsilon);

  list = {1.0, 2.0, 3.0};
  stdDev = math::stdDeviation(list.begin(), list.end());
  EXPECT_NEAR(0.816497, stdDev, kEpsilon);
}

TEST(Math, stdDeviation_complex) {
  struct MyType {
    int a;
    double b;
  };
  constexpr double kEpsilon = 1e-6;

  std::vector<MyType> list = {MyType{1, 1.0}, MyType{2, 1.0}, MyType{3, 1.0}};
  double stdDev = math::stdDeviation(list.begin(), list.end(),
                                     [](MyType item) { return item.b; });
  EXPECT_NEAR(0.0, stdDev, kEpsilon);

  list = {MyType{1, 1.0}, MyType{2, 2.0}, MyType{3, 3.0}};
  stdDev = math::stdDeviation(list.begin(), list.end(),
                              [](MyType item) { return item.b; });
  EXPECT_NEAR(0.816497, stdDev, kEpsilon);
}

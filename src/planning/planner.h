#pragma once

#include "perception/lane_detector.h"
#include "planning/plan.h"

namespace robocar {
namespace planning {

class Planner {
 public:
  Planner() noexcept;

  /**
   * Calculate the next action of the car.
   *
   * @param lane  Information about the lane the car is in.
   *              @see perception::Lane
   */
  Plan calculateRoute(perception::Lane lane);

  /**
   * Calculate the next action of the car.
   *
   * @param laneFinalSlope  The average of the slope of the lines of the current
   *                        lane.
   *                        @see perception::LaneDetector::getAverageSlope()
   */
  Plan calculateRouteLegacy(double laneLinesSlopeSum);

 private:
  size_t numIterations_{0};
  Plan plan_;
};

}  // namespace planning
}  // namespace robocar


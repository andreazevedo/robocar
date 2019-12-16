#pragma once

#include "planning/planning_result.h"

namespace robocar {
namespace planning {
class Planner {
 public:
  /**
   * Calculate the next action of the car.
   *
   * @param laneFinalSlope  The average of the slope of the lines of the current
   *                        lane.
   *                        @see perception::LaneDetector::getAverageSlope()
   */
  PlanningResult calculateRoute(double laneLinesSlopeSum) const;
};
}  // namespace planning
}  // namespace robocar


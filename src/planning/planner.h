#pragma once

#include "perception/lane_detector.h"
#include "planning/planning_result.h"

namespace robocar {
namespace planning {
class Planner {
 public:
  /**
   * Calculate the next action of the car.
   *
   * @param lane  Information about the lane the car is in.
   *              @see perception::Lane
   */
  PlanningResult calculateRoute(perception::Lane lane) const;

  /**
   * Calculate the next action of the car.
   *
   * @param laneFinalSlope  The average of the slope of the lines of the current
   *                        lane.
   *                        @see perception::LaneDetector::getAverageSlope()
   */
  PlanningResult calculateRouteLegacy(double laneLinesSlopeSum) const;
};
}  // namespace planning
}  // namespace robocar


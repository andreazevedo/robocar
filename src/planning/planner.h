#pragma once

#include <vector>

#include "perception/lane.h"
#include "perception/obstacles.h"
#include "planning/plan.h"

namespace robocar {
namespace planning {

class Planner {
 public:
  Planner() noexcept;

  /**
   * Calculate the next action of the car, based only on a lane.
   *
   * @param lane  Information about the lane the car is in.
   *              @see perception::Lane
   */
  Plan calculateRoute(perception::Lane lane);

  /**
   * Calculate the next action of the car, based on all obstacles.
   *
   * @param obstacles   The obstacles detected by perception.
   *                    @see perception::Obstacles.
   */
  Plan calculateRoute(perception::Obstacles obstacles);

 private:
  size_t numIterations_{0};

  // for the planner that deals with sharp curves.
  Plan lastPlan_{0.0, 0.0};
  size_t lastPlanCount_{0};
  Plan calculateRouteWithSharpCurves(perception::Lane lane);
};

}  // namespace planning
}  // namespace robocar


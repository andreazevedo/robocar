#pragma once

#include <vector>

#include "perception/lane.h"
#include "perception/obstacles.h"
#include "planning/plan.h"
#include "planning/stop_sign_handler.h"

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
  Plan calculateRoute(const perception::Lane& lane);

  /**
   * Calculate the next action of the car, based on all obstacles.
   *
   * @param obstacles   The obstacles detected by perception.
   *                    @see perception::Obstacles.
   */
  Plan calculateRoute(const perception::Obstacles& obstacles);

 private:
  // Number of iterations of planner so far.
  size_t numIterations_{0};

  // Handles stop signs.
  StopSignHandler stopSignHandler_;

  // for the planner that deals with sharp curves.
  Plan lastPlan_{0.0, 0.0};
  size_t lastPlanCount_{0};

  // Similar to calculateRoute(Lane), but optmized for sharp curves.
  Plan calculateRouteWithSharpCurves(const perception::Lane& lane);
};

}  // namespace planning
}  // namespace robocar

#include "planning/planner.h"

#include <cmath>

#include "planning/planning_result.h"

namespace robocar {
namespace planning {

PlanningResult Planner::calculateRoute(double laneFinalSlope) const {
  double angle = laneFinalSlope * 90;
  angle *= -1;
  angle = std::min(90.0, angle);
  angle = std::max(-90.0, angle);

  PlanningResult result;
  result.throttle = 0.40;
  if (::abs(angle) > 38) {
    result.throttle = 0.50;
  }
  result.steeringAngle = angle;
  return result;
}

}  // namespace planning
}  // namespace robocar


#include "planning/planner.h"

#include <cmath>

#include "perception/lane_detector.h"
#include "planning/planning_result.h"

namespace robocar {
namespace planning {

namespace {

double getThrottle(double originalThrottle, double steeringAngle) {
  double throttle = originalThrottle;

  if (::abs(steeringAngle) > 30.0) {
    throttle += 0.1;
  }
  if (::abs(steeringAngle) > 50.0) {
    throttle += 0.1;
  }
  if (::abs(steeringAngle) > 70.0) {
    throttle += 0.1;
  }

  return std::min(throttle, 1.0);
}

}

PlanningResult Planner::calculateRoute(perception::Lane lane) const {
  PlanningResult result;

  if (!lane.left && !lane.right) {
    // no lane detected - can create a plan
    return result;
  }

  double angle;
  if (!lane.left) {
    angle = 60.0;
  } else if (!lane.right) {
    angle = -60.0;
  } else {
    angle = (lane.right->slope + lane.left->slope) * 90;
  }
  angle *= -1;
  angle = std::min(90.0, angle);
  angle = std::max(-90.0, angle);

  // throttle
  result.throttle = getThrottle(0.4, angle);
  result.steeringAngle = angle;
  return result;
}

PlanningResult Planner::calculateRouteLegacy(double laneFinalSlope) const {
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


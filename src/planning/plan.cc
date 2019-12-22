#include "planning/plan.h"

#include "math/floating_point.h"

namespace robocar {
namespace planning {

Plan::Plan(double throttle, double steeringAngle)
    : throttle_(throttle), steeringAngle_(steeringAngle) {}

/*  static */ Plan Plan::emptyPlan() {
  static Plan plan(0.0, 0.0);
  return plan;
}

bool Plan::isEmpty() const {
  return math::isZero(throttle_) && math::isZero(steeringAngle_);
}

}  // namespace planning
}  // namespace robocar

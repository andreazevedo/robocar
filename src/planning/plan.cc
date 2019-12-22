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

Plan operator+(const Plan& lhs, const Plan& rhs) {
  return Plan(lhs.throttle() + rhs.throttle(),
              lhs.steeringAngle() + rhs.steeringAngle());
}
Plan operator*(const Plan& plan, double mult) {
  return Plan(plan.throttle() * mult, plan.steeringAngle() * mult);
}
Plan operator/(const Plan& plan, double div) {
  return Plan(plan.throttle() / div, plan.steeringAngle() / div);
}

}  // namespace planning
}  // namespace robocar

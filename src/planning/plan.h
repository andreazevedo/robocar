#pragma once

#include <string>

namespace robocar {
namespace planning {

class Plan {
 public:
  Plan(double throttle, double steeringAngle);

  double throttle() const { return throttle_; }
  double steeringAngle() const { return steeringAngle_; }

  bool isEmpty() const;

  static Plan emptyPlan();

 private:
  double throttle_{0.0};
  double steeringAngle_{0.0};
};

Plan operator+(const Plan& lhs, const Plan& rhs);
Plan operator*(const Plan& lhs, double mult);
Plan operator/(const Plan& plan, double div);

}  // namespace planning
}  // namespace robocar

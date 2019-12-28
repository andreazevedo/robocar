#include "planning/planner.h"

#include <cmath>
#include <iostream>
#include <vector>

#include "math/floating_point.h"
#include "perception/lane.h"
#include "perception/obstacles.h"
#include "planning/plan.h"

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

Plan calculateRouteImpl(const perception::Lane& lane) {
  if (!lane.left && !lane.right) {
    // no lane detected - can't create a plan
    return Plan::emptyPlan();
  }

  double angle;
  if (!lane.left) {
    angle = 80.0;
  } else if (!lane.right) {
    angle = -80.0;
  } else {
    angle = (lane.right->slope + lane.left->slope) * 90;
  }
  angle *= -1;
  angle = std::min(90.0, angle);
  angle = std::max(-90.0, angle);

  // throttle
  return Plan(getThrottle(0.4, angle), angle);
}

bool isHighConfidenceLaneLine(const perception::LaneLine& laneLine) {
  constexpr double kMaxStdDev = 0.35;
  constexpr size_t kMinLines = 2;
  return laneLine.slopeStdDeviation <= kMaxStdDev &&
         laneLine.numLines >= kMinLines;
}

bool isHighConfidenceLane(const perception::Lane& lane) {
  if (!lane.left || !lane.right) {
    return false;
  }

  return isHighConfidenceLaneLine(*lane.left) &&
         isHighConfidenceLaneLine(*lane.right);
}

}  // namespace

Planner::Planner() noexcept {}

Plan Planner::calculateRoute(perception::Lane lane) {
  ++numIterations_;
  return calculateRouteImpl(lane);
}

Plan Planner::calculateRouteWithSharpCurves(perception::Lane lane) {
  ++numIterations_;
  auto plan = calculateRouteImpl(lane);
  if (::fabs(plan.steeringAngle()) >= 70.0) {
    if (math::equals(lastPlan_.steeringAngle(), plan.steeringAngle())) {
      if (++lastPlanCount_ == 2) {
        plan = Plan{0.4, 0.0};
        lastPlanCount_ = 0;
      }
    }
  } else {
    lastPlanCount_ = 0;
  }
  lastPlan_ = plan;
  return plan;
}

Plan Planner::calculateRoute(perception::Obstacles obstacles) {
  return calculateRouteWithSharpCurves(obstacles.lane);
}

}  // namespace planning
}  // namespace robocar

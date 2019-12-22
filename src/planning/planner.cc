#include "planning/planner.h"

#include <cmath>
#include <iostream>
#include <vector>

#include "math/floating_point.h"
#include "perception/lane_detector.h"
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

Plan Planner::calculateRouteExperimental(perception::Lane lane) {
  ++numIterations_;
  if (isHighConfidenceLane(lane)) {
    backupLane_ = lane;
    backupLaneIteration_ = numIterations_;
  }

  Plan plan = calculateRouteImpl(lane);
  if (plan.isEmpty() && isBackupLaneValid()) {
    std::cout << "Using backup lane!" << std::endl;
    plan = calculateRouteImpl(backupLane_);
  }

  // update backup lane
  if (isBackupLaneValid() && !plan.isEmpty()) {
    backupLane_.left->slope += ((plan.steeringAngle() / 90.0) * 0.2);
    backupLane_.right->slope += ((plan.steeringAngle() / 90.0) * 0.2);
  }
  return plan;
}

Plan Planner::calculateRouteLegacy(double laneFinalSlope) {
  ++numIterations_;

  double angle = laneFinalSlope * 90;
  angle *= -1;
  angle = std::min(90.0, angle);
  angle = std::max(-90.0, angle);

  double throttle = 0.40;
  if (::abs(angle) > 38) {
    throttle = 0.50;
  }
  return Plan(throttle, angle);
}

}  // namespace planning
}  // namespace robocar


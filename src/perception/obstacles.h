#pragma once

#include <vector>

#include "perception/agent.h"
#include "perception/lane.h"

namespace robocar {
namespace perception {

/**
 * Output of the perception service.
 */
struct Obstacles {
  Lane lane;
  std::vector<Agent> agents;
};

}  // namespace perception
}  // namespace robocar

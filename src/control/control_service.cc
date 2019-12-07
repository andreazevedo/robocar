#include "control/control_service.h"

namespace robocar {
namespace control {

ControlService::ControlService() noexcept
    : frontLeft_{1 /* motorId */},
      frontRight_{2 /* motorId */},
      rearLeft_{3 /* motorId */},
      rearRight_{1 /* motorId */} {}

}  // namespace control
}  // namespace robocar

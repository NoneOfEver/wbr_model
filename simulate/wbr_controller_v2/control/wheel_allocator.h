#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_CONTROL_WHEEL_ALLOCATOR_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_CONTROL_WHEEL_ALLOCATOR_H_

#include <mujoco/mujoco.h>

#include "../common/controller_types.h"

namespace wbr::v2 {

struct WheelAllocationInput {
  double balance_torque = 0.0;
  double yaw_torque = 0.0;
  bool grounded[2] = {};
  WbrContactSafetyState contact_state = WbrContactSafetyState::kAirborne;
};

struct WheelAllocationOutput {
  double actuator_torque[2] = {};
  double applied_yaw_torque = 0.0;
};

WheelAllocationOutput AllocateWheelTorque(const WheelAllocationInput& input);
void WriteWheelActuators(mjData* data, int actuator_1, int actuator_2,
                         const WheelAllocationOutput& output);

}  // namespace wbr::v2

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_CONTROL_WHEEL_ALLOCATOR_H_

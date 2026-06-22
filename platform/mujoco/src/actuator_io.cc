#include "wbr/platform/mujoco/actuator_io.h"

namespace wbr::v2 {

void WriteMotorCommands(
    mjData* data, const int actuator_ids[control::kMotorCount],
    const control::ControlOutput& output) {
  if (!data || !actuator_ids) return;
  constexpr double kActuatorSign[control::kMotorCount] = {
      1.0, 1.0, -1.0, -1.0, 1.0, -1.0};
  for (int motor = 0; motor < control::kMotorCount; ++motor) {
    const int actuator = actuator_ids[motor];
    if (actuator < 0) continue;
    data->ctrl[actuator] = output.motor[motor].enabled
        ? kActuatorSign[motor] * output.motor[motor].torque
        : 0.0;
  }
}

}  // namespace wbr::v2

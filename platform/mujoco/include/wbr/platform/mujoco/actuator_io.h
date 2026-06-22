#ifndef WBR_PLATFORM_MUJOCO_ACTUATOR_IO_H_
#define WBR_PLATFORM_MUJOCO_ACTUATOR_IO_H_

#include <mujoco/mujoco.h>

#include "wbr/control/controller_io.h"

namespace wbr::v2 {

void WriteMotorCommands(
    mjData* data, const int actuator_ids[control::kMotorCount],
    const control::ControlOutput& output);

}  // namespace wbr::v2

#endif  // WBR_PLATFORM_MUJOCO_ACTUATOR_IO_H_

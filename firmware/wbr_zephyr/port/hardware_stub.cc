#include "wbr/firmware/hardware_interface.h"

#include <zephyr/kernel.h>

namespace wbr::firmware {

bool InitializeHardware() {
  return false;
}

bool LoadRobotParameters(control::RobotParameters&) {
  return false;
}

bool CaptureSnapshot(HardwareSnapshot&) {
  return false;
}

bool ReadOperatorCommand(control::ControlCommand&) {
  return false;
}

bool SubmitMotorCommands(
    const control::MotorCommand (&)[control::kMotorCount]) {
  return false;
}

void DisableAllMotors() {}

std::uint64_t MonotonicTimeUs() {
  return static_cast<std::uint64_t>(k_uptime_get()) * 1000U;
}

}  // namespace wbr::firmware

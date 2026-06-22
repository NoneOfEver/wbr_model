#ifndef WBR_FIRMWARE_HARDWARE_INTERFACE_H_
#define WBR_FIRMWARE_HARDWARE_INTERFACE_H_

#include <cstdint>

#include "wbr/control/controller_io.h"
#include "wbr/control/robot_observer.h"

namespace wbr::firmware {

struct ImuRawSample {
  double angular_velocity[3] = {};
  double acceleration[3] = {};
  std::uint64_t timestamp_us = 0;
  bool valid = false;
};

struct HardwareSnapshot {
  ImuRawSample imu;
  control::MotorFeedback motor[control::kMotorCount];
  double wheel_contact_confidence[2] = {};
};

// Implement these functions in firmware/wbr_zephyr/port for the target board.
bool InitializeHardware();
bool LoadRobotParameters(control::RobotParameters& parameters);
bool CaptureSnapshot(HardwareSnapshot& snapshot);
bool ReadOperatorCommand(control::ControlCommand& command);
bool SubmitMotorCommands(
    const control::MotorCommand (&commands)[control::kMotorCount]);
void DisableAllMotors();
std::uint64_t MonotonicTimeUs();

}  // namespace wbr::firmware

#endif  // WBR_FIRMWARE_HARDWARE_INTERFACE_H_

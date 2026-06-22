#include <cmath>
#include <cstdio>

#include "wbr/control/contact_safety.h"
#include "wbr/control/ground_balance_controller.h"
#include "wbr/control/leg_kinematics.h"
#include "wbr/control/lqr_schedule.h"
#include "wbr/control/robot_observer.h"
#include "wbr/control/wheel_allocator.h"

namespace {

bool Finite(double value) {
  return std::isfinite(value);
}

}  // namespace

int main() {
  double gain[2][6] = {};
  wbr::v2::EvaluateLqrGain(0.18, gain);
  for (const auto& input : gain) {
    for (double value : input) {
      if (!Finite(value)) return 1;
    }
  }

  wbr::v2::LegKinematics leg;
  if (!wbr::v2::ComputeLegKinematics(
          1.8, 1.3, 0.0, 0.0, 1, leg)) {
    return 2;
  }
  const wbr::v2::LegVmcOutput vmc = wbr::v2::ComputeLegVmc(
      leg, leg.length, 35.0, 0.0, 0.0, 0.0);
  if (!Finite(vmc.axial_force) ||
      !Finite(vmc.joint_torque[0]) || !Finite(vmc.joint_torque[1])) {
    return 3;
  }

  wbr::v2::ContactSafetyMachine contact_safety;
  contact_safety.Reset();
  const bool grounded[2] = {true, true};
  wbr::v2::ContactSafetyOutput contact;
  for (int step = 0; step < 300; ++step) {
    contact = contact_safety.Update(grounded, 0.0, 0.0, 0.001);
  }
  if (contact.state != WbrContactSafetyState::kDualSupport) return 4;

  wbr::v2::WheelAllocationInput allocation_input;
  allocation_input.balance_torque = 1.0;
  allocation_input.yaw_torque = 2.0;
  allocation_input.grounded[0] = allocation_input.grounded[1] = true;
  allocation_input.contact_state = WbrContactSafetyState::kDualSupport;
  const wbr::v2::WheelAllocationOutput allocation =
      wbr::v2::AllocateWheelTorque(allocation_input);
  if (!Finite(allocation.actuator_torque[0]) ||
      !Finite(allocation.actuator_torque[1])) {
    return 5;
  }

  wbr::v2::GroundBalanceController controller;
  controller.Reset();
  controller.InitializeLegTarget(leg.length, 0.0);
  controller.SetVelocityCommand(0.0, 0.2);
  wbr::v2::GroundBalanceInput controller_input;
  controller_input.observation_valid = true;
  controller_input.yaw_plant_enabled = true;
  controller_input.leg[0] = controller_input.leg[1] = leg;
  controller_input.leg_valid[0] = controller_input.leg_valid[1] = true;
  controller_input.wheel_grounded[0] =
      controller_input.wheel_grounded[1] = true;
  controller_input.wheel_normal_force[0] =
      controller_input.wheel_normal_force[1] = 40.0;
  controller_input.total_mass = 8.18;
  controller_input.target_leg_length = leg.length;
  wbr::v2::GroundBalanceOutput controller_output;
  for (int step = 0; step < 400; ++step) {
    controller_output = controller.Update(controller_input);
  }
  for (const auto& motor : controller_output.actuator.motor) {
    if (!Finite(motor.torque)) return 6;
  }

  wbr::control::RobotObserver observer;
  wbr::control::RobotParameters robot_parameters;
  wbr::control::ControlInput observer_sample;
  constexpr std::uint64_t timestamp_us = 10000;
  observer_sample.dt = 0.001;
  observer_sample.imu.valid = true;
  observer_sample.imu.timestamp_us = timestamp_us;
  observer_sample.imu.acceleration[2] = robot_parameters.gravity;
  for (auto& motor : observer_sample.motor) {
    motor.valid = true;
    motor.timestamp_us = timestamp_us;
  }
  const auto index = [](wbr::control::MotorId id) {
    return static_cast<int>(id);
  };
  observer_sample.motor[index(wbr::control::MotorId::kLeftJointB)].position =
      1.3;
  observer_sample.motor[index(wbr::control::MotorId::kLeftJointD)].position =
      1.8;
  observer_sample.motor[index(wbr::control::MotorId::kRightJointB)].position =
      1.3;
  observer_sample.motor[index(wbr::control::MotorId::kRightJointD)].position =
      1.8;
  observer_sample.contact.confidence[0] =
      observer_sample.contact.confidence[1] = 1.0;
  observer.Reset();
  const wbr::control::ObservationResult upright = observer.Update(
      observer_sample, robot_parameters, timestamp_us);
  if (!upright.valid || std::fabs(upright.input.roll) > 1e-9 ||
      std::fabs(upright.input.pitch) > 1e-9) {
    return 7;
  }
  const wbr::control::ObservationResult stale = observer.Update(
      observer_sample, robot_parameters,
      timestamp_us + robot_parameters.maximum_sample_age_us + 1);
  if (stale.valid) return 8;
  observer_sample.dt = robot_parameters.maximum_control_period + 0.001;
  const wbr::control::ObservationResult late_cycle = observer.Update(
      observer_sample, robot_parameters, timestamp_us);
  if (late_cycle.valid) return 9;

  std::printf("control_core smoke test: PASS\n");
  return 0;
}

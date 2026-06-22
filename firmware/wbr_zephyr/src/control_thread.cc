#include "wbr/firmware/control_thread.h"

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "wbr/control/ground_balance_controller.h"
#include "wbr/control/robot_observer.h"
#include "wbr/firmware/hardware_interface.h"

LOG_MODULE_REGISTER(wbr_control_thread, LOG_LEVEL_INF);

namespace wbr::firmware {
namespace {

K_SEM_DEFINE(start_sem, 0, 1);
K_SEM_DEFINE(tick_sem, 0, 1);

void Tick(struct k_timer*) {
  k_sem_give(&tick_sem);
}

K_TIMER_DEFINE(control_timer, Tick, nullptr);

void DisableAndReset(control::RobotObserver& observer,
                     v2::GroundBalanceController& controller,
                     bool& active) {
  DisableAllMotors();
  if (active) {
    observer.Reset();
    controller.Reset();
    active = false;
  }
}

void ControlThreadEntry(void*, void*, void*) {
  k_sem_take(&start_sem, K_FOREVER);

  control::RobotParameters robot_parameters;
  control::RobotObserver observer;
  v2::GroundBalanceController controller;
  observer.Reset();
  controller.Reset();

  if (!LoadRobotParameters(robot_parameters)) {
    DisableAllMotors();
    LOG_ERR("robot parameters are unavailable; control thread stopped");
    return;
  }

  std::uint64_t previous_time_us = 0;
  bool active = false;
  k_timer_start(&control_timer, K_MSEC(1), K_MSEC(1));

  while (true) {
    k_sem_take(&tick_sem, K_FOREVER);
    const std::uint64_t now_us = MonotonicTimeUs();
    HardwareSnapshot snapshot;
    control::ControlCommand command;
    if (!CaptureSnapshot(snapshot) || !ReadOperatorCommand(command)) {
      DisableAndReset(observer, controller, active);
      previous_time_us = now_us;
      continue;
    }

    control::ControlInput sample;
    sample.dt = previous_time_us > 0 && now_us > previous_time_us
        ? static_cast<double>(now_us - previous_time_us) * 1e-6
        : 0.001;
    previous_time_us = now_us;
    sample.command = command;
    sample.imu.timestamp_us = snapshot.imu.timestamp_us;
    sample.imu.valid = snapshot.imu.valid;
    for (int axis = 0; axis < 3; ++axis) {
      sample.imu.angular_velocity[axis] =
          snapshot.imu.angular_velocity[axis];
      sample.imu.acceleration[axis] = snapshot.imu.acceleration[axis];
    }
    for (int motor = 0; motor < control::kMotorCount; ++motor) {
      sample.motor[motor] = snapshot.motor[motor];
    }
    sample.contact.confidence[0] = snapshot.wheel_contact_confidence[0];
    sample.contact.confidence[1] = snapshot.wheel_contact_confidence[1];

    const bool command_fresh = command.valid &&
        command.timestamp_us <= now_us &&
        now_us - command.timestamp_us <=
            robot_parameters.maximum_sample_age_us;
    if (!command.enabled || !command_fresh) {
      DisableAndReset(observer, controller, active);
      continue;
    }
    const control::ObservationResult observation =
        observer.Update(sample, robot_parameters, now_us);
    if (!observation.valid) {
      DisableAndReset(observer, controller, active);
      continue;
    }

    if (!active) {
      controller.Reset();
      controller.InitializeLegTarget(
          0.5 * (observation.input.leg[0].length +
                 observation.input.leg[1].length),
          0.0);
      active = true;
    }
    controller.SetVelocityCommand(command.linear_velocity, command.yaw_rate);
    v2::GroundBalanceInput controller_input = observation.input;
    controller_input.target_leg_length = command.leg_length;
    controller_input.target_leg_angle = command.leg_angle;
    const v2::GroundBalanceOutput output = controller.Update(controller_input);
    if (output.actuator.emergency_stop ||
        !SubmitMotorCommands(output.actuator.motor)) {
      DisableAndReset(observer, controller, active);
    }
  }
}

K_THREAD_DEFINE(control_thread_id, 8192, ControlThreadEntry,
                nullptr, nullptr, nullptr, K_PRIO_PREEMPT(1), 0, 0);

}  // namespace

void StartControlThread() {
  k_sem_give(&start_sem);
}

}  // namespace wbr::firmware

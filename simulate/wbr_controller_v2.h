#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_

#include <mujoco/mujoco.h>
#include "wbr_controller_v2/common/controller_types.h"
#include "wbr_controller_v2/control/contact_safety.h"
#include "wbr_controller_v2/control/yaw_coordinator.h"
#include "wbr_controller_v2/model/state_estimator.h"

class WbrControllerV2 {
 public:
  void Reset(const mjModel* model);
  void SyncTargetsFromState(const mjModel* model, const mjData* data,
                            double& target_leg_length, double& target_leg_angle);
  void Apply(const mjModel* model, mjData* data,
             double& target_leg_length, double& target_leg_angle);
  void SetLqrEnabled(bool enabled) { lqr_enabled_ = enabled; }
  void SetYawEnabled(bool enabled) { yaw_enabled_ = enabled; }
  void SetVelocityCommand(double linear_velocity, double yaw_rate) {
    target_linear_velocity_ = linear_velocity;
    target_yaw_rate_ = yaw_rate;
  }
  void ResetLqrReference() { lqr_initialized_ = false; }
  const WbrControllerV2Telemetry& telemetry() const { return telemetry_; }

 private:
  int act_q_b_ = -1;
  int act_q_d_ = -1;
  int act_q_b_2_ = -1;
  int act_q_d_2_ = -1;
  int act_wheel_ = -1;
  int act_wheel_2_ = -1;
  int joint_q_b_ = -1;
  int joint_q_d_ = -1;
  int joint_q_b_2_ = -1;
  int joint_q_d_2_ = -1;
  int joint_wheel_ = -1;
  int joint_wheel_2_ = -1;
  int joint_root_ = -1;
  int body_plate_ = -1;
  int body_wheel_1_ = -1;
  int body_wheel_2_ = -1;
  int geom_floor_ = -1;
  int geom_wheel_1_ = -1;
  int geom_wheel_2_ = -1;
  int sensor_imu_gyro_ = -1;
  int sensor_imu_accelerometer_ = -1;
  int sensor_imu_quaternion_ = -1;
  int sensor_wheel_speed_1_ = -1;
  int sensor_wheel_speed_2_ = -1;
  int branch_1_ = 1;
  int branch_2_ = 1;
  double leg_speed_1_ = 0.0;
  double leg_speed_2_ = 0.0;
  double leg_angle_speed_1_ = 0.0;
  double leg_angle_speed_2_ = 0.0;
  double filtered_x_speed_ = 0.0;
  double filtered_pitch_speed_ = 0.0;
  double filtered_yaw_speed_ = 0.0;
  double filtered_yaw_acceleration_ = 0.0;
  double previous_yaw_speed_ = 0.0;
  double filtered_wheel_speed_difference_ = 0.0;
  double x_reference_ = 0.0;
  double yaw_reference_ = 0.0;
  double target_linear_velocity_ = 0.0;
  double target_yaw_rate_ = 0.0;
  double commanded_linear_velocity_ = 0.0;
  double commanded_yaw_rate_ = 0.0;
  wbr::v2::YawCoordinator yaw_coordinator_;
  double commanded_yaw_torque_ = 0.0;
  double commanded_differential_leg_angle_torque_ = 0.0;
  double commanded_leg_length_ = 0.18;
  double commanded_leg_angle_ = 0.0;
  double support_factor_1_ = 0.0;
  double support_factor_2_ = 0.0;
  wbr::v2::StateEstimator state_estimator_;
  double leg_length_integral_[2] = {};
  wbr::v2::ContactSafetyMachine contact_safety_;
  double contact_leg_length_offset_[2] = {};
  double last_time_ = 0.0;
  bool command_initialized_ = false;
  bool lqr_initialized_ = false;
  bool yaw_initialized_ = false;
  bool lqr_enabled_ = true;
  bool yaw_enabled_ = true;
  WbrContactSafetyState contact_safety_state_ =
      WbrContactSafetyState::kAirborne;
  WbrControllerV2Telemetry telemetry_{};
};

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_

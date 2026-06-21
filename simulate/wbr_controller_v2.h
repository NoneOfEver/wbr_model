#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_

#include <mujoco/mujoco.h>

enum class WbrContactSafetyState {
  kDualSupport = 0,
  kSingleSupportFirst = 1,
  kSingleSupportSecond = 2,
  kAirborne = 3,
  kRecovery = 4,
};

struct WbrControllerV2Telemetry {
  double leg_length[2] = {};
  double leg_length_rate[2] = {};
  double axial_force[2] = {};
  double integral_force[2] = {};
  double requested_wheel_torque = 0.0;
  double requested_leg_angle_torque = 0.0;
  double applied_wheel_torque = 0.0;
  double applied_leg_angle_torque = 0.0;
  double yaw_error = 0.0;
  double yaw_rate = 0.0;
  double yaw_rate_error = 0.0;
  double yaw_authority_scale = 1.0;
  double yaw_attitude_authority = 1.0;
  double yaw_contact_authority = 1.0;
  double yaw_torque_authority = 1.0;
  double yaw_split_authority = 1.0;
  double spin_mode_blend = 0.0;
  double reserved_yaw_torque_per_wheel = 0.0;
  double balance_torque_authority = 1.0;
  double coordinated_yaw_rate = 0.0;
  double differential_leg_angle_reference = 0.0;
  double differential_leg_angle_residual = 0.0;
  double differential_leg_angle_rate_residual = 0.0;
  double predicted_split_error = 0.0;
  double split_activity = 0.0;
  double predicted_roll = 0.0;
  double predicted_normal_force[2] = {};
  double wheel_normal_force[2] = {};
  double wheel_torque_margin = 0.0;
  double requested_yaw_torque = 0.0;
  double applied_yaw_torque = 0.0;
  double target_wheel_speed_difference = 0.0;
  double wheel_speed_difference = 0.0;
  double commanded_linear_velocity = 0.0;
  double commanded_yaw_rate = 0.0;
  double differential_leg_angle_error = 0.0;
  double differential_leg_angle_rate = 0.0;
  double differential_leg_angle_torque = 0.0;
  double state_error[6] = {};
  bool wheel_grounded[2] = {};
  bool balance_active = false;
  WbrContactSafetyState contact_safety_state =
      WbrContactSafetyState::kAirborne;
  double contact_authority_scale = 0.0;
  double estimated_roll = 0.0;
  double estimated_pitch = 0.0;
  double estimated_yaw = 0.0;
  double estimated_x = 0.0;
  double estimated_x_speed = 0.0;
  double wheel_odometry_x_speed = 0.0;
  double wheel_odometry_confidence = 0.0;
};

enum class WbrControlMode {
  kStandLeg = 0,
  kGroundBalance = 1,
};

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
  void SetControlMode(WbrControlMode mode);
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
  double coordinated_yaw_rate_ = 0.0;
  double commanded_yaw_torque_ = 0.0;
  double commanded_differential_leg_angle_torque_ = 0.0;
  double differential_leg_angle_reference_ = 0.0;
  double commanded_leg_length_ = 0.18;
  double commanded_leg_angle_ = 0.0;
  double support_factor_1_ = 0.0;
  double support_factor_2_ = 0.0;
  double filtered_normal_force_[2] = {};
  double filtered_normal_force_rate_[2] = {};
  double split_activity_ = 0.0;
  double estimated_roll_ = 0.0;
  double estimated_pitch_ = 0.0;
  double estimated_yaw_ = 0.0;
  double estimated_x_ = 0.0;
  double estimated_x_speed_ = 0.0;
  double wheel_odometry_confidence_ = 0.0;
  double leg_length_integral_[2] = {};
  double wheel_contact_grace_[2] = {};
  double contact_leg_length_offset_[2] = {};
  double contact_state_elapsed_ = 0.0;
  double contact_candidate_elapsed_ = 0.0;
  double last_time_ = 0.0;
  bool command_initialized_ = false;
  bool lqr_initialized_ = false;
  bool yaw_initialized_ = false;
  bool yaw_coordinator_initialized_ = false;
  bool state_estimator_initialized_ = false;
  bool lqr_enabled_ = true;
  bool yaw_enabled_ = true;
  WbrContactSafetyState contact_safety_state_ =
      WbrContactSafetyState::kAirborne;
  WbrContactSafetyState contact_candidate_state_ =
      WbrContactSafetyState::kAirborne;
  WbrControlMode control_mode_ = WbrControlMode::kGroundBalance;
  WbrControllerV2Telemetry telemetry_{};
};

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_

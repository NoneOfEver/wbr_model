#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_COMMON_CONTROLLER_TYPES_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_COMMON_CONTROLLER_TYPES_H_

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

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_COMMON_CONTROLLER_TYPES_H_

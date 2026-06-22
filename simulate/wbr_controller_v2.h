#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_

#include <mujoco/mujoco.h>
#include "wbr/control/ground_balance_controller.h"
#include "wbr/platform/mujoco/state_estimator.h"

class WbrControllerV2 {
 public:
  void Reset(const mjModel* model);
  void SyncTargetsFromState(const mjModel* model, const mjData* data,
                            double& target_leg_length, double& target_leg_angle);
  void Apply(const mjModel* model, mjData* data,
             double& target_leg_length, double& target_leg_angle);
  void SetLqrEnabled(bool enabled) { controller_.SetLqrEnabled(enabled); }
  void SetYawEnabled(bool enabled) { controller_.SetYawEnabled(enabled); }
  void SetVelocityCommand(double linear_velocity, double yaw_rate) {
    controller_.SetVelocityCommand(linear_velocity, yaw_rate);
  }
  void ResetLqrReference() { controller_.ResetLqrReference(); }
  const WbrControllerV2Telemetry& telemetry() const {
    return controller_.telemetry();
  }

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
  int geom_floor_ = -1;
  int geom_wheel_1_ = -1;
  int geom_wheel_2_ = -1;
  int sensor_imu_gyro_ = -1;
  int sensor_imu_accelerometer_ = -1;
  int sensor_imu_quaternion_ = -1;
  int sensor_wheel_speed_1_ = -1;
  int sensor_wheel_speed_2_ = -1;
  wbr::v2::StateEstimator state_estimator_;
  wbr::v2::GroundBalanceController controller_;
  double last_time_ = 0.0;
  bool time_initialized_ = false;
};

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_H_

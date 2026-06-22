#include "wbr_controller_v2.h"

#include <cmath>
#include <cstdio>

#include "wbr/platform/mujoco/observation_io.h"
#include "wbr/control/leg_kinematics.h"

using namespace wbr::v2;

void WbrControllerV2::Reset(const mjModel* model) {
  act_q_b_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_B");
  act_q_d_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_D");
  act_q_b_2_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_B_2");
  act_q_d_2_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_D_2");
  act_wheel_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel");
  act_wheel_2_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel_2");
  joint_q_b_ = mj_name2id(model, mjOBJ_JOINT, "q_B");
  joint_q_d_ = mj_name2id(model, mjOBJ_JOINT, "q_D");
  joint_q_b_2_ = mj_name2id(model, mjOBJ_JOINT, "q_B_2");
  joint_q_d_2_ = mj_name2id(model, mjOBJ_JOINT, "q_D_2");
  joint_wheel_ = mj_name2id(model, mjOBJ_JOINT, "q_H_wheel");
  joint_wheel_2_ = mj_name2id(model, mjOBJ_JOINT, "q_H_wheel_2");
  joint_root_ = mj_name2id(model, mjOBJ_JOINT, "root_free");
  body_plate_ = mj_name2id(model, mjOBJ_BODY, "plate");
  geom_floor_ = mj_name2id(model, mjOBJ_GEOM, "floor");
  geom_wheel_1_ = mj_name2id(model, mjOBJ_GEOM, "H_wheel");
  geom_wheel_2_ = mj_name2id(model, mjOBJ_GEOM, "H_wheel_2");
  sensor_imu_gyro_ = mj_name2id(model, mjOBJ_SENSOR, "imu_gyro");
  sensor_imu_accelerometer_ =
      mj_name2id(model, mjOBJ_SENSOR, "imu_accelerometer");
  sensor_imu_quaternion_ =
      mj_name2id(model, mjOBJ_SENSOR, "imu_quaternion");
  sensor_wheel_speed_1_ =
      mj_name2id(model, mjOBJ_SENSOR, "wheel_speed_1");
  sensor_wheel_speed_2_ =
      mj_name2id(model, mjOBJ_SENSOR, "wheel_speed_2");

  state_estimator_.Reset();
  controller_.Reset();
  last_time_ = 0.0;
  time_initialized_ = false;

  if (act_q_b_ < 0 || act_q_d_ < 0 || joint_q_b_ < 0 || joint_q_d_ < 0) {
    std::printf("WbrControllerV2: first leg actuators/joints not found.\n");
  }
  if (act_q_b_2_ < 0 || act_q_d_2_ < 0 ||
      joint_q_b_2_ < 0 || joint_q_d_2_ < 0) {
    std::printf("WbrControllerV2: second leg actuators/joints not found.\n");
  }
  if (joint_root_ >= 0 && body_plate_ >= 0) {
    std::printf("WbrControllerV2: Ground Balance controller enabled.\n");
  }
}

void WbrControllerV2::SyncTargetsFromState(
    const mjModel* model, const mjData* data, double& target_leg_length,
    double& target_leg_angle) {
  if (!model || !data) return;
  target_leg_length = 0.18;
  target_leg_angle = 0.0;

  double length_sum = 0.0;
  double angle_sin_sum = 0.0;
  double angle_cos_sum = 0.0;
  int valid_legs = 0;
  const int joint_b[2] = {joint_q_b_, joint_q_b_2_};
  const int joint_d[2] = {joint_q_d_, joint_q_d_2_};
  const double mirror[2] = {1.0, -1.0};
  const double body_pitch =
      body_plate_ >= 0 ? RootPitch(data, body_plate_) : 0.0;
  for (int leg = 0; leg < 2; ++leg) {
    if (joint_b[leg] < 0 || joint_d[leg] < 0) continue;
    const double phi1 =
        mirror[leg] * data->qpos[model->jnt_qposadr[joint_d[leg]]];
    const double phi2 =
        mirror[leg] * data->qpos[model->jnt_qposadr[joint_b[leg]]];
    double hx;
    double hz;
    if (!ForwardKinematics(phi1, phi2, 1, hx, hz)) continue;
    const double angle = std::atan2(hx, -hz) + body_pitch;
    length_sum += std::hypot(hx, hz);
    angle_sin_sum += std::sin(angle);
    angle_cos_sum += std::cos(angle);
    ++valid_legs;
  }
  if (valid_legs > 0) {
    target_leg_length = length_sum / valid_legs;
    target_leg_angle = std::atan2(angle_sin_sum, angle_cos_sum);
    controller_.InitializeLegTarget(target_leg_length, target_leg_angle);
  }
}

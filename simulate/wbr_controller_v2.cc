#include "wbr_controller_v2.h"

#include <cmath>

#include "wbr/control/math_utils.h"
#include "wbr/platform/mujoco/actuator_io.h"
#include "wbr/platform/mujoco/leg_io.h"
#include "wbr/platform/mujoco/observation_io.h"

using namespace wbr::v2;

void WbrControllerV2::Apply(const mjModel* model, mjData* data,
                            double& target_leg_length,
                            double& target_leg_angle) {
  if (!model || !data) return;

  const bool time_reset = time_initialized_ && data->time < last_time_;
  double control_dt = model->opt.timestep;
  if (time_initialized_ && !time_reset && data->time > last_time_) {
    control_dt = Clamp(data->time - last_time_, 1e-4, 0.02);
  }

  GroundBalanceInput input;
  input.control_dt = control_dt;
  input.time_reset = time_reset;
  input.target_leg_length = target_leg_length;
  input.target_leg_angle = target_leg_angle;
  input.total_mass = mj_getTotalmass(model);
  input.gravity_magnitude = std::fabs(model->opt.gravity[2]);
  input.yaw_plant_enabled =
      joint_root_ >= 0 && model->jnt_type[joint_root_] == mjJNT_FREE;

  const bool first_leg_available =
      act_q_b_ >= 0 && act_q_d_ >= 0 && joint_q_b_ >= 0 && joint_q_d_ >= 0;
  const bool second_leg_available =
      act_q_b_2_ >= 0 && act_q_d_2_ >= 0 &&
      joint_q_b_2_ >= 0 && joint_q_d_2_ >= 0;
  input.leg_valid[0] = first_leg_available && ReadLegKinematics(
      model, data, joint_q_b_, joint_q_d_, 1, 1.0, input.leg[0]);
  input.leg_valid[1] = second_leg_available && ReadLegKinematics(
      model, data, joint_q_b_2_, joint_q_d_2_, 1, -1.0, input.leg[1]);

  input.wheel_grounded[0] =
      geom_floor_ >= 0 && geom_wheel_1_ >= 0 &&
      GeomsInContact(data, geom_floor_, geom_wheel_1_);
  input.wheel_grounded[1] =
      geom_floor_ >= 0 && geom_wheel_2_ >= 0 &&
      GeomsInContact(data, geom_floor_, geom_wheel_2_);
  input.wheel_normal_force[0] = ContactNormalForce(
      model, data, geom_floor_, geom_wheel_1_);
  input.wheel_normal_force[1] = ContactNormalForce(
      model, data, geom_floor_, geom_wheel_2_);

  input.observation_valid = input.leg_valid[0] && input.leg_valid[1] &&
      joint_root_ >= 0 && body_plate_ >= 0;
  if (input.observation_valid) {
    StateEstimatorHandles handles;
    handles.body_plate = body_plate_;
    handles.joint_wheel[0] = joint_wheel_;
    handles.joint_wheel[1] = joint_wheel_2_;
    handles.geom_wheel[0] = geom_wheel_1_;
    handles.geom_wheel[1] = geom_wheel_2_;
    handles.sensor_imu_gyro = sensor_imu_gyro_;
    handles.sensor_imu_accelerometer = sensor_imu_accelerometer_;
    handles.sensor_imu_quaternion = sensor_imu_quaternion_;
    handles.sensor_wheel_speed[0] = sensor_wheel_speed_1_;
    handles.sensor_wheel_speed[1] = sensor_wheel_speed_2_;
    const StateEstimate estimate = state_estimator_.Update(
        model, data, handles, input.wheel_grounded, control_dt, time_reset);
    input.roll = estimate.roll;
    input.pitch = estimate.pitch;
    input.roll_rate = estimate.roll_rate;
    input.pitch_rate = estimate.pitch_rate;
    input.yaw_rate = estimate.yaw_rate;
    input.x = estimate.x;
    input.x_speed = estimate.x_speed;
    input.wheel_odometry_x_speed = estimate.wheel_odometry_x_speed;
    input.wheel_odometry_confidence = estimate.wheel_odometry_confidence;
  }

  const GroundBalanceOutput output = controller_.Update(input);
  target_leg_length = output.sanitized_leg_length;
  target_leg_angle = output.sanitized_leg_angle;

  const int actuator_ids[wbr::control::kMotorCount] = {
      act_q_b_, act_q_d_, act_q_b_2_, act_q_d_2_,
      act_wheel_, act_wheel_2_};
  WriteMotorCommands(data, actuator_ids, output.actuator);
  last_time_ = data->time;
  time_initialized_ = true;
}

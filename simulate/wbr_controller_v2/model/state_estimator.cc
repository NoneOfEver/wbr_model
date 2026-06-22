#include "state_estimator.h"

#include <cmath>

#include "../common/controller_config.h"
#include "../common/controller_math.h"

namespace wbr::v2 {

void StateEstimator::Reset() {
  initialized_ = false;
  roll_ = pitch_ = yaw_ = 0.0;
  x_ = x_speed_ = 0.0;
  wheel_odometry_confidence_ = 0.0;
}

StateEstimate StateEstimator::Update(
    const mjModel* model, const mjData* data,
    const StateEstimatorHandles& handles, const bool wheel_grounded[2],
    double control_dt, bool time_reset) {
  StateEstimate estimate;
  mjtNum plate_velocity[6] = {};
  mj_objectVelocity(model, data, mjOBJ_BODY, handles.body_plate,
                    plate_velocity, 0);

  double measured_roll = RootRoll(data, handles.body_plate);
  double measured_pitch = RootPitch(data, handles.body_plate);
  double measured_yaw = RootYaw(data, handles.body_plate);
  estimate.roll_rate = plate_velocity[0];
  estimate.pitch_rate = -plate_velocity[1];
  estimate.yaw_rate = plate_velocity[2];

  mjtNum imu_rotation[9];
  mju_copy(imu_rotation, data->xmat + 9 * handles.body_plate, 9);
  if (const mjtNum* quaternion = SensorData(
          model, data, handles.sensor_imu_quaternion, 4)) {
    QuaternionToEuler(quaternion, measured_roll, measured_pitch, measured_yaw,
                      imu_rotation);
  }
  if (const mjtNum* gyro = SensorData(
          model, data, handles.sensor_imu_gyro, 3)) {
    estimate.roll_rate = gyro[0];
    estimate.pitch_rate = -gyro[1];
    estimate.yaw_rate = gyro[2];
  }

  const double* rotation = data->xmat + 9 * handles.body_plate;
  for (int axis = 0; axis < 3; ++axis) {
    estimate.forward[axis] = rotation[3 * axis];
    estimate.lateral[axis] = rotation[3 * axis + 1];
  }
  const double chassis_forward_speed =
      plate_velocity[3] * estimate.forward[0] +
      plate_velocity[4] * estimate.forward[1] +
      plate_velocity[5] * estimate.forward[2];

  if (const mjtNum* accelerometer = SensorData(
          model, data, handles.sensor_imu_accelerometer, 3)) {
    double world_acceleration[3];
    for (int row = 0; row < 3; ++row) {
      world_acceleration[row] =
          imu_rotation[3 * row] * accelerometer[0] +
          imu_rotation[3 * row + 1] * accelerometer[1] +
          imu_rotation[3 * row + 2] * accelerometer[2] +
          model->opt.gravity[row];
    }
    estimate.forward_acceleration =
        world_acceleration[0] * estimate.forward[0] +
        world_acceleration[1] * estimate.forward[1] +
        world_acceleration[2] * estimate.forward[2];
  }

  if (handles.joint_wheel[0] >= 0 && handles.joint_wheel[1] >= 0 &&
      handles.geom_wheel[0] >= 0 && handles.geom_wheel[1] >= 0) {
    const double wheel_radius = 0.5 *
        (model->geom_size[3 * handles.geom_wheel[0]] +
         model->geom_size[3 * handles.geom_wheel[1]]);
    double wheel_speed[2];
    for (int wheel = 0; wheel < 2; ++wheel) {
      const mjtNum* measured = SensorData(
          model, data, handles.sensor_wheel_speed[wheel], 1);
      wheel_speed[wheel] = measured
          ? measured[0]
          : data->qvel[model->jnt_dofadr[handles.joint_wheel[wheel]]];
    }
    estimate.wheel_speed_difference =
        wheel_radius * (wheel_speed[0] + wheel_speed[1]);
    estimate.wheel_odometry_x_speed =
        0.5 * wheel_radius * (wheel_speed[0] - wheel_speed[1]);
  }

  if (!initialized_ || time_reset) {
    roll_ = measured_roll;
    pitch_ = measured_pitch;
    yaw_ = measured_yaw;
    x_ = 0.0;
    x_speed_ = chassis_forward_speed;
    initialized_ = true;
  } else {
    roll_ += estimate.roll_rate * control_dt;
    pitch_ += estimate.pitch_rate * control_dt;
    yaw_ += estimate.yaw_rate * control_dt;
    const double orientation_alpha = 1.0 - std::exp(-control_dt / 0.03);
    roll_ += orientation_alpha *
        std::remainder(measured_roll - roll_, 2.0 * M_PI);
    pitch_ += orientation_alpha *
        std::remainder(measured_pitch - pitch_, 2.0 * M_PI);
    yaw_ += orientation_alpha *
        std::remainder(measured_yaw - yaw_, 2.0 * M_PI);

    x_speed_ += estimate.forward_acceleration * control_dt;
    const double chassis_alpha = 1.0 - std::exp(
        -control_dt / kChassisVelocityCorrectionTimeConstant);
    x_speed_ += chassis_alpha * (chassis_forward_speed - x_speed_);
    const double contact_confidence = wheel_grounded[0] && wheel_grounded[1]
        ? 1.0
        : ((wheel_grounded[0] || wheel_grounded[1]) ? 0.2 : 0.0);
    const double slip_confidence = FadeAuthority(
        std::fabs(estimate.wheel_odometry_x_speed - x_speed_),
        kWheelSlipSoftSpeed, kWheelSlipHardSpeed);
    wheel_odometry_confidence_ = contact_confidence * slip_confidence;
    const double odometry_alpha = wheel_odometry_confidence_ *
        (1.0 - std::exp(
            -control_dt / kWheelOdometryCorrectionTimeConstant));
    x_speed_ += odometry_alpha *
        (estimate.wheel_odometry_x_speed - x_speed_);
    x_ += x_speed_ * control_dt;
  }

  const double wheel_separation[3] = {
      data->xpos[3 * handles.body_wheel[0]] -
          data->xpos[3 * handles.body_wheel[1]],
      data->xpos[3 * handles.body_wheel[0] + 1] -
          data->xpos[3 * handles.body_wheel[1] + 1],
      data->xpos[3 * handles.body_wheel[0] + 2] -
          data->xpos[3 * handles.body_wheel[1] + 2]};
  estimate.track_width = std::fmax(
      std::fabs(wheel_separation[0] * estimate.lateral[0] +
                wheel_separation[1] * estimate.lateral[1] +
                wheel_separation[2] * estimate.lateral[2]),
      0.1);
  estimate.roll = roll_;
  estimate.pitch = pitch_;
  estimate.yaw = yaw_;
  estimate.x = x_;
  estimate.x_speed = x_speed_;
  estimate.wheel_odometry_confidence = wheel_odometry_confidence_;
  return estimate;
}

}  // namespace wbr::v2

#ifndef WBR_PLATFORM_MUJOCO_STATE_ESTIMATOR_H_
#define WBR_PLATFORM_MUJOCO_STATE_ESTIMATOR_H_

#include <mujoco/mujoco.h>

namespace wbr::v2 {

struct StateEstimatorHandles {
  int body_plate = -1;
  int joint_wheel[2] = {-1, -1};
  int geom_wheel[2] = {-1, -1};
  int sensor_imu_gyro = -1;
  int sensor_imu_accelerometer = -1;
  int sensor_imu_quaternion = -1;
  int sensor_wheel_speed[2] = {-1, -1};
};

struct StateEstimate {
  double roll = 0.0;
  double pitch = 0.0;
  double yaw = 0.0;
  double roll_rate = 0.0;
  double pitch_rate = 0.0;
  double yaw_rate = 0.0;
  double forward_acceleration = 0.0;
  double forward[3] = {1.0, 0.0, 0.0};
  double x = 0.0;
  double x_speed = 0.0;
  double wheel_odometry_x_speed = 0.0;
  double wheel_odometry_confidence = 0.0;
};

class StateEstimator {
 public:
  void Reset();
  StateEstimate Update(const mjModel* model, const mjData* data,
                       const StateEstimatorHandles& handles,
                       const bool wheel_grounded[2], double control_dt,
                       bool time_reset);

 private:
  bool initialized_ = false;
  double roll_ = 0.0;
  double pitch_ = 0.0;
  double yaw_ = 0.0;
  double x_ = 0.0;
  double x_speed_ = 0.0;
  double wheel_odometry_confidence_ = 0.0;
};

}  // namespace wbr::v2

#endif  // WBR_PLATFORM_MUJOCO_STATE_ESTIMATOR_H_

#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_COMMON_CONTROLLER_MATH_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_COMMON_CONTROLLER_MATH_H_

#include <mujoco/mujoco.h>

namespace wbr::v2 {

double Clamp(double value, double lo, double hi);
double MoveTowards(double value, double target, double max_step);
double FadeAuthority(double magnitude, double soft_limit, double hard_limit);
double RiseAuthority(double value, double hard_limit, double soft_limit);

double ContactNormalForce(const mjModel* model, const mjData* data,
                          int geom_1, int geom_2);
bool GeomsInContact(const mjData* data, int geom_1, int geom_2);
const mjtNum* SensorData(const mjModel* model, const mjData* data,
                         int sensor_id, int expected_dimension);

void QuaternionToEuler(const mjtNum quaternion[4], double& roll,
                       double& pitch, double& yaw, mjtNum rotation[9]);
double RootPitch(const mjData* data, int body_id);
double RootRoll(const mjData* data, int body_id);
double RootYaw(const mjData* data, int body_id);

}  // namespace wbr::v2

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_COMMON_CONTROLLER_MATH_H_

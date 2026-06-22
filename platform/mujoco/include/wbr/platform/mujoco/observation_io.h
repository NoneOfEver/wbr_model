#ifndef WBR_PLATFORM_MUJOCO_OBSERVATION_IO_H_
#define WBR_PLATFORM_MUJOCO_OBSERVATION_IO_H_

#include <mujoco/mujoco.h>

namespace wbr::v2 {

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

#endif  // WBR_PLATFORM_MUJOCO_OBSERVATION_IO_H_

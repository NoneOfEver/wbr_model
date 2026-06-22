#include "controller_math.h"

#include <cmath>

namespace wbr::v2 {

double Clamp(double value, double lo, double hi) {
  return std::fmin(std::fmax(value, lo), hi);
}

double MoveTowards(double value, double target, double max_step) {
  return value + Clamp(target - value, -max_step, max_step);
}

double FadeAuthority(double magnitude, double soft_limit, double hard_limit) {
  return 1.0 - Clamp((magnitude - soft_limit) /
                         (hard_limit - soft_limit),
                     0.0, 1.0);
}

double RiseAuthority(double value, double hard_limit, double soft_limit) {
  return Clamp((value - hard_limit) / (soft_limit - hard_limit), 0.0, 1.0);
}

double ContactNormalForce(const mjModel* model, const mjData* data,
                          int geom_1, int geom_2) {
  double normal_force = 0.0;
  if (geom_1 < 0 || geom_2 < 0) {
    return normal_force;
  }
  for (int contact_id = 0; contact_id < data->ncon; ++contact_id) {
    const mjContact& contact = data->contact[contact_id];
    if (!((contact.geom[0] == geom_1 && contact.geom[1] == geom_2) ||
          (contact.geom[0] == geom_2 && contact.geom[1] == geom_1))) {
      continue;
    }
    mjtNum contact_force[6] = {};
    mj_contactForce(model, data, contact_id, contact_force);
    normal_force += std::fmax(0.0, contact_force[0]);
  }
  return normal_force;
}

bool GeomsInContact(const mjData* data, int geom_1, int geom_2) {
  for (int contact_id = 0; contact_id < data->ncon; ++contact_id) {
    const mjContact& contact = data->contact[contact_id];
    if ((contact.geom[0] == geom_1 && contact.geom[1] == geom_2) ||
        (contact.geom[0] == geom_2 && contact.geom[1] == geom_1)) {
      return true;
    }
  }
  return false;
}

const mjtNum* SensorData(const mjModel* model, const mjData* data,
                         int sensor_id, int expected_dimension) {
  if (sensor_id < 0 || model->sensor_dim[sensor_id] != expected_dimension) {
    return nullptr;
  }
  return data->sensordata + model->sensor_adr[sensor_id];
}

void QuaternionToEuler(const mjtNum quaternion[4], double& roll,
                       double& pitch, double& yaw, mjtNum rotation[9]) {
  mju_quat2Mat(rotation, quaternion);
  roll = std::atan2(rotation[7], rotation[8]);
  pitch = std::atan2(rotation[6],
                     std::hypot(rotation[0], rotation[3]));
  yaw = std::atan2(rotation[3], rotation[0]);
}

double RootPitch(const mjData* data, int body_id) {
  const double* rotation = data->xmat + 9 * body_id;
  return std::atan2(rotation[6], std::hypot(rotation[0], rotation[3]));
}

double RootRoll(const mjData* data, int body_id) {
  const double* rotation = data->xmat + 9 * body_id;
  return std::atan2(rotation[7], rotation[8]);
}

double RootYaw(const mjData* data, int body_id) {
  const double* rotation = data->xmat + 9 * body_id;
  return std::atan2(rotation[3], rotation[0]);
}

}  // namespace wbr::v2

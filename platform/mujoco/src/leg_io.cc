#include "wbr/platform/mujoco/leg_io.h"

#include "wbr/control/control_parameters.h"
#include "wbr/control/math_utils.h"

namespace wbr::v2 {

bool ReadLegKinematics(const mjModel* model, const mjData* data,
                       int joint_b, int joint_d, int branch,
                       double mirror_sign, LegKinematics& leg) {
  const double phi1 = mirror_sign * data->qpos[model->jnt_qposadr[joint_d]];
  const double phi2 = mirror_sign * data->qpos[model->jnt_qposadr[joint_b]];
  const double dphi1 = mirror_sign * data->qvel[model->jnt_dofadr[joint_d]];
  const double dphi2 = mirror_sign * data->qvel[model->jnt_dofadr[joint_b]];
  return ComputeLegKinematics(phi1, phi2, dphi1, dphi2, branch, leg);
}

double ApplyLegVmc(mjData* data, int actuator_b, int actuator_d,
                   const LegKinematics& leg, double target_leg_length,
                   double support_feedforward, double integral_force,
                   double leg_angle_torque, double mirror_sign,
                   double filtered_leg_speed) {
  const LegVmcOutput output = ComputeLegVmc(
      leg, target_leg_length, support_feedforward, integral_force,
      leg_angle_torque, filtered_leg_speed);
  data->ctrl[actuator_b] = Clamp(
      mirror_sign * output.joint_torque[1],
      -kJointTorqueLimit, kJointTorqueLimit);
  data->ctrl[actuator_d] = Clamp(
      mirror_sign * output.joint_torque[0],
      -kJointTorqueLimit, kJointTorqueLimit);
  return output.axial_force;
}

}  // namespace wbr::v2


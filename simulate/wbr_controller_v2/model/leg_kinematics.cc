#include "leg_kinematics.h"

#include <cmath>

#include "../common/controller_config.h"
#include "../common/controller_math.h"

namespace wbr::v2 {

bool ForwardKinematics(double phi1, double phi2, int branch,
                       double& hx, double& hz) {
  const double bx = kLengthAB * std::cos(phi2);
  const double bz = -kLengthAB * std::sin(phi2);
  const double dx = kLengthAD * std::cos(phi1);
  const double dz = -kLengthAD * std::sin(phi1);
  const double gx = kLengthAG * std::cos(phi1);
  const double gz = -kLengthAG * std::sin(phi1);

  const double dbx = dx - bx;
  const double dbz = dz - bz;
  const double distance = std::hypot(dbx, dbz);
  if (distance > kLengthBC + kLengthCD - 1e-9 || distance < 1e-9) {
    return false;
  }

  const double a = (kLengthBC * kLengthBC - kLengthCD * kLengthCD +
                    distance * distance) /
                   (2.0 * distance);
  const double h = std::sqrt(std::fmax(0.0, kLengthBC * kLengthBC - a * a));
  const double px = bx + a * dbx / distance;
  const double pz = bz + a * dbz / distance;
  const double cx = branch == 1 ? px - h * dbz / distance
                                : px + h * dbz / distance;
  const double cz = branch == 1 ? pz + h * dbx / distance
                                : pz - h * dbx / distance;

  const double dcx = cx - dx;
  const double dcz = cz - dz;
  const double dc_length = std::hypot(dcx, dcz);
  if (dc_length < 1e-12) {
    return false;
  }
  hx = gx + kLengthGH * dcx / dc_length;
  hz = gz + kLengthGH * dcz / dc_length;
  return true;
}

bool NumericalJacobian(double phi1, double phi2, int branch,
                       double jacobian[2][2]) {
  double hx_plus;
  double hz_plus;
  double hx_minus;
  double hz_minus;
  if (!ForwardKinematics(phi1 + kJacobianStep, phi2, branch,
                         hx_plus, hz_plus) ||
      !ForwardKinematics(phi1 - kJacobianStep, phi2, branch,
                         hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][0] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][0] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  if (!ForwardKinematics(phi1, phi2 + kJacobianStep, branch,
                         hx_plus, hz_plus) ||
      !ForwardKinematics(phi1, phi2 - kJacobianStep, branch,
                         hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][1] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][1] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  return true;
}

bool ComputeLegKinematics(const mjModel* model, const mjData* data,
                          int joint_b, int joint_d, int branch,
                          double mirror_sign, LegKinematics& leg) {
  const double phi1 = mirror_sign * data->qpos[model->jnt_qposadr[joint_d]];
  const double phi2 = mirror_sign * data->qpos[model->jnt_qposadr[joint_b]];
  const double dphi1 = mirror_sign * data->qvel[model->jnt_dofadr[joint_d]];
  const double dphi2 = mirror_sign * data->qvel[model->jnt_dofadr[joint_b]];
  if (!ForwardKinematics(phi1, phi2, branch, leg.hx, leg.hz) ||
      !NumericalJacobian(phi1, phi2, branch, leg.jacobian)) {
    return false;
  }
  leg.length = std::hypot(leg.hx, leg.hz);
  if (leg.length < kMinLegLength) {
    return false;
  }
  const double velocity_x =
      leg.jacobian[0][0] * dphi1 + leg.jacobian[0][1] * dphi2;
  const double velocity_z =
      leg.jacobian[1][0] * dphi1 + leg.jacobian[1][1] * dphi2;
  leg.length_rate =
      (leg.hx * velocity_x + leg.hz * velocity_z) / leg.length;
  leg.angle = std::atan2(leg.hx, -leg.hz);
  leg.angle_rate = (-leg.hz * velocity_x + leg.hx * velocity_z) /
                   (leg.length * leg.length);
  return true;
}

double ApplyLegVmc(mjData* data, int actuator_b, int actuator_d,
                   const LegKinematics& leg, double target_leg_length,
                   double support_feedforward, double integral_force,
                   double leg_angle_torque, double mirror_sign,
                   double filtered_leg_speed) {
  double axial_force = kLegLengthKp * (target_leg_length - leg.length) -
                       kLegLengthKd * filtered_leg_speed + support_feedforward +
                       integral_force;
  axial_force = Clamp(axial_force, -kLegForceLimit, kLegForceLimit);
  const double radial_x = leg.hx / leg.length;
  const double radial_z = leg.hz / leg.length;
  const double tangential_force = leg_angle_torque / leg.length;
  const double force_x = axial_force * radial_x - tangential_force * radial_z;
  const double force_z = axial_force * radial_z + tangential_force * radial_x;
  const double torque_phi1 =
      leg.jacobian[0][0] * force_x + leg.jacobian[1][0] * force_z;
  const double torque_phi2 =
      leg.jacobian[0][1] * force_x + leg.jacobian[1][1] * force_z;
  data->ctrl[actuator_b] = Clamp(mirror_sign * torque_phi2,
                                 -kJointTorqueLimit, kJointTorqueLimit);
  data->ctrl[actuator_d] = Clamp(mirror_sign * torque_phi1,
                                 -kJointTorqueLimit, kJointTorqueLimit);
  return axial_force;
}

}  // namespace wbr::v2

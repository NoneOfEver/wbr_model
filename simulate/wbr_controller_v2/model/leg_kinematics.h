#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_MODEL_LEG_KINEMATICS_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_MODEL_LEG_KINEMATICS_H_

#include <mujoco/mujoco.h>

namespace wbr::v2 {

struct LegKinematics {
  double hx = 0.0;
  double hz = 0.0;
  double length = 0.0;
  double length_rate = 0.0;
  double angle = 0.0;
  double angle_rate = 0.0;
  double jacobian[2][2] = {};
};

bool ForwardKinematics(double phi1, double phi2, int branch,
                       double& hx, double& hz);
bool NumericalJacobian(double phi1, double phi2, int branch,
                       double jacobian[2][2]);
bool ComputeLegKinematics(const mjModel* model, const mjData* data,
                          int joint_b, int joint_d, int branch,
                          double mirror_sign, LegKinematics& leg);

double ApplyLegVmc(mjData* data, int actuator_b, int actuator_d,
                   const LegKinematics& leg, double target_leg_length,
                   double support_feedforward, double integral_force,
                   double leg_angle_torque, double mirror_sign,
                   double filtered_leg_speed);

}  // namespace wbr::v2

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_V2_MODEL_LEG_KINEMATICS_H_

#ifndef WBR_PLATFORM_MUJOCO_LEG_IO_H_
#define WBR_PLATFORM_MUJOCO_LEG_IO_H_

#include <mujoco/mujoco.h>

#include "wbr/control/leg_kinematics.h"

namespace wbr::v2 {

bool ReadLegKinematics(const mjModel* model, const mjData* data,
                       int joint_b, int joint_d, int branch,
                       double mirror_sign, LegKinematics& leg);
double ApplyLegVmc(mjData* data, int actuator_b, int actuator_d,
                   const LegKinematics& leg, double target_leg_length,
                   double support_feedforward, double integral_force,
                   double leg_angle_torque, double mirror_sign,
                   double filtered_leg_speed);

}  // namespace wbr::v2

#endif  // WBR_PLATFORM_MUJOCO_LEG_IO_H_


#ifndef WBR_MODEL_SIMULATE_WBR_CONTROLLER_H_
#define WBR_MODEL_SIMULATE_WBR_CONTROLLER_H_

#include <mujoco/mujoco.h>

class WbrController {
 public:
  void Reset(const mjModel* model);
  void SyncTargetsFromState(const mjModel* model, const mjData* data,
                            double& target_hx, double& target_hz,
                            double& target_hx_2, double& target_hz_2);
  void Apply(const mjModel* model, mjData* data,
             double& target_hx, double& target_hz,
             double& target_hx_2, double& target_hz_2);

 private:
  int act_q_b_ = -1;
  int act_q_d_ = -1;
  int act_q_b_2_ = -1;
  int act_q_d_2_ = -1;
  int joint_q_b_ = -1;
  int joint_q_d_ = -1;
  int joint_q_b_2_ = -1;
  int joint_q_d_2_ = -1;
  double phi1_prev_ = 0.0;
  double phi2_prev_ = 0.0;
  double phi1_prev_2_ = 0.0;
  double phi2_prev_2_ = 0.0;
  int branch_prev_ = 1;
  int branch_prev_2_ = 1;
};

#endif  // WBR_MODEL_SIMULATE_WBR_CONTROLLER_H_

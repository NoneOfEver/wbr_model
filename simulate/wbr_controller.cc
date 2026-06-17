#include "wbr_controller.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {
constexpr double L_AB = 0.0945;
constexpr double L_BC = 0.1125;
constexpr double L_CD = 0.116;
constexpr double L_AD = 0.090;
constexpr double L_AG = 0.210;
constexpr double L_GH = 0.250;
constexpr double kTargetHRadius = 0.4;

constexpr double kWheelTorqueLimit = 5.0;
constexpr double kWheel1Sign = 1.0;
constexpr double kWheel2Sign = -1.0;
constexpr double kBalanceSign = 1.0;
constexpr double kPitchKp = -2000.0;
constexpr double kPitchKi = 0.0;
constexpr double kPitchKd = -1.0;
constexpr double kPositionKp = 0.10;
constexpr double kVelocityKd = 0.20;

double Clamp(double value, double lo, double hi) {
  return std::fmin(std::fmax(value, lo), hi);
}

void ClampTargetH(double& hx, double& hz) {
  const double dist = std::sqrt(hx * hx + hz * hz);
  if (dist > kTargetHRadius) {
    hx *= kTargetHRadius / dist;
    hz *= kTargetHRadius / dist;
  }
}

double UnwrapAngle(double phi, double ref) {
  while (phi - ref > M_PI) {
    phi -= 2 * M_PI;
  }
  while (phi - ref < -M_PI) {
    phi += 2 * M_PI;
  }
  return phi;
}

bool ForwardKinematics(double phi1, double phi2, int branch, double& hx, double& hz) {
  const double bx = L_AB * std::cos(phi2);
  const double bz = -L_AB * std::sin(phi2);

  const double dx = L_AD * std::cos(phi1);
  const double dz = -L_AD * std::sin(phi1);

  const double gx = L_AG * std::cos(phi1);
  const double gz = -L_AG * std::sin(phi1);

  const double dbx = dx - bx;
  const double dbz = dz - bz;
  const double d = std::sqrt(dbx * dbx + dbz * dbz);

  if (d > L_BC + L_CD - 1e-9 || d < 1e-9) {
    return false;
  }

  const double a = (L_BC * L_BC - L_CD * L_CD + d * d) / (2.0 * d);
  double h2 = L_BC * L_BC - a * a;
  if (h2 < 0) {
    h2 = 0;
  }
  const double h = std::sqrt(h2);

  const double px = bx + a * dbx / d;
  const double pz = bz + a * dbz / d;

  double cx;
  double cz;
  if (branch == 1) {
    cx = px - h * dbz / d;
    cz = pz + h * dbx / d;
  } else {
    cx = px + h * dbz / d;
    cz = pz - h * dbx / d;
  }

  const double dcx = cx - dx;
  const double dcz = cz - dz;
  const double dc_len = std::sqrt(dcx * dcx + dcz * dcz);
  if (dc_len < 1e-12) {
    return false;
  }

  hx = gx + L_GH * dcx / dc_len;
  hz = gz + L_GH * dcz / dc_len;
  return true;
}

bool InverseKinematics(double hx_target, double hz_target,
                       double phi1_init, double phi2_init,
                       double& phi1, double& phi2, int& branch_out) {
  const std::vector<std::pair<double, double>> inits = {
      {phi1_init, phi2_init},
      {1.05, 2.44},
      {-1.05, 2.44},
      {1.05, -2.44},
      {-1.05, -2.44},
  };

  double best_dist = 1e20;
  bool found = false;

  for (int branch = 1; branch <= 2; ++branch) {
    for (const auto& init : inits) {
      double p1 = init.first;
      double p2 = init.second;

      constexpr int max_iter = 50;
      constexpr double dphi = 1e-7;

      for (int iter = 0; iter < max_iter; ++iter) {
        double hx;
        double hz;
        if (!ForwardKinematics(p1, p2, branch, hx, hz)) {
          break;
        }

        const double ex = hx - hx_target;
        const double ez = hz - hz_target;
        const double err = ex * ex + ez * ez;

        if (err < 1e-12) {
          const double d1 = p1 - phi1_init;
          const double d2 = p2 - phi2_init;
          const double dist = d1 * d1 + d2 * d2;
          if (dist < best_dist) {
            best_dist = dist;
            phi1 = p1;
            phi2 = p2;
            branch_out = branch;
            found = true;
          }
          break;
        }

        double hx1;
        double hz1;
        double hx2;
        double hz2;
        if (!ForwardKinematics(p1 + dphi, p2, branch, hx1, hz1) ||
            !ForwardKinematics(p1, p2 + dphi, branch, hx2, hz2)) {
          break;
        }

        const double j11 = (hx1 - hx) / dphi;
        const double j21 = (hz1 - hz) / dphi;
        const double j12 = (hx2 - hx) / dphi;
        const double j22 = (hz2 - hz) / dphi;

        const double det = j11 * j22 - j12 * j21;
        if (std::fabs(det) < 1e-12) {
          p1 -= 0.05 * (j11 * ex + j21 * ez);
          p2 -= 0.05 * (j12 * ex + j22 * ez);
        } else {
          const double d1 = (j22 * ex - j12 * ez) / det;
          const double d2 = (-j21 * ex + j11 * ez) / det;
          constexpr double lambda = 0.5;
          p1 -= lambda * d1;
          p2 -= lambda * d2;
        }
      }
    }
  }

  return found;
}

double RootPitch(const mjData* data, int body_id) {
  const double* xmat = data->xmat + 9 * body_id;
  return std::atan2(-xmat[2], xmat[0]);
}
}  // namespace

void WbrController::Reset(const mjModel* model) {
  act_q_b_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_B");
  act_q_d_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_D");
  act_q_b_2_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_B_2");
  act_q_d_2_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_D_2");
  act_wheel_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel");
  act_wheel_2_ = mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel_2");
  joint_q_b_ = mj_name2id(model, mjOBJ_JOINT, "q_B");
  joint_q_d_ = mj_name2id(model, mjOBJ_JOINT, "q_D");
  joint_q_b_2_ = mj_name2id(model, mjOBJ_JOINT, "q_B_2");
  joint_q_d_2_ = mj_name2id(model, mjOBJ_JOINT, "q_D_2");
  joint_root_ = mj_name2id(model, mjOBJ_JOINT, "root_free");
  body_plate_ = mj_name2id(model, mjOBJ_BODY, "plate");

  branch_prev_ = 1;
  branch_prev_2_ = 1;
  balance_integral_ = 0.0;
  balance_last_time_ = 0.0;
  balance_initialized_ = false;

  if (act_q_b_ < 0 || act_q_d_ < 0 || joint_q_b_ < 0 || joint_q_d_ < 0) {
    std::printf("WbrController: first linkage actuators/joints not found; controller disabled.\n");
  }

  if (act_q_b_2_ < 0 || act_q_d_2_ < 0 || joint_q_b_2_ < 0 || joint_q_d_2_ < 0) {
    std::printf("WbrController: second linkage actuators/joints not found; controlling first linkage only.\n");
  }

  if (joint_root_ >= 0 && body_plate_ >= 0 && act_wheel_ >= 0 && act_wheel_2_ >= 0) {
    std::printf("WbrController: wheel balance PID enabled.\n");
  }
}

void WbrController::SyncTargetsFromState(const mjModel* model, const mjData* data,
                                         double& target_hx, double& target_hz,
                                         double& target_hx_2, double& target_hz_2) {
  if (!model || !data) {
    return;
  }

  if (joint_q_b_ >= 0 && joint_q_d_ >= 0) {
    phi1_prev_ = data->qpos[model->jnt_qposadr[joint_q_d_]];
    phi2_prev_ = data->qpos[model->jnt_qposadr[joint_q_b_]];
    if (ForwardKinematics(phi1_prev_, phi2_prev_, branch_prev_, target_hx, target_hz)) {
      ClampTargetH(target_hx, target_hz);
    }
  }

  if (joint_q_b_2_ >= 0 && joint_q_d_2_ >= 0) {
    phi1_prev_2_ = -data->qpos[model->jnt_qposadr[joint_q_d_2_]];
    phi2_prev_2_ = -data->qpos[model->jnt_qposadr[joint_q_b_2_]];
    if (ForwardKinematics(phi1_prev_2_, phi2_prev_2_, branch_prev_2_,
                          target_hx_2, target_hz_2)) {
      ClampTargetH(target_hx_2, target_hz_2);
    }
  }
}

void WbrController::Apply(const mjModel* model, mjData* data,
                          double& target_hx, double& target_hz,
                          double& target_hx_2, double& target_hz_2) {
  if (!model || !data) {
    return;
  }

  ClampTargetH(target_hx, target_hz);
  ClampTargetH(target_hx_2, target_hz_2);

  if (act_q_b_ >= 0 && act_q_d_ >= 0 && joint_q_b_ >= 0 && joint_q_d_ >= 0) {
    if (data->time == 0.0) {
      phi1_prev_ = data->qpos[model->jnt_qposadr[joint_q_d_]];
      phi2_prev_ = data->qpos[model->jnt_qposadr[joint_q_b_]];
    }

    double phi1;
    double phi2;
    int branch = branch_prev_;
    if (InverseKinematics(target_hx, target_hz, phi1_prev_, phi2_prev_, phi1, phi2, branch)) {
      phi1 = UnwrapAngle(phi1, phi1_prev_);
      phi2 = UnwrapAngle(phi2, phi2_prev_);
      phi1_prev_ = phi1;
      phi2_prev_ = phi2;
      branch_prev_ = branch;
      data->ctrl[act_q_b_] = phi2;
      data->ctrl[act_q_d_] = phi1;
    }
  }

  if (act_q_b_2_ >= 0 && act_q_d_2_ >= 0 && joint_q_b_2_ >= 0 && joint_q_d_2_ >= 0) {
    if (data->time == 0.0) {
      phi1_prev_2_ = -data->qpos[model->jnt_qposadr[joint_q_d_2_]];
      phi2_prev_2_ = -data->qpos[model->jnt_qposadr[joint_q_b_2_]];
    }

    double phi1_2;
    double phi2_2;
    int branch_2 = branch_prev_2_;
    if (InverseKinematics(target_hx_2, target_hz_2,
                          phi1_prev_2_, phi2_prev_2_, phi1_2, phi2_2, branch_2)) {
      phi1_2 = UnwrapAngle(phi1_2, phi1_prev_2_);
      phi2_2 = UnwrapAngle(phi2_2, phi2_prev_2_);
      phi1_prev_2_ = phi1_2;
      phi2_prev_2_ = phi2_2;
      branch_prev_2_ = branch_2;
      data->ctrl[act_q_b_2_] = -phi2_2;
      data->ctrl[act_q_d_2_] = -phi1_2;
    }
  }

  if (joint_root_ >= 0 && body_plate_ >= 0 && act_wheel_ >= 0 && act_wheel_2_ >= 0) {
    double dt = model->opt.timestep;
    if (balance_initialized_) {
      dt = Clamp(data->time - balance_last_time_, 1e-4, 0.02);
    }
    if (data->time == 0.0 || data->time < balance_last_time_) {
      balance_integral_ = 0.0;
    }
    balance_last_time_ = data->time;
    balance_initialized_ = true;

    const int root_qposadr = model->jnt_qposadr[joint_root_];
    const int root_dofadr = model->jnt_dofadr[joint_root_];
    const double pitch = RootPitch(data, body_plate_);
    const double pitch_rate = data->qvel[root_dofadr + 4];
    const double x = data->qpos[root_qposadr + 0];
    const double x_velocity = data->qvel[root_dofadr + 0];

    balance_integral_ = Clamp(balance_integral_ + pitch * dt, -0.3, 0.3);
    double wheel_torque = kBalanceSign *
                          (kPitchKp * pitch + kPitchKi * balance_integral_ + kPitchKd * pitch_rate + 
                           kPositionKp * x + kVelocityKd * x_velocity);
    wheel_torque = Clamp(wheel_torque, -kWheelTorqueLimit, kWheelTorqueLimit);

    data->ctrl[act_wheel_] = kWheel1Sign * wheel_torque;
    data->ctrl[act_wheel_2_] = kWheel2Sign * wheel_torque;
  }
}

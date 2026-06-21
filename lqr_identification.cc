#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

#include <mujoco/mujoco.h>

namespace {
constexpr int kStateDim = 6;
constexpr int kInputDim = 2;
// With the current soft closed-chain constraints and full robot weight, a
// 0.20 m unloaded VMC command trims to an actual leg length near 0.18 m.
double g_target_length = 0.20;
double g_support_force_per_leg = -1.0;
constexpr double kLengthAB = 0.0945;
constexpr double kLengthBC = 0.1125;
constexpr double kLengthCD = 0.116;
constexpr double kLengthAD = 0.090;
constexpr double kLengthAG = 0.210;
constexpr double kLengthGH = 0.250;
constexpr double kJacobianStep = 1e-6;
constexpr double kLengthKp = 300.0;
constexpr double kLengthKd = 20.0;
constexpr double kSettleJointKp = 500.0;
constexpr double kSettleJointKd = 50.0;
constexpr double kSettleJointForceLimit = 200.0;
constexpr double kSettleThetaKp = 20.0;
constexpr double kSettleThetaKd = 10.0;
constexpr double kJointTorqueLimit = 20.0;
constexpr double kForceLimit = 150.0;
double g_rollout_duration = 0.003;
constexpr double kStatePositionPerturbation = 1e-3;
constexpr double kStateVelocityPerturbation = 2e-2;
constexpr double kInputPerturbation = 0.2;

using Matrix6 = std::array<std::array<double, kStateDim>, kStateDim>;
using Matrix62 = std::array<std::array<double, kInputDim>, kStateDim>;
using Gain26 = std::array<std::array<double, kStateDim>, kInputDim>;
using State = std::array<double, kStateDim>;

struct Ids {
  int root_joint;
  int root_dof;
  int root_qpos;
  int joint_b[2];
  int joint_d[2];
  int actuator_b[2];
  int actuator_d[2];
  int actuator_wheel[2];
  int body_a[2];
  int body_wheel[2];
  int body_plate;
  int geom_wheel[2];
};

struct LegState {
  double hx;
  double hz;
  double length;
  double length_rate;
  double current_angle;
  double current_angle_rate;
  double jacobian[2][2];
};

struct FullState {
  std::vector<mjtNum> qpos;
  std::vector<mjtNum> qvel;
};

struct ActualLegState {
  double rx;
  double rz;
  double rvx;
  double rvz;
  double length;
  double length_rate;
  double paper_theta;
  double paper_theta_rate;
};

double Clamp(double value, double lo, double hi) {
  return std::fmin(std::fmax(value, lo), hi);
}

double UnwrapNear(double angle, double reference) {
  while (angle - reference > M_PI) angle -= 2.0 * M_PI;
  while (angle - reference < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

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
                       double jacobian[2][2]);

bool InverseKinematics(double target_hx, double target_hz,
                       double phi1_initial, double phi2_initial,
                       double& phi1, double& phi2) {
  const std::array<std::array<double, 2>, 5> initial_guesses = {{
      {phi1_initial, phi2_initial},
      {1.05, 2.44},
      {-1.05, 2.44},
      {1.05, -2.44},
      {-1.05, -2.44},
  }};
  double best_distance = 1e20;
  bool found = false;
  for (int branch = 1; branch <= 2; ++branch) {
    for (const auto& guess : initial_guesses) {
      double p1 = guess[0];
      double p2 = guess[1];
      for (int iteration = 0; iteration < 80; ++iteration) {
        double hx;
        double hz;
        if (!ForwardKinematics(p1, p2, branch, hx, hz)) {
          break;
        }
        const double ex = hx - target_hx;
        const double ez = hz - target_hz;
        if (ex * ex + ez * ez < 1e-14) {
          const double distance = (p1 - phi1_initial) * (p1 - phi1_initial) +
                                  (p2 - phi2_initial) * (p2 - phi2_initial);
          if (distance < best_distance && branch == 1) {
            best_distance = distance;
            phi1 = p1;
            phi2 = p2;
            found = true;
          }
          break;
        }
        double jacobian[2][2];
        if (!NumericalJacobian(p1, p2, branch, jacobian)) {
          break;
        }
        const double determinant =
            jacobian[0][0] * jacobian[1][1] -
            jacobian[0][1] * jacobian[1][0];
        if (std::fabs(determinant) < 1e-10) {
          break;
        }
        const double dp1 =
            (jacobian[1][1] * ex - jacobian[0][1] * ez) / determinant;
        const double dp2 =
            (-jacobian[1][0] * ex + jacobian[0][0] * ez) / determinant;
        p1 -= 0.5 * dp1;
        p2 -= 0.5 * dp2;
      }
    }
  }
  return found;
}

bool NumericalJacobian(double phi1, double phi2, int branch,
                       double jacobian[2][2]) {
  double hx_plus;
  double hz_plus;
  double hx_minus;
  double hz_minus;
  if (!ForwardKinematics(phi1 + kJacobianStep, phi2, branch, hx_plus, hz_plus) ||
      !ForwardKinematics(phi1 - kJacobianStep, phi2, branch, hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][0] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][0] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  if (!ForwardKinematics(phi1, phi2 + kJacobianStep, branch, hx_plus, hz_plus) ||
      !ForwardKinematics(phi1, phi2 - kJacobianStep, branch, hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][1] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][1] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  return true;
}

bool FindIds(mjModel* model, Ids& ids) {
  ids.root_joint = mj_name2id(model, mjOBJ_JOINT, "root_free");
  ids.joint_b[0] = mj_name2id(model, mjOBJ_JOINT, "q_B");
  ids.joint_b[1] = mj_name2id(model, mjOBJ_JOINT, "q_B_2");
  ids.joint_d[0] = mj_name2id(model, mjOBJ_JOINT, "q_D");
  ids.joint_d[1] = mj_name2id(model, mjOBJ_JOINT, "q_D_2");
  ids.actuator_b[0] = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_B");
  ids.actuator_b[1] = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_B_2");
  ids.actuator_d[0] = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_D");
  ids.actuator_d[1] = mj_name2id(model, mjOBJ_ACTUATOR, "act_q_D_2");
  ids.actuator_wheel[0] = mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel");
  ids.actuator_wheel[1] = mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel_2");
  ids.body_a[0] = mj_name2id(model, mjOBJ_BODY, "A");
  ids.body_a[1] = mj_name2id(model, mjOBJ_BODY, "A_2");
  ids.body_wheel[0] = mj_name2id(model, mjOBJ_BODY, "H_wheel_body");
  ids.body_wheel[1] = mj_name2id(model, mjOBJ_BODY, "H_wheel_body_2");
  ids.body_plate = mj_name2id(model, mjOBJ_BODY, "plate");
  ids.geom_wheel[0] = mj_name2id(model, mjOBJ_GEOM, "H_wheel");
  ids.geom_wheel[1] = mj_name2id(model, mjOBJ_GEOM, "H_wheel_2");
  const int required[] = {
      ids.root_joint, ids.joint_b[0], ids.joint_b[1], ids.joint_d[0], ids.joint_d[1],
      ids.actuator_b[0], ids.actuator_b[1], ids.actuator_d[0], ids.actuator_d[1],
      ids.actuator_wheel[0], ids.actuator_wheel[1], ids.body_a[0], ids.body_a[1],
      ids.body_wheel[0], ids.body_wheel[1], ids.body_plate,
      ids.geom_wheel[0], ids.geom_wheel[1],
  };
  for (int id : required) {
    if (id < 0) {
      return false;
    }
  }
  ids.root_dof = model->jnt_dofadr[ids.root_joint];
  ids.root_qpos = model->jnt_qposadr[ids.root_joint];

  const int stand = mj_name2id(model, mjOBJ_GEOM, "leg_controller_stand");
  if (stand >= 0) {
    model->geom_contype[stand] = 0;
    model->geom_conaffinity[stand] = 0;
  }
  return true;
}

double CurrentBodyPitch(const mjData* data, int body_id) {
  const double* rotation = data->xmat + 9 * body_id;
  return std::atan2(rotation[6], rotation[0]);
}

bool ComputeLegState(const mjModel* model, const mjData* data, const Ids& ids,
                     int leg, LegState& state) {
  const double mirror = leg == 0 ? 1.0 : -1.0;
  const double phi1 = mirror * data->qpos[model->jnt_qposadr[ids.joint_d[leg]]];
  const double phi2 = mirror * data->qpos[model->jnt_qposadr[ids.joint_b[leg]]];
  const double dphi1 = mirror * data->qvel[model->jnt_dofadr[ids.joint_d[leg]]];
  const double dphi2 = mirror * data->qvel[model->jnt_dofadr[ids.joint_b[leg]]];
  if (!ForwardKinematics(phi1, phi2, 1, state.hx, state.hz) ||
      !NumericalJacobian(phi1, phi2, 1, state.jacobian)) {
    return false;
  }
  state.length = std::hypot(state.hx, state.hz);
  if (state.length < 1e-6) {
    return false;
  }
  const double vx = state.jacobian[0][0] * dphi1 + state.jacobian[0][1] * dphi2;
  const double vz = state.jacobian[1][0] * dphi1 + state.jacobian[1][1] * dphi2;
  state.length_rate = (state.hx * vx + state.hz * vz) / state.length;
  state.current_angle = std::atan2(state.hx, -state.hz);
  state.current_angle_rate =
      (-state.hz * vx + state.hx * vz) / (state.length * state.length);
  return true;
}

void ApplyVmc(mjData* data, const Ids& ids, int leg, const LegState& state,
              double axial_force, double current_angle_torque) {
  const double radial_x = state.hx / state.length;
  const double radial_z = state.hz / state.length;
  const double force_x = axial_force * radial_x -
                         current_angle_torque * radial_z / state.length;
  const double force_z = axial_force * radial_z +
                         current_angle_torque * radial_x / state.length;
  const double torque_phi1 =
      state.jacobian[0][0] * force_x + state.jacobian[1][0] * force_z;
  const double torque_phi2 =
      state.jacobian[0][1] * force_x + state.jacobian[1][1] * force_z;
  const double mirror = leg == 0 ? 1.0 : -1.0;
  data->ctrl[ids.actuator_b[leg]] =
      Clamp(mirror * torque_phi2, -kJointTorqueLimit, kJointTorqueLimit);
  data->ctrl[ids.actuator_d[leg]] =
      Clamp(mirror * torque_phi1, -kJointTorqueLimit, kJointTorqueLimit);
}

bool ApplyLegController(const mjModel* model, mjData* data, const Ids& ids,
                        double target_paper_theta, double paper_tp,
                        bool support_weight, bool settle_angle) {
  for (int leg = 0; leg < 2; ++leg) {
    LegState state{};
    if (!ComputeLegState(model, data, ids, leg, state)) {
      return false;
    }
    const double paper_theta = -state.current_angle;
    const double paper_theta_rate = -state.current_angle_rate;

    double axial_force = kLengthKp * (g_target_length - state.length) -
                         kLengthKd * state.length_rate;
    if (support_weight) {
      const double half_weight =
          0.5 * mj_getTotalmass(model) * std::fabs(model->opt.gravity[2]);
      const double support_force =
          g_support_force_per_leg != -1.0 ? g_support_force_per_leg : half_weight;
      axial_force += support_force / std::fmax(std::cos(paper_theta), 0.5);
    }
    axial_force = Clamp(axial_force, -kForceLimit, kForceLimit);

    double paper_leg_torque = 0.5 * paper_tp;
    if (settle_angle) {
      paper_leg_torque +=
          kSettleThetaKp * (target_paper_theta - paper_theta) -
          kSettleThetaKd * paper_theta_rate;
    }
    paper_leg_torque = Clamp(paper_leg_torque, -kSettleJointForceLimit,
                             kSettleJointForceLimit);

    ApplyVmc(data, ids, leg, state, axial_force, -paper_leg_torque);
  }
  return true;
}

void ApplyRootStabilizer(const mjModel* model, mjData* data, const Ids& ids,
                         double target_z) {
  constexpr double kp_position = 1000.0;
  constexpr double kd_position = 100.0;
  constexpr double kp_rotation = 300.0;
  constexpr double kd_rotation = 30.0;
  const int q = ids.root_qpos;
  const int v = ids.root_dof;
  data->qfrc_applied[v] = -kp_position * data->qpos[q] -
                          kd_position * data->qvel[v];
  data->qfrc_applied[v + 1] = -kp_position * data->qpos[q + 1] -
                              kd_position * data->qvel[v + 1];
  data->qfrc_applied[v + 2] =
      mj_getTotalmass(model) * std::fabs(model->opt.gravity[2]) +
      kp_position * (target_z - data->qpos[q + 2]) -
      kd_position * data->qvel[v + 2];
  data->qfrc_applied[v + 3] = -kd_rotation * data->qvel[v + 3];
  data->qfrc_applied[v + 4] =
      kp_rotation * CurrentBodyPitch(data, ids.body_plate) -
      kd_rotation * data->qvel[v + 4];
  data->qfrc_applied[v + 5] = -kd_rotation * data->qvel[v + 5];
}

double AveragePaperTheta(const mjModel* model, const mjData* data, const Ids& ids) {
  double sine_sum = 0.0;
  double cosine_sum = 0.0;
  for (int leg = 0; leg < 2; ++leg) {
    const double rx = data->xpos[3 * ids.body_wheel[leg]] -
                      data->xpos[3 * ids.body_a[leg]];
    const double rz = data->xpos[3 * ids.body_wheel[leg] + 2] -
                      data->xpos[3 * ids.body_a[leg] + 2];
    const double paper_theta = -std::atan2(rx, -rz);
    sine_sum += std::sin(paper_theta);
    cosine_sum += std::cos(paper_theta);
  }
  return std::atan2(sine_sum, cosine_sum);
}

double AverageLegLength(const mjModel* model, const mjData* data, const Ids& ids) {
  double length = 0.0;
  for (int leg = 0; leg < 2; ++leg) {
    const double rx = data->xpos[3 * ids.body_wheel[leg]] -
                      data->xpos[3 * ids.body_a[leg]];
    const double rz = data->xpos[3 * ids.body_wheel[leg] + 2] -
                      data->xpos[3 * ids.body_a[leg] + 2];
    length += 0.5 * std::hypot(rx, rz);
  }
  return length;
}

std::array<double, 2> ActualAverageH(const mjModel* model, const Ids& ids,
                                    const FullState& state) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, state.qpos.data(), model->nq);
  mju_copy(data->qvel, state.qvel.data(), model->nv);
  mj_forward(model, data);
  std::array<double, 2> h{};
  for (int leg = 0; leg < 2; ++leg) {
    h[0] += 0.5 * (data->xpos[3 * ids.body_wheel[leg]] -
                   data->xpos[3 * ids.body_a[leg]]);
    h[1] += 0.5 * (data->xpos[3 * ids.body_wheel[leg] + 2] -
                   data->xpos[3 * ids.body_a[leg] + 2]);
  }
  mj_deleteData(data);
  return h;
}

FullState SettleJointTargets(const mjModel* model, const Ids& ids,
                             const FullState& initial, double target_b,
                             double target_d, double duration) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, initial.qpos.data(), model->nq);
  mju_copy(data->qvel, initial.qvel.data(), model->nv);
  mj_forward(model, data);
  const int steps = static_cast<int>(duration / model->opt.timestep);
  for (int step = 0; step < steps; ++step) {
    mju_zero(data->ctrl, model->nu);
    mju_zero(data->qfrc_applied, model->nv);
    const double target_b_leg[2] = {target_b, -target_b};
    const double target_d_leg[2] = {target_d, -target_d};
    for (int leg = 0; leg < 2; ++leg) {
      const int qb = model->jnt_qposadr[ids.joint_b[leg]];
      const int qd = model->jnt_qposadr[ids.joint_d[leg]];
      const int vb = model->jnt_dofadr[ids.joint_b[leg]];
      const int vd = model->jnt_dofadr[ids.joint_d[leg]];
      data->qfrc_applied[vb] +=
          Clamp(kSettleJointKp * (target_b_leg[leg] - data->qpos[qb]) -
                    kSettleJointKd * data->qvel[vb],
                -kSettleJointForceLimit, kSettleJointForceLimit);
      data->qfrc_applied[vd] +=
          Clamp(kSettleJointKp * (target_d_leg[leg] - data->qpos[qd]) -
                    kSettleJointKd * data->qvel[vd],
                -kSettleJointForceLimit, kSettleJointForceLimit);
    }
    ApplyRootStabilizer(model, data, ids, 0.65);
    mj_step(model, data);
  }
  mju_zero(data->qvel, model->nv);
  mju_zero(data->ctrl, model->nu);
  mju_zero(data->qfrc_applied, model->nv);
  mj_forward(model, data);
  FullState result{
      std::vector<mjtNum>(data->qpos, data->qpos + model->nq),
      std::vector<mjtNum>(data->qvel, data->qvel + model->nv),
  };
  mj_deleteData(data);
  return result;
}

FullState SettleConfiguration(const mjModel* model, const Ids& ids,
                              double target_paper_theta, double* paper_tp_bias_out) {
  mjData* data = mj_makeData(model);
  const int key = mj_name2id(model, mjOBJ_KEY, "init");
  if (key >= 0) {
    mj_resetDataKeyframe(model, data, key);
  }
  data->qpos[ids.root_qpos] = 0.0;
  data->qpos[ids.root_qpos + 1] = 0.0;
  data->qpos[ids.root_qpos + 2] = 0.65;
  data->qpos[ids.root_qpos + 3] = 1.0;
  data->qpos[ids.root_qpos + 4] = 0.0;
  data->qpos[ids.root_qpos + 5] = 0.0;
  data->qpos[ids.root_qpos + 6] = 0.0;
  mju_zero(data->qvel, model->nv);
  const double target_hx = -g_target_length * std::sin(target_paper_theta);
  const double target_hz = -g_target_length * std::cos(target_paper_theta);
  double phi1;
  double phi2;
  if (!InverseKinematics(
          target_hx, target_hz,
          data->qpos[model->jnt_qposadr[ids.joint_d[0]]],
          data->qpos[model->jnt_qposadr[ids.joint_b[0]]], phi1, phi2)) {
    std::fprintf(stderr, "Could not solve IK for L0=%g theta=%g.\n",
                 g_target_length, target_paper_theta);
    std::exit(3);
  }
  const double bx = kLengthAB * std::cos(phi2);
  const double bz = -kLengthAB * std::sin(phi2);
  const double dx = kLengthAD * std::cos(phi1);
  const double dz = -kLengthAD * std::sin(phi1);
  const double center_distance = std::hypot(dx - bx, dz - bz);
  const double circle_a =
      (kLengthBC * kLengthBC - kLengthCD * kLengthCD +
       center_distance * center_distance) /
      (2.0 * center_distance);
  const double circle_h =
      std::sqrt(std::fmax(0.0, kLengthBC * kLengthBC - circle_a * circle_a));
  const double px = bx + circle_a * (dx - bx) / center_distance;
  const double pz = bz + circle_a * (dz - bz) / center_distance;
  const double cx = px - circle_h * (dz - bz) / center_distance;
  const double cz = pz + circle_h * (dx - bx) / center_distance;

  const int joint_bc = mj_name2id(model, mjOBJ_JOINT, "q_BC");
  const int joint_c = mj_name2id(model, mjOBJ_JOINT, "q_C");
  const int joint_de = mj_name2id(model, mjOBJ_JOINT, "q_DE");
  const int joint_e = mj_name2id(model, mjOBJ_JOINT, "q_E");
  const int joint_f = mj_name2id(model, mjOBJ_JOINT, "q_F");
  double gamma_bc = std::atan2(-(cz - bz), cx - bx);
  double gamma_cd = std::atan2(-(dz - cz), dx - cx);
  gamma_bc = UnwrapNear(
      gamma_bc, phi2 + data->qpos[model->jnt_qposadr[joint_bc]]);
  gamma_cd = UnwrapNear(
      gamma_cd, gamma_bc + data->qpos[model->jnt_qposadr[joint_c]]);
  const double q_bc = gamma_bc - phi2;
  const double q_c = gamma_cd - gamma_bc;
  const double q_de = gamma_cd - phi1;
  const double q_e = phi1 - gamma_cd;
  const double q_f = -M_PI - q_e;

  const int first_joints[7] = {
      ids.joint_b[0], joint_bc, joint_c, ids.joint_d[0], joint_de, joint_e, joint_f};
  const double first_values[7] = {phi2, q_bc, q_c, phi1, q_de, q_e, q_f};
  const char* second_names[7] = {
      "q_B_2", "q_BC_2", "q_C_2", "q_D_2", "q_DE_2", "q_E_2", "q_F_2"};
  for (int joint = 0; joint < 7; ++joint) {
    data->qpos[model->jnt_qposadr[first_joints[joint]]] = first_values[joint];
    const int second_joint = mj_name2id(model, mjOBJ_JOINT, second_names[joint]);
    data->qpos[model->jnt_qposadr[second_joint]] = -first_values[joint];
  }
  mj_forward(model, data);
  FullState result{
      std::vector<mjtNum>(data->qpos, data->qpos + model->nq),
      std::vector<mjtNum>(data->qvel, data->qvel + model->nv),
  };
  const std::array<double, 2> final_h = ActualAverageH(model, ids, result);
  const double final_length = std::hypot(final_h[0], final_h[1]);
  const double final_theta = -std::atan2(final_h[0], -final_h[1]);
  std::printf("  analytic closed-chain theta=% .8f L=% .8f\n",
              final_theta, final_length);
  if (std::fabs(final_length - g_target_length) > 2e-4 ||
      std::fabs(final_theta - target_paper_theta) > 2e-4) {
    std::fprintf(stderr,
                 "Could not reach L0=%.3f, theta=%g in the full XML constraints.\n",
                 g_target_length, target_paper_theta);
    std::exit(3);
  }
  if (paper_tp_bias_out) {
    *paper_tp_bias_out = 0.0;
  }
  mj_deleteData(data);
  return result;
}

void PlaceWheelsOnFloor(const mjModel* model, const Ids& ids, FullState& state) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, state.qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mj_forward(model, data);
  double bottom = 1e9;
  for (int leg = 0; leg < 2; ++leg) {
    const double center_z = data->geom_xpos[3 * ids.geom_wheel[leg] + 2];
    const double radius = model->geom_size[3 * ids.geom_wheel[leg]];
    bottom = std::fmin(bottom, center_z - radius);
  }
  state.qpos[ids.root_qpos + 2] -= bottom + 1e-5;
  std::fill(state.qvel.begin(), state.qvel.end(), 0.0);
  mj_deleteData(data);
}

State MeasureReducedState(const mjModel* model, mjData* data, const Ids& ids,
                          double x_origin) {
  State state{};
  mjtNum plate_velocity[6];
  mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_plate, plate_velocity, 0);
  const double body_pitch = CurrentBodyPitch(data, ids.body_plate);
  const double body_pitch_rate = -plate_velocity[1];
  double theta_sin = 0.0;
  double theta_cos = 0.0;
  double theta_rate = 0.0;
  for (int leg = 0; leg < 2; ++leg) {
    mjtNum wheel_velocity[6];
    mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_wheel[leg], wheel_velocity, 0);
    LegState leg_state{};
    if (!ComputeLegState(model, data, ids, leg, leg_state)) continue;
    const double world_leg_angle = leg_state.current_angle + body_pitch;
    theta_sin += std::sin(world_leg_angle);
    theta_cos += std::cos(world_leg_angle);
    theta_rate += 0.5 * (leg_state.current_angle_rate + body_pitch_rate);
    state[2] += 0.5 * data->xpos[3 * ids.body_wheel[leg]];
    state[3] += 0.5 * wheel_velocity[3];
  }
  state[0] = std::atan2(theta_sin, theta_cos);
  state[1] = theta_rate;
  state[2] -= x_origin;
  state[4] = body_pitch;
  state[5] = body_pitch_rate;
  return state;
}

State MeasureInitialState(const mjModel* model, const Ids& ids,
                          const FullState& initial, double x_origin) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, initial.qpos.data(), model->nq);
  mju_copy(data->qvel, initial.qvel.data(), model->nv);
  mj_forward(model, data);
  const State result = MeasureReducedState(model, data, ids, x_origin);
  mj_deleteData(data);
  return result;
}

std::array<double, 3> EvaluateStaticInput(
    const mjModel* model, const Ids& ids, const FullState& equilibrium,
    double support_force_per_leg, double paper_t, double paper_tp) {
  g_support_force_per_leg = support_force_per_leg;
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, equilibrium.qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mj_forward(model, data);
  mju_zero(data->ctrl, model->nu);
  mju_zero(data->qfrc_applied, model->nv);
  if (!ApplyLegController(model, data, ids, 0.0, paper_tp, true, false)) {
    mj_deleteData(data);
    return {1e9, 1e9, 1e9};
  }
  data->ctrl[ids.actuator_wheel[0]] = 0.5 * paper_t;
  data->ctrl[ids.actuator_wheel[1]] = -0.5 * paper_t;
  mj_step(model, data);
  mj_forward(model, data);
  const State state = MeasureReducedState(model, data, ids, 0.0);
  mj_deleteData(data);
  return {state[1], state[3], state[5]};
}

bool SolveCalibrationSystem(double matrix[3][3], const double right[3],
                            double solution[3]) {
  double augmented[3][4] = {};
  for (int row = 0; row < 3; ++row) {
    for (int col = 0; col < 3; ++col) augmented[row][col] = matrix[row][col];
    augmented[row][3] = right[row];
  }
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    for (int row = col + 1; row < 3; ++row) {
      if (std::fabs(augmented[row][col]) > std::fabs(augmented[pivot][col])) {
        pivot = row;
      }
    }
    if (std::fabs(augmented[pivot][col]) < 1e-12) return false;
    for (int item = col; item < 4; ++item) {
      std::swap(augmented[col][item], augmented[pivot][item]);
    }
    const double scale = augmented[col][col];
    for (int item = col; item < 4; ++item) augmented[col][item] /= scale;
    for (int row = 0; row < 3; ++row) {
      if (row == col) continue;
      const double factor = augmented[row][col];
      for (int item = col; item < 4; ++item) {
        augmented[row][item] -= factor * augmented[col][item];
      }
    }
  }
  for (int row = 0; row < 3; ++row) solution[row] = augmented[row][3];
  return true;
}

bool CalibrateStaticInputs(const mjModel* model, const Ids& ids,
                           const FullState& equilibrium,
                           double& support_force_per_leg,
                           double& paper_t_bias, double& paper_tp_bias) {
  const double half_weight =
      0.5 * mj_getTotalmass(model) * std::fabs(model->opt.gravity[2]);
  std::array<double, 3> variables = {half_weight, 0.0, 0.0};
  std::array<double, 3> search_steps = {1.0, 0.02, 0.02};
  constexpr double perturbations[3] = {0.1, 0.01, 0.01};
  auto norm = [](const std::array<double, 3>& residual) {
    return std::hypot(std::hypot(residual[0], residual[1]), residual[2]);
  };

  for (int iteration = 0; iteration < 60; ++iteration) {
    const auto residual = EvaluateStaticInput(
        model, ids, equilibrium, variables[0], variables[1], variables[2]);
    const double residual_norm = norm(residual);
    std::printf("  F0 calibration %2d: F0=% .8g T0=% .8g Tp0=% .8g "
                "dv=[% .5g % .5g % .5g] |dv|=%.9g\n",
                iteration, variables[0], variables[1], variables[2],
                residual[0], residual[1], residual[2], residual_norm);
    if (residual_norm < 1e-8) {
      support_force_per_leg = variables[0];
      paper_t_bias = variables[1];
      paper_tp_bias = variables[2];
      g_support_force_per_leg = support_force_per_leg;
      return true;
    }

    double jacobian[3][3] = {};
    for (int variable = 0; variable < 3; ++variable) {
      auto plus = variables;
      auto minus = variables;
      plus[variable] += perturbations[variable];
      minus[variable] -= perturbations[variable];
      const auto residual_plus = EvaluateStaticInput(
          model, ids, equilibrium, plus[0], plus[1], plus[2]);
      const auto residual_minus = EvaluateStaticInput(
          model, ids, equilibrium, minus[0], minus[1], minus[2]);
      for (int row = 0; row < 3; ++row) {
        jacobian[row][variable] =
            (residual_plus[row] - residual_minus[row]) /
            (2.0 * perturbations[variable]);
      }
    }
    const double right[3] = {-residual[0], -residual[1], -residual[2]};
    double delta[3] = {};
    if (!SolveCalibrationSystem(jacobian, right, delta)) return false;
    delta[0] = Clamp(delta[0], -0.5 * half_weight, 0.5 * half_weight);
    delta[1] = Clamp(delta[1], -2.0, 2.0);
    delta[2] = Clamp(delta[2], -4.0, 4.0);

    bool accepted = false;
    for (double scale = 1.0; scale >= 1.0 / 128.0; scale *= 0.5) {
      auto candidate = variables;
      for (int variable = 0; variable < 3; ++variable) {
        candidate[variable] += scale * delta[variable];
      }
      candidate[0] = Clamp(candidate[0], 0.0, 2.0 * half_weight);
      candidate[1] = Clamp(candidate[1], -10.0, 10.0);
      candidate[2] = Clamp(candidate[2], -20.0, 20.0);
      const auto candidate_residual = EvaluateStaticInput(
          model, ids, equilibrium, candidate[0], candidate[1], candidate[2]);
      if (norm(candidate_residual) < residual_norm) {
        variables = candidate;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      auto best = variables;
      double best_norm = residual_norm;
      for (int variable = 0; variable < 3; ++variable) {
        for (double direction : {-1.0, 1.0}) {
          auto candidate = variables;
          candidate[variable] += direction * search_steps[variable];
          candidate[0] = Clamp(candidate[0], 0.0, 2.0 * half_weight);
          candidate[1] = Clamp(candidate[1], -10.0, 10.0);
          candidate[2] = Clamp(candidate[2], -20.0, 20.0);
          const auto candidate_residual = EvaluateStaticInput(
              model, ids, equilibrium, candidate[0], candidate[1], candidate[2]);
          const double candidate_norm = norm(candidate_residual);
          if (candidate_norm < best_norm) {
            best = candidate;
            best_norm = candidate_norm;
          }
        }
      }
      if (best_norm < residual_norm) {
        variables = best;
      } else {
        for (double& step : search_steps) step *= 0.5;
        if (search_steps[0] < 1e-5 && search_steps[1] < 1e-6 &&
            search_steps[2] < 1e-6) {
          return false;
        }
      }
    }
  }
  const auto residual = EvaluateStaticInput(
      model, ids, equilibrium, variables[0], variables[1], variables[2]);
  if (norm(residual) < 1e-3) {
    support_force_per_leg = variables[0];
    paper_t_bias = variables[1];
    paper_tp_bias = variables[2];
    g_support_force_per_leg = support_force_per_leg;
    std::printf("  accepting practical calibration with |dv|=%.9g\n",
                norm(residual));
    return true;
  }
  return false;
}

FullState ApplyTrimPose(const Ids& ids, const FullState& floor_state,
                        const FullState& theta_plus,
                        const FullState& theta_minus,
                        const FullState& length_plus,
                        const FullState& length_minus, double z_offset,
                        double theta_offset, double phi_offset,
                        double length_offset) {
  FullState result = floor_state;
  const double theta_scale =
      theta_offset / kStatePositionPerturbation;
  for (size_t q = 0; q < result.qpos.size(); ++q) {
    result.qpos[q] += theta_scale * 0.5 *
                      (theta_plus.qpos[q] - theta_minus.qpos[q]);
    result.qpos[q] += length_offset / kStatePositionPerturbation * 0.5 *
                      (length_plus.qpos[q] - length_minus.qpos[q]);
  }
  result.qpos[ids.root_qpos + 2] += z_offset;
  result.qpos[ids.root_qpos + 3] = std::cos(0.5 * phi_offset);
  result.qpos[ids.root_qpos + 4] = 0.0;
  result.qpos[ids.root_qpos + 5] = std::sin(0.5 * phi_offset);
  result.qpos[ids.root_qpos + 6] = 0.0;
  std::fill(result.qvel.begin(), result.qvel.end(), 0.0);
  return result;
}

State TrimEquilibriumResidual(
    const mjModel* model, const Ids& ids, const FullState& floor_state,
    const FullState& theta_plus, const FullState& theta_minus,
    const FullState& length_plus, const FullState& length_minus,
    double z_offset, double paper_t, double paper_tp, double theta_offset,
    double phi_offset, double length_offset) {
  const FullState posed = ApplyTrimPose(ids, floor_state, theta_plus,
                                        theta_minus, length_plus, length_minus,
                                        z_offset, theta_offset, phi_offset,
                                        length_offset);
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, posed.qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mj_forward(model, data);
  const State initial = MeasureReducedState(model, data, ids, 0.0);
  mju_zero(data->ctrl, model->nu);
  mju_zero(data->qfrc_applied, model->nv);
  if (!ApplyLegController(model, data, ids, 0.0, paper_tp, true, false)) {
    mj_deleteData(data);
    State failed{};
    failed.fill(1e9);
    return failed;
  }
  data->ctrl[ids.actuator_wheel[0]] = 0.5 * paper_t;
  data->ctrl[ids.actuator_wheel[1]] = -0.5 * paper_t;
  const int trim_steps = std::max(
      1, static_cast<int>(std::lround(0.02 / model->opt.timestep)));
  for (int step = 0; step < trim_steps; ++step) {
    mju_zero(data->ctrl, model->nu);
    mju_zero(data->qfrc_applied, model->nv);
    if (!ApplyLegController(model, data, ids, 0.0, paper_tp, true, false)) {
      mj_deleteData(data);
      State failed{};
      failed.fill(1e9);
      return failed;
    }
    data->ctrl[ids.actuator_wheel[0]] = 0.5 * paper_t;
    data->ctrl[ids.actuator_wheel[1]] = -0.5 * paper_t;
    mj_step(model, data);
  }
  mj_forward(model, data);
  const State final = MeasureReducedState(model, data, ids, 0.0);
  mj_deleteData(data);
  const double duration = trim_steps * model->opt.timestep;
  return {
      (final[0] - initial[0]) / duration, final[1] - initial[1],
      (final[2] - initial[2]) / duration, final[3] - initial[3],
      (final[4] - initial[4]) / duration, final[5] - initial[5],
  };
}

constexpr int kTrimDim = 6;

bool SolveTrimSystem(double matrix[kTrimDim][kTrimDim],
                     const double right[kTrimDim],
                     double solution[kTrimDim]) {
  double augmented[kTrimDim][kTrimDim + 1] = {};
  for (int row = 0; row < kTrimDim; ++row) {
    for (int col = 0; col < kTrimDim; ++col) {
      augmented[row][col] = matrix[row][col];
    }
    augmented[row][kTrimDim] = right[row];
  }
  for (int col = 0; col < kTrimDim; ++col) {
    int pivot = col;
    for (int row = col + 1; row < kTrimDim; ++row) {
      if (std::fabs(augmented[row][col]) > std::fabs(augmented[pivot][col])) {
        pivot = row;
      }
    }
    if (std::fabs(augmented[pivot][col]) < 1e-12) {
      return false;
    }
    for (int item = col; item <= kTrimDim; ++item) {
      std::swap(augmented[col][item], augmented[pivot][item]);
    }
    const double scale = augmented[col][col];
    for (int item = col; item <= kTrimDim; ++item) {
      augmented[col][item] /= scale;
    }
    for (int row = 0; row < kTrimDim; ++row) {
      if (row == col) continue;
      const double factor = augmented[row][col];
      for (int item = col; item <= kTrimDim; ++item) {
        augmented[row][item] -= factor * augmented[col][item];
      }
    }
  }
  for (int row = 0; row < kTrimDim; ++row) {
    solution[row] = augmented[row][kTrimDim];
  }
  return true;
}

void RelaxInternalConfiguration(const mjModel* model, const Ids& ids,
                                FullState& state) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, state.qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mj_forward(model, data);
  const double target_wheel_x[2] = {
      data->xpos[3 * ids.body_wheel[0]],
      data->xpos[3 * ids.body_wheel[1]],
  };
  const int steps = static_cast<int>(2.0 / model->opt.timestep);
  for (int step = 0; step < steps; ++step) {
    mj_forward(model, data);
    mju_zero(data->ctrl, model->nu);
    mju_zero(data->qfrc_applied, model->nv);
    if (!ApplyLegController(model, data, ids, 0.0, 0.0, true, false)) {
      break;
    }
    // The fixture prevents the unstable planar modes while leaving vertical
    // motion completely free, so wheel contact and VMC carry the real weight.
    const int q = ids.root_qpos;
    const int v = ids.root_dof;
    data->qfrc_applied[v] = -1.0e4 * data->qpos[q] -
                            200.0 * data->qvel[v];
    data->qfrc_applied[v + 1] = -1.0e4 * data->qpos[q + 1] -
                                200.0 * data->qvel[v + 1];
    data->qfrc_applied[v + 3] = -30.0 * data->qvel[v + 3];
    data->qfrc_applied[v + 4] =
        300.0 * CurrentBodyPitch(data, ids.body_plate) -
        30.0 * data->qvel[v + 4];
    data->qfrc_applied[v + 5] = -30.0 * data->qvel[v + 5];
    for (int leg = 0; leg < 2; ++leg) {
      mjtNum velocity[6];
      mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_wheel[leg],
                        velocity, 0);
      const mjtNum force[3] = {
          -1.0e4 * (data->xpos[3 * ids.body_wheel[leg]] -
                     target_wheel_x[leg]) -
              200.0 * velocity[3],
          0.0, 0.0};
      const mjtNum torque[3] = {0.0, 0.0, 0.0};
      mj_applyFT(model, data, force, torque,
                 data->xpos + 3 * ids.body_wheel[leg], ids.body_wheel[leg],
                 data->qfrc_applied);
    }
    mj_step(model, data);
  }
  mju_copy(state.qpos.data(), data->qpos, model->nq);
  std::fill(state.qvel.begin(), state.qvel.end(), 0.0);
  mj_deleteData(data);
}

bool TrimDynamicEquilibrium(const mjModel* model, const Ids& ids,
                            FullState& equilibrium,
                            const FullState& theta_plus,
                            const FullState& theta_minus,
                            const FullState& length_plus,
                            const FullState& length_minus,
                            double& paper_t_bias, double& paper_tp_bias,
                            double& z_offset, double& theta_offset,
                            double& phi_offset, double& length_offset) {
  std::array<double, kTrimDim> variables = {
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  constexpr double steps[kTrimDim] = {
      1e-5, 1e-3, 1e-3, 1e-4, 1e-4, 1e-5};
  std::array<double, kTrimDim> search_steps = {
      2e-4, 0.05, 0.05, 0.002, 0.002, 0.001};
  auto norm = [](const State& residual) {
    double sum = 0.0;
    for (double value : residual) sum += value * value;
    return std::sqrt(sum);
  };

  for (int iteration = 0; iteration < 100; ++iteration) {
    const auto residual = TrimEquilibriumResidual(
        model, ids, equilibrium, theta_plus, theta_minus, length_plus,
        length_minus, variables[0], variables[1], variables[2], variables[3],
        variables[4], variables[5]);
    const double residual_norm = norm(residual);
    std::printf("  trim %2d: z=% .7g T=% .7g Tp=% .7g th=% .6g ph=% .6g "
                "dL=% .6g "
                "posrate=[% .3g % .3g % .3g] vel=[% .3g % .3g % .3g] "
                "|r|=%.9g\n",
                iteration, variables[0], variables[1], variables[2],
                variables[3], variables[4], variables[5],
                residual[0], residual[2], residual[4], residual[1],
                residual[3], residual[5], residual_norm);
    if (residual_norm < 1e-5) {
      z_offset = variables[0];
      paper_t_bias = variables[1];
      paper_tp_bias = variables[2];
      theta_offset = variables[3];
      phi_offset = variables[4];
      length_offset = variables[5];
      equilibrium = ApplyTrimPose(
          ids, equilibrium, theta_plus, theta_minus, length_plus, length_minus,
          z_offset, theta_offset, phi_offset, length_offset);
      return true;
    }

    double jacobian[kStateDim][kTrimDim] = {};
    for (int variable = 0; variable < kTrimDim; ++variable) {
      auto plus = variables;
      auto minus = variables;
      plus[variable] += steps[variable];
      minus[variable] -= steps[variable];
      const auto residual_plus = TrimEquilibriumResidual(
          model, ids, equilibrium, theta_plus, theta_minus, length_plus,
          length_minus, plus[0], plus[1], plus[2], plus[3], plus[4], plus[5]);
      const auto residual_minus = TrimEquilibriumResidual(
          model, ids, equilibrium, theta_plus, theta_minus, length_plus,
          length_minus, minus[0], minus[1], minus[2], minus[3], minus[4],
          minus[5]);
      for (int row = 0; row < kStateDim; ++row) {
        jacobian[row][variable] =
            (residual_plus[row] - residual_minus[row]) /
            (2.0 * steps[variable]);
      }
    }
    double normal[kTrimDim][kTrimDim] = {};
    double right[kTrimDim] = {};
    for (int row = 0; row < kStateDim; ++row) {
      for (int col = 0; col < kTrimDim; ++col) {
        right[col] -= jacobian[row][col] * residual[row];
        for (int other = 0; other < kTrimDim; ++other) {
          normal[col][other] += jacobian[row][col] * jacobian[row][other];
        }
      }
    }
    double largest_diagonal = 0.0;
    for (int variable = 0; variable < kTrimDim; ++variable) {
      largest_diagonal =
          std::fmax(largest_diagonal, normal[variable][variable]);
    }
    for (int variable = 0; variable < kTrimDim; ++variable) {
      normal[variable][variable] += 1e-8 * largest_diagonal;
    }
    double delta[kTrimDim] = {};
    if (!SolveTrimSystem(normal, right, delta)) return false;
    delta[0] = Clamp(delta[0], -1e-3, 1e-3);
    delta[1] = Clamp(delta[1], -1.0, 1.0);
    delta[2] = Clamp(delta[2], -1.0, 1.0);
    delta[3] = Clamp(delta[3], -0.02, 0.02);
    delta[4] = Clamp(delta[4], -0.02, 0.02);
    delta[5] = Clamp(delta[5], -0.005, 0.005);

    bool accepted = false;
    for (double scale = 1.0; scale >= 1.0 / 64.0; scale *= 0.5) {
      auto candidate = variables;
      for (int variable = 0; variable < kTrimDim; ++variable) {
        candidate[variable] += scale * delta[variable];
      }
      candidate[0] = Clamp(candidate[0], -0.01, 0.002);
      candidate[1] = Clamp(candidate[1], -10.0, 10.0);
      candidate[2] = Clamp(candidate[2], -20.0, 20.0);
      candidate[3] = Clamp(candidate[3], -0.15, 0.15);
      candidate[4] = Clamp(candidate[4], -0.15, 0.15);
      candidate[5] = Clamp(candidate[5], -0.03, 0.03);
      const auto candidate_residual = TrimEquilibriumResidual(
          model, ids, equilibrium, theta_plus, theta_minus, length_plus,
          length_minus, candidate[0], candidate[1], candidate[2], candidate[3],
          candidate[4], candidate[5]);
      if (norm(candidate_residual) < residual_norm) {
        variables = candidate;
        accepted = true;
        break;
      }
    }
    if (!accepted) {
      auto best = variables;
      double best_norm = residual_norm;
      for (int variable = 0; variable < kTrimDim; ++variable) {
        for (double direction : {-1.0, 1.0}) {
          auto candidate = variables;
          candidate[variable] += direction * search_steps[variable];
          candidate[0] = Clamp(candidate[0], -0.01, 0.002);
          candidate[1] = Clamp(candidate[1], -10.0, 10.0);
          candidate[2] = Clamp(candidate[2], -20.0, 20.0);
          candidate[3] = Clamp(candidate[3], -0.15, 0.15);
          candidate[4] = Clamp(candidate[4], -0.15, 0.15);
          candidate[5] = Clamp(candidate[5], -0.03, 0.03);
          const auto candidate_residual = TrimEquilibriumResidual(
              model, ids, equilibrium, theta_plus, theta_minus, length_plus,
              length_minus, candidate[0], candidate[1], candidate[2],
              candidate[3], candidate[4], candidate[5]);
          const double candidate_norm = norm(candidate_residual);
          if (candidate_norm < best_norm) {
            best = candidate;
            best_norm = candidate_norm;
          }
        }
      }
      if (best_norm < residual_norm) {
        variables = best;
      } else {
        for (double& step : search_steps) step *= 0.5;
        if (search_steps[0] < 1e-9 && search_steps[1] < 1e-6 &&
            search_steps[2] < 1e-6 && search_steps[3] < 1e-7 &&
            search_steps[4] < 1e-7 && search_steps[5] < 1e-7) {
          return false;
        }
      }
    }
  }
  return false;
}

State Rollout(const mjModel* model, const Ids& ids, const FullState& initial,
              double paper_t, double paper_tp, double paper_t_bias,
              double paper_tp_bias, double x_origin) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, initial.qpos.data(), model->nq);
  mju_copy(data->qvel, initial.qvel.data(), model->nv);
  mj_forward(model, data);
  const int steps =
      std::max(1, static_cast<int>(std::lround(g_rollout_duration /
                                               model->opt.timestep)));
  for (int step = 0; step < steps; ++step) {
    mju_zero(data->ctrl, model->nu);
    mju_zero(data->qfrc_applied, model->nv);
    if (!ApplyLegController(model, data, ids, 0.0,
                            paper_tp_bias + paper_tp, true, false)) {
      std::fprintf(stderr, "Leg kinematics failed during identification rollout.\n");
      std::exit(2);
    }
    data->ctrl[ids.actuator_wheel[0]] = 0.5 * (paper_t_bias + paper_t);
    data->ctrl[ids.actuator_wheel[1]] = -0.5 * (paper_t_bias + paper_t);
    mj_step(model, data);
  }
  // mj_step finishes with newly integrated qpos/qvel, while Cartesian body
  // caches still describe the dynamics evaluation before that integration.
  mj_forward(model, data);
  const State result = MeasureReducedState(model, data, ids, x_origin);
  mj_deleteData(data);
  return result;
}

double WheelCenterX(const mjModel* model, const Ids& ids, const FullState& state) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, state.qpos.data(), model->nq);
  mju_copy(data->qvel, state.qvel.data(), model->nv);
  mj_forward(model, data);
  const double x = 0.5 * (data->xpos[3 * ids.body_wheel[0]] +
                          data->xpos[3 * ids.body_wheel[1]]);
  mj_deleteData(data);
  return x;
}

bool Invert6(const Matrix6& input, Matrix6& inverse) {
  double augmented[kStateDim][2 * kStateDim] = {};
  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < kStateDim; ++col) {
      augmented[row][col] = input[row][col];
      augmented[row][kStateDim + col] = row == col ? 1.0 : 0.0;
    }
  }
  for (int col = 0; col < kStateDim; ++col) {
    int pivot = col;
    for (int row = col + 1; row < kStateDim; ++row) {
      if (std::fabs(augmented[row][col]) > std::fabs(augmented[pivot][col])) {
        pivot = row;
      }
    }
    if (std::fabs(augmented[pivot][col]) < 1e-10) {
      return false;
    }
    for (int j = 0; j < 2 * kStateDim; ++j) {
      std::swap(augmented[col][j], augmented[pivot][j]);
    }
    const double scale = augmented[col][col];
    for (int j = 0; j < 2 * kStateDim; ++j) {
      augmented[col][j] /= scale;
    }
    for (int row = 0; row < kStateDim; ++row) {
      if (row == col) {
        continue;
      }
      const double factor = augmented[row][col];
      for (int j = 0; j < 2 * kStateDim; ++j) {
        augmented[row][j] -= factor * augmented[col][j];
      }
    }
  }
  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < kStateDim; ++col) {
      inverse[row][col] = augmented[row][kStateDim + col];
    }
  }
  return true;
}

Matrix6 Multiply(const Matrix6& left, const Matrix6& right) {
  Matrix6 result{};
  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < kStateDim; ++col) {
      for (int k = 0; k < kStateDim; ++k) {
        result[row][col] += left[row][k] * right[k][col];
      }
    }
  }
  return result;
}

Gain26 SolveDiscreteLqr(const Matrix6& a, const Matrix62& b) {
  constexpr double q[kStateDim] = {1, 1, 500, 100, 5000, 1};
  constexpr double r[kInputDim] = {1, 0.25};
  Matrix6 p{};
  for (int i = 0; i < kStateDim; ++i) p[i][i] = q[i];

  Gain26 gain{};
  for (int iteration = 0; iteration < 200000; ++iteration) {
    Matrix62 pb{};
    for (int row = 0; row < kStateDim; ++row) {
      for (int input = 0; input < kInputDim; ++input) {
        for (int k = 0; k < kStateDim; ++k) {
          pb[row][input] += p[row][k] * b[k][input];
        }
      }
    }
    double s[2][2] = {{r[0], 0.0}, {0.0, r[1]}};
    for (int i = 0; i < kInputDim; ++i) {
      for (int j = 0; j < kInputDim; ++j) {
        for (int k = 0; k < kStateDim; ++k) {
          s[i][j] += b[k][i] * pb[k][j];
        }
      }
    }
    const double determinant = s[0][0] * s[1][1] - s[0][1] * s[1][0];
    const double inverse[2][2] = {
        {s[1][1] / determinant, -s[0][1] / determinant},
        {-s[1][0] / determinant, s[0][0] / determinant},
    };
    double btpa[kInputDim][kStateDim] = {};
    for (int input = 0; input < kInputDim; ++input) {
      for (int col = 0; col < kStateDim; ++col) {
        for (int row = 0; row < kStateDim; ++row) {
          for (int k = 0; k < kStateDim; ++k) {
            btpa[input][col] += b[row][input] * p[row][k] * a[k][col];
          }
        }
      }
    }
    for (int input = 0; input < kInputDim; ++input) {
      for (int col = 0; col < kStateDim; ++col) {
        gain[input][col] = inverse[input][0] * btpa[0][col] +
                           inverse[input][1] * btpa[1][col];
      }
    }
    Matrix6 next{};
    double max_change = 0.0;
    for (int row = 0; row < kStateDim; ++row) {
      for (int col = 0; col < kStateDim; ++col) {
        for (int i = 0; i < kStateDim; ++i) {
          for (int j = 0; j < kStateDim; ++j) {
            next[row][col] += a[i][row] * p[i][j] * a[j][col];
          }
        }
        for (int input = 0; input < kInputDim; ++input) {
          double atpb = 0.0;
          for (int k = 0; k < kStateDim; ++k) {
            atpb += a[k][row] * pb[k][input];
          }
          next[row][col] -= atpb * gain[input][col];
        }
        if (row == col) next[row][col] += q[row];
        max_change = std::fmax(max_change,
                               std::fabs(next[row][col] - p[row][col]));
      }
    }
    p = next;
    if (max_change < 1e-10) break;
  }
  return gain;
}

std::array<double, kStateDim> ControllabilitySingularValues(
    const Matrix6& a, const Matrix62& b) {
  double controllability[kStateDim][kStateDim * kInputDim] = {};
  Matrix62 block = b;
  for (int power = 0; power < kStateDim; ++power) {
    for (int row = 0; row < kStateDim; ++row) {
      for (int input = 0; input < kInputDim; ++input) {
        controllability[row][power * kInputDim + input] = block[row][input];
      }
    }
    Matrix62 next{};
    for (int row = 0; row < kStateDim; ++row) {
      for (int input = 0; input < kInputDim; ++input) {
        for (int k = 0; k < kStateDim; ++k) {
          next[row][input] += a[row][k] * block[k][input];
        }
      }
    }
    block = next;
  }

  // Powers of A can differ by many orders of magnitude. Column scaling does
  // not change rank and keeps the small 6x6 Gram matrix well conditioned.
  for (int col = 0; col < kStateDim * kInputDim; ++col) {
    double norm = 0.0;
    for (int row = 0; row < kStateDim; ++row) {
      norm = std::hypot(norm, controllability[row][col]);
    }
    if (norm > 0.0) {
      for (int row = 0; row < kStateDim; ++row) {
        controllability[row][col] /= norm;
      }
    }
  }

  Matrix6 gram{};
  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < kStateDim; ++col) {
      for (int sample = 0; sample < kStateDim * kInputDim; ++sample) {
        gram[row][col] += controllability[row][sample] *
                          controllability[col][sample];
      }
    }
  }

  // Jacobi eigenvalue iteration for the symmetric positive semidefinite Gram
  // matrix. Its eigenvalues are the squared singular values of C.
  for (int iteration = 0; iteration < 100; ++iteration) {
    int p = 0;
    int q = 1;
    double largest = 0.0;
    for (int row = 0; row < kStateDim; ++row) {
      for (int col = row + 1; col < kStateDim; ++col) {
        if (std::fabs(gram[row][col]) > largest) {
          largest = std::fabs(gram[row][col]);
          p = row;
          q = col;
        }
      }
    }
    if (largest < 1e-14) {
      break;
    }
    const double angle = 0.5 * std::atan2(
        2.0 * gram[p][q], gram[q][q] - gram[p][p]);
    const double cosine = std::cos(angle);
    const double sine = std::sin(angle);
    const double app = gram[p][p];
    const double aqq = gram[q][q];
    const double apq = gram[p][q];
    gram[p][p] = cosine * cosine * app - 2.0 * sine * cosine * apq +
                 sine * sine * aqq;
    gram[q][q] = sine * sine * app + 2.0 * sine * cosine * apq +
                 cosine * cosine * aqq;
    gram[p][q] = gram[q][p] = 0.0;
    for (int index = 0; index < kStateDim; ++index) {
      if (index == p || index == q) {
        continue;
      }
      const double aip = gram[index][p];
      const double aiq = gram[index][q];
      gram[index][p] = gram[p][index] = cosine * aip - sine * aiq;
      gram[index][q] = gram[q][index] = sine * aip + cosine * aiq;
    }
  }

  std::array<double, kStateDim> singular_values{};
  for (int index = 0; index < kStateDim; ++index) {
    singular_values[index] = std::sqrt(std::fmax(0.0, gram[index][index]));
  }
  std::sort(singular_values.begin(), singular_values.end(), std::greater<double>());
  return singular_values;
}

int NumericalRank(const std::array<double, kStateDim>& singular_values) {
  const double tolerance = singular_values.front() * 1e-8;
  int rank = 0;
  for (double value : singular_values) {
    rank += value > tolerance ? 1 : 0;
  }
  return rank;
}

FullState CombinePerturbations(
    const mjModel* model, const Ids& ids, const FullState& equilibrium,
    const std::array<FullState, kStateDim>& plus,
    const std::array<FullState, kStateDim>& minus,
    const std::array<double, kStateDim>& coefficients) {
  FullState result = equilibrium;
  for (int state = 0; state < kStateDim; ++state) {
    for (int q = 0; q < model->nq; ++q) {
      result.qpos[q] += coefficients[state] * 0.5 *
                        (plus[state].qpos[q] - minus[state].qpos[q]);
    }
    for (int v = 0; v < model->nv; ++v) {
      result.qvel[v] += coefficients[state] * 0.5 *
                        (plus[state].qvel[v] - minus[state].qvel[v]);
    }
  }
  mju_normalize4(result.qpos.data() + ids.root_qpos + 3);
  return result;
}

void RunNonlinearClosedLoopTest(
    const mjModel* model, const Ids& ids, const FullState& equilibrium,
    const std::array<FullState, kStateDim>& plus,
    const std::array<FullState, kStateDim>& minus, double paper_t_bias,
    double paper_tp_bias, double x_origin, const Matrix62& bd,
    const Gain26& gain, const char* gain_name) {
  struct TestCase {
    const char* name;
    std::array<double, kStateDim> coefficients;
  };
  const std::array<TestCase, 5> tests = {{
      {"zero", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {"theta", {10.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {"phi", {0.0, 0.0, 0.0, 0.0, 10.0, 0.0}},
      {"position", {0.0, 0.0, 20.0, 0.0, 0.0, 0.0}},
      {"combined", {8.0, 2.0, 10.0, 2.0, -6.0, 2.0}},
  }};
  const State reference =
      MeasureInitialState(model, ids, equilibrium, x_origin);
  const State zero_output =
      Rollout(model, ids, equilibrium, 0.0, 0.0, paper_t_bias,
              paper_tp_bias, x_origin);
  double normal[kInputDim][kInputDim] = {};
  double right[kInputDim] = {};
  constexpr int velocity_rows[3] = {1, 3, 5};
  for (int row : velocity_rows) {
    const double drift = zero_output[row] - reference[row];
    for (int i = 0; i < kInputDim; ++i) {
      right[i] -= bd[row][i] * drift;
      for (int j = 0; j < kInputDim; ++j) {
        normal[i][j] += bd[row][i] * bd[row][j];
      }
    }
  }
  const double determinant =
      normal[0][0] * normal[1][1] - normal[0][1] * normal[1][0];
  const double trim[kInputDim] = {
      (right[0] * normal[1][1] - normal[0][1] * right[1]) / determinant,
      (normal[0][0] * right[1] - right[0] * normal[1][0]) / determinant,
  };
  const int steps = static_cast<int>(std::ceil(5.0 / model->opt.timestep));

  std::printf("\nNONLINEAR closed-loop validation with %s LQR K:\n", gain_name);
  std::printf("  zero-input %.3g ms velocity drift: [% .6g % .6g % .6g]\n",
              1000.0 * g_rollout_duration,
              zero_output[1] - reference[1], zero_output[3] - reference[3],
              zero_output[5] - reference[5]);
  std::printf("  least-squares trim [T0 Tp0]: [% .6g % .6g] N.m\n",
              trim[0], trim[1]);
  std::printf("  case       upright  max|T|  max|Tp|  final theta  final x    final phi\n");
  for (const TestCase& test : tests) {
    const FullState initial = CombinePerturbations(
        model, ids, equilibrium, plus, minus, test.coefficients);
    mjData* data = mj_makeData(model);
    mju_copy(data->qpos, initial.qpos.data(), model->nq);
    mju_copy(data->qvel, initial.qvel.data(), model->nv);
    mj_forward(model, data);
    double max_t = 0.0;
    double max_tp = 0.0;
    bool valid = true;
    for (int step = 0; step < steps; ++step) {
      mj_forward(model, data);
      const State state = MeasureReducedState(model, data, ids, x_origin);
      double input[kInputDim] = {0.0, 0.0};
      for (int input_index = 0; input_index < kInputDim; ++input_index) {
        for (int state_index = 0; state_index < kStateDim; ++state_index) {
          input[input_index] -= gain[input_index][state_index] *
                                (state[state_index] - reference[state_index]);
        }
      }
      input[0] = Clamp(input[0], -10.0, 10.0);
      input[1] = Clamp(input[1], -20.0, 20.0);
      max_t = std::fmax(max_t, std::fabs(input[0]));
      max_tp = std::fmax(max_tp, std::fabs(input[1]));

      mju_zero(data->ctrl, model->nu);
      mju_zero(data->qfrc_applied, model->nv);
      if (!ApplyLegController(model, data, ids, 0.0,
                              paper_tp_bias + input[1], true, false)) {
        valid = false;
        break;
      }
      data->ctrl[ids.actuator_wheel[0]] = 0.5 * (paper_t_bias + input[0]);
      data->ctrl[ids.actuator_wheel[1]] = -0.5 * (paper_t_bias + input[0]);
      mj_step(model, data);
      if (!std::isfinite(data->qpos[0])) {
        valid = false;
        break;
      }
    }
    mj_forward(model, data);
    const State final_state = MeasureReducedState(model, data, ids, x_origin);
    const bool upright = valid && std::fabs(final_state[0]) < 0.15 &&
                         std::fabs(final_state[4]) < 0.15;
    std::printf("  %-10s %-7s %7.3f %8.3f %12.6f %10.6f %11.6f\n",
                test.name, upright ? "PASS" : "FAIL", max_t, max_tp,
                final_state[0], final_state[2], final_state[4]);
    mj_deleteData(data);
  }
}

void ValidateLinearModel(
    const mjModel* model, const Ids& ids, const FullState& equilibrium,
    const std::array<FullState, kStateDim>& plus,
    const std::array<FullState, kStateDim>& minus, const Matrix6& ad,
    const Matrix62& bd, double paper_t_bias, double paper_tp_bias,
    double x_origin,
    double state_scale, double input_scale, const char* label) {
  constexpr int kCases = 8;
  constexpr double coefficients[kCases][kStateDim + kInputDim] = {
      { 0.8, -0.3,  0.4,  0.2, -0.6,  0.5,  0.7, -0.4},
      {-0.5,  0.7, -0.2,  0.6,  0.3, -0.8, -0.5,  0.8},
      { 0.2,  0.5,  0.8, -0.4,  0.7,  0.1,  0.3,  0.6},
      {-0.7, -0.2,  0.5,  0.8, -0.1,  0.4, -0.8, -0.2},
      { 0.4, -0.8, -0.6,  0.1,  0.5,  0.7,  0.6,  0.3},
      {-0.3,  0.4,  0.1, -0.7, -0.8,  0.6, -0.2, -0.7},
      { 0.6,  0.1, -0.7,  0.5,  0.2, -0.4,  0.9,  0.1},
      {-0.1, -0.6,  0.3, -0.5,  0.8, -0.2,  0.1, -0.9},
  };
  constexpr double state_scales[kStateDim] = {
      kStatePositionPerturbation, kStateVelocityPerturbation,
      kStatePositionPerturbation, kStateVelocityPerturbation,
      kStatePositionPerturbation, kStateVelocityPerturbation,
  };

  const State equilibrium_initial =
      MeasureInitialState(model, ids, equilibrium, x_origin);
  const State equilibrium_final =
      Rollout(model, ids, equilibrium, 0.0, 0.0, paper_t_bias,
              paper_tp_bias, x_origin);
  State squared_error{};
  State max_error{};
  double normalized_squared_error = 0.0;

  for (int test = 0; test < kCases; ++test) {
    FullState initial = equilibrium;
    for (int state = 0; state < kStateDim; ++state) {
      const double coefficient = state_scale * coefficients[test][state];
      for (int q = 0; q < model->nq; ++q) {
        initial.qpos[q] +=
            coefficient * 0.5 * (plus[state].qpos[q] - minus[state].qpos[q]);
      }
      for (int v = 0; v < model->nv; ++v) {
        initial.qvel[v] +=
            coefficient * 0.5 * (plus[state].qvel[v] - minus[state].qvel[v]);
      }
    }
    mju_normalize4(initial.qpos.data() + ids.root_qpos + 3);

    const double input[kInputDim] = {
        input_scale * kInputPerturbation * coefficients[test][kStateDim],
        input_scale * kInputPerturbation *
            coefficients[test][kStateDim + 1],
    };
    const State initial_reduced =
        MeasureInitialState(model, ids, initial, x_origin);
    const State actual = Rollout(model, ids, initial, input[0], input[1],
                                 paper_t_bias, paper_tp_bias, x_origin);
    for (int row = 0; row < kStateDim; ++row) {
      double predicted_delta = 0.0;
      for (int col = 0; col < kStateDim; ++col) {
        predicted_delta +=
            ad[row][col] * (initial_reduced[col] - equilibrium_initial[col]);
      }
      for (int input_index = 0; input_index < kInputDim; ++input_index) {
        predicted_delta += bd[row][input_index] * input[input_index];
      }
      const double error = equilibrium_final[row] + predicted_delta - actual[row];
      squared_error[row] += error * error;
      max_error[row] = std::fmax(max_error[row], std::fabs(error));
      const double normalized = error / state_scales[row];
      normalized_squared_error += normalized * normalized;
    }
  }

  static constexpr const char* names[kStateDim] = {
      "theta", "dtheta", "x", "dx", "phi", "dphi"};
  std::printf("\n%s mixed-perturbation validation (%d held-out cases):\n", label,
              kCases);
  std::printf("  state       RMSE          max|error|\n");
  for (int row = 0; row < kStateDim; ++row) {
    std::printf("  %-6s  % .9g   % .9g\n", names[row],
                std::sqrt(squared_error[row] / kCases), max_error[row]);
  }
  const double normalized_rmse =
      std::sqrt(normalized_squared_error / (kCases * kStateDim));
  std::printf("  overall normalized RMSE: %.6g of identification perturbation\n",
              normalized_rmse);
}

void PrintMatrix(const char* name, const Matrix6& matrix) {
  std::printf("\n%s = [\n", name);
  for (int row = 0; row < kStateDim; ++row) {
    std::printf("  ");
    for (int col = 0; col < kStateDim; ++col) {
      std::printf("% .9g%s", matrix[row][col], col + 1 == kStateDim ? "" : " ");
    }
    std::printf("%s\n", row + 1 == kStateDim ? "" : ";");
  }
  std::printf("];\n");
}

void PrintMatrix(const char* name, const Matrix62& matrix) {
  std::printf("\n%s = [\n", name);
  for (int row = 0; row < kStateDim; ++row) {
    std::printf("  % .9g % .9g%s\n", matrix[row][0], matrix[row][1],
                row + 1 == kStateDim ? "" : ";");
  }
  std::printf("];\n");
}
}  // namespace

int main(int argc, char** argv) {
  const char* model_path = argc > 1 ? argv[1] : "wbr_free.xml";
  if (argc > 2) {
    g_target_length = std::atof(argv[2]);
  }
  if (argc > 3) {
    g_rollout_duration = std::atof(argv[3]);
  }
  if (g_target_length <= 0.0) {
    std::fprintf(stderr, "Target leg length must be positive.\n");
    return 1;
  }
  if (g_rollout_duration <= 0.0) {
    std::fprintf(stderr, "Identification horizon must be positive.\n");
    return 1;
  }
  char error[1024] = {};
  mjModel* model = mj_loadXML(model_path, nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Could not load %s: %s\n", model_path, error);
    return 1;
  }
  Ids ids{};
  if (!FindIds(model, ids)) {
    std::fprintf(stderr, "Required WBR model elements are missing.\n");
    mj_deleteModel(model);
    return 1;
  }
  const int rollout_steps =
      1;
  g_rollout_duration = rollout_steps * model->opt.timestep;

  std::printf("Constructing L0=%.3f m equilibrium configurations...\n",
              g_target_length);
  const double gravity_z = model->opt.gravity[2];
  model->opt.gravity[2] = 0.0;
  double paper_t_bias = 0.0;
  double paper_tp_bias = 0.0;
  FullState equilibrium = SettleConfiguration(model, ids, 0.0, &paper_tp_bias);
  FullState theta_plus =
      SettleConfiguration(model, ids, kStatePositionPerturbation, nullptr);
  FullState theta_minus =
      SettleConfiguration(model, ids, -kStatePositionPerturbation, nullptr);
  const double nominal_target_length = g_target_length;
  g_target_length = nominal_target_length + kStatePositionPerturbation;
  FullState length_plus = SettleConfiguration(model, ids, 0.0, nullptr);
  g_target_length = nominal_target_length - kStatePositionPerturbation;
  FullState length_minus = SettleConfiguration(model, ids, 0.0, nullptr);
  g_target_length = nominal_target_length;
  model->opt.gravity[2] = gravity_z;
  paper_tp_bias = 0.0;
  PlaceWheelsOnFloor(model, ids, equilibrium);
  PlaceWheelsOnFloor(model, ids, theta_plus);
  PlaceWheelsOnFloor(model, ids, theta_minus);
  PlaceWheelsOnFloor(model, ids, length_plus);
  PlaceWheelsOnFloor(model, ids, length_minus);
  const FullState geometric_equilibrium = equilibrium;
  g_support_force_per_leg =
      0.5 * mj_getTotalmass(model) * std::fabs(model->opt.gravity[2]);
  std::printf("Relaxing the landed closed chain with x/pitch fixture only...\n");
  RelaxInternalConfiguration(model, ids, equilibrium);
  const double relaxed_x_origin = WheelCenterX(model, ids, equilibrium);
  const State relaxed_state =
      MeasureInitialState(model, ids, equilibrium, relaxed_x_origin);
  mjData* relaxed_data = mj_makeData(model);
  mju_copy(relaxed_data->qpos, equilibrium.qpos.data(), model->nq);
  mju_zero(relaxed_data->qvel, model->nv);
  mj_forward(model, relaxed_data);
  const double relaxed_length = AverageLegLength(model, relaxed_data, ids);
  mj_deleteData(relaxed_data);
  std::printf("  relaxed pose: L=%.9g theta=% .9g phi=% .9g\n",
              relaxed_length, relaxed_state[0], relaxed_state[4]);
  if (std::fabs(relaxed_length - g_target_length) > 0.03 ||
      std::fabs(relaxed_state[0]) > 0.2 ||
      std::fabs(relaxed_state[4]) > 0.15) {
    std::fprintf(stderr, "Relaxation left the intended kinematic branch.\n");
    mj_deleteModel(model);
    return 3;
  }
  for (int q = 0; q < model->nq; ++q) {
    const double relaxed_offset =
        equilibrium.qpos[q] - geometric_equilibrium.qpos[q];
    theta_plus.qpos[q] += relaxed_offset;
    theta_minus.qpos[q] += relaxed_offset;
    length_plus.qpos[q] += relaxed_offset;
    length_minus.qpos[q] += relaxed_offset;
  }
  const double support_force_per_leg =
      0.5 * mj_getTotalmass(model) * std::fabs(model->opt.gravity[2]);
  g_support_force_per_leg = support_force_per_leg;
  std::printf("Using free-body support seed F0=mg/2=%.9g N per leg.\n",
              support_force_per_leg);
  double trim_z = 0.0;
  double trim_theta = 0.0;
  double trim_phi = 0.0;
  double trim_length = 0.0;
  std::printf("Solving the free landed dynamic trim...\n");
  if (!TrimDynamicEquilibrium(
          model, ids, equilibrium, theta_plus, theta_minus, length_plus,
          length_minus, paper_t_bias, paper_tp_bias, trim_z, trim_theta,
          trim_phi, trim_length)) {
    std::fprintf(stderr, "Could not solve the free landed dynamic trim.\n");
    mj_deleteModel(model);
    return 3;
  }
  std::printf("  trim solution: dz=% .9g dtheta=% .9g dphi=% .9g dL=% .9g\n",
              trim_z, trim_theta, trim_phi, trim_length);
  const double x_origin = WheelCenterX(model, ids, equilibrium);

  std::array<FullState, kStateDim> plus;
  std::array<FullState, kStateDim> minus;
  for (int state = 0; state < kStateDim; ++state) {
    plus[state] = equilibrium;
    minus[state] = equilibrium;
  }
  plus[0] = theta_plus;
  minus[0] = theta_minus;
  plus[2].qpos[ids.root_qpos] += kStatePositionPerturbation;
  minus[2].qpos[ids.root_qpos] -= kStatePositionPerturbation;

  plus[3].qvel[ids.root_dof] = kStateVelocityPerturbation;
  minus[3].qvel[ids.root_dof] = -kStateVelocityPerturbation;
  const double radius = model->geom_size[3 * ids.geom_wheel[0]];
  plus[3].qvel[model->jnt_dofadr[
      mj_name2id(model, mjOBJ_JOINT, "q_H_wheel")]] =
      kStateVelocityPerturbation / radius;
  plus[3].qvel[model->jnt_dofadr[
      mj_name2id(model, mjOBJ_JOINT, "q_H_wheel_2")]] =
      -kStateVelocityPerturbation / radius;
  minus[3].qvel[model->jnt_dofadr[
      mj_name2id(model, mjOBJ_JOINT, "q_H_wheel")]] =
      -kStateVelocityPerturbation / radius;
  minus[3].qvel[model->jnt_dofadr[
      mj_name2id(model, mjOBJ_JOINT, "q_H_wheel_2")]] =
      kStateVelocityPerturbation / radius;

  const double half_phi = 0.5 * kStatePositionPerturbation;
  plus[4].qpos[ids.root_qpos + 3] = std::cos(half_phi);
  plus[4].qpos[ids.root_qpos + 4] = 0.0;
  plus[4].qpos[ids.root_qpos + 5] = std::sin(half_phi);
  plus[4].qpos[ids.root_qpos + 6] = 0.0;
  minus[4].qpos[ids.root_qpos + 3] = std::cos(half_phi);
  minus[4].qpos[ids.root_qpos + 4] = 0.0;
  minus[4].qpos[ids.root_qpos + 5] = -std::sin(half_phi);
  minus[4].qpos[ids.root_qpos + 6] = 0.0;
  plus[5].qvel[ids.root_dof + 4] = kStateVelocityPerturbation;
  minus[5].qvel[ids.root_dof + 4] = -kStateVelocityPerturbation;

  for (int joint = 1; joint < model->njnt; ++joint) {
    if (model->jnt_type[joint] != mjJNT_HINGE &&
        model->jnt_type[joint] != mjJNT_SLIDE) {
      continue;
    }
    const int qpos_address = model->jnt_qposadr[joint];
    const int dof_address = model->jnt_dofadr[joint];
    const double tangent =
        (theta_plus.qpos[qpos_address] - theta_minus.qpos[qpos_address]) /
        (2.0 * kStatePositionPerturbation);
    plus[1].qvel[dof_address] = tangent * kStateVelocityPerturbation;
    minus[1].qvel[dof_address] = -tangent * kStateVelocityPerturbation;
  }

  Matrix6 initial_delta{};
  Matrix6 final_delta{};
  for (int column = 0; column < kStateDim; ++column) {
    const State initial_plus = MeasureInitialState(model, ids, plus[column], x_origin);
    const State initial_minus = MeasureInitialState(model, ids, minus[column], x_origin);
    const State final_plus =
        Rollout(model, ids, plus[column], 0.0, 0.0, paper_t_bias,
                paper_tp_bias, x_origin);
    const State final_minus =
        Rollout(model, ids, minus[column], 0.0, 0.0, paper_t_bias,
                paper_tp_bias, x_origin);
    for (int row = 0; row < kStateDim; ++row) {
      initial_delta[row][column] = 0.5 * (initial_plus[row] - initial_minus[row]);
      final_delta[row][column] = 0.5 * (final_plus[row] - final_minus[row]);
    }
  }

  Matrix6 inverse_initial{};
  if (!Invert6(initial_delta, inverse_initial)) {
    std::fprintf(stderr, "The six reduced-state perturbations are singular.\n");
    mj_deleteModel(model);
    return 2;
  }
  const Matrix6 ad = Multiply(final_delta, inverse_initial);
  Matrix6 a{};
  for (int row = 0; row < kStateDim; ++row) {
    for (int col = 0; col < kStateDim; ++col) {
      a[row][col] = (ad[row][col] - (row == col ? 1.0 : 0.0)) /
                    g_rollout_duration;
    }
  }

  Matrix62 bd{};
  for (int input = 0; input < kInputDim; ++input) {
    const double t_plus = input == 0 ? kInputPerturbation : 0.0;
    const double tp_plus = input == 1 ? kInputPerturbation : 0.0;
    const State output_plus =
        Rollout(model, ids, equilibrium, t_plus, tp_plus, paper_t_bias,
                paper_tp_bias, x_origin);
    const State output_minus =
        Rollout(model, ids, equilibrium, -t_plus, -tp_plus, paper_t_bias,
                paper_tp_bias, x_origin);
    for (int row = 0; row < kStateDim; ++row) {
      bd[row][input] = (output_plus[row] - output_minus[row]) /
                       (2.0 * kInputPerturbation);
    }
  }
  Matrix62 b{};
  for (int row = 0; row < kStateDim; ++row) {
    for (int input = 0; input < kInputDim; ++input) {
      b[row][input] = bd[row][input] / g_rollout_duration;
    }
  }

  const State equilibrium_state =
      MeasureInitialState(model, ids, equilibrium, x_origin);
  mjData* equilibrium_data = mj_makeData(model);
  mju_copy(equilibrium_data->qpos, equilibrium.qpos.data(), model->nq);
  mju_copy(equilibrium_data->qvel, equilibrium.qvel.data(), model->nv);
  mj_forward(model, equilibrium_data);
  const double equilibrium_length =
      AverageLegLength(model, equilibrium_data, ids);
  mj_deleteData(equilibrium_data);
  std::printf("Equilibrium reduced state [theta dtheta x dx phi dphi]:\n  ");
  for (double value : equilibrium_state) {
    std::printf("% .8g ", value);
  }
  std::printf("\nEquilibrium average leg length: %.9g m\n", equilibrium_length);
  std::printf("Identification horizon: %.6f s (%d simulation steps)\n",
              g_rollout_duration, rollout_steps);
  std::printf("Equilibrium paper input bias [T0 Tp0]: [%.9g %.9g] N.m\n",
              paper_t_bias, paper_tp_bias);
  std::printf("Calibrated support feedforward F0: %.9g N per leg\n",
              support_force_per_leg);
  PrintMatrix("A_identified", a);
  PrintMatrix("B_identified", b);
  const auto singular_values = ControllabilitySingularValues(a, b);
  const int rank = NumericalRank(singular_values);
  std::printf("\nNormalized controllability singular values:\n  ");
  for (double value : singular_values) {
    std::printf("%.9g ", value);
  }
  std::printf("\n");
  std::printf("\nrank(ctrb(A_identified, B_identified)) = %d\n", rank);
  std::printf("Controllability: %s\n", rank == kStateDim ? "FULL" : "NOT FULL");

  ValidateLinearModel(model, ids, equilibrium, plus, minus, ad, bd,
                      paper_t_bias, paper_tp_bias, x_origin, 0.5, 0.5,
                      "LOCAL");
  ValidateLinearModel(model, ids, equilibrium, plus, minus, ad, bd,
                      paper_t_bias, paper_tp_bias, x_origin, 1.5, 1.5,
                      "STRESS");
  const State nominal_output = Rollout(model, ids, equilibrium, 0.0, 0.0,
                                       paper_t_bias, paper_tp_bias, x_origin);
  std::printf("\nNominal one-step velocity drift [dtheta dx dphi]: "
              "[% .9g % .9g % .9g]\n",
              nominal_output[1], nominal_output[3], nominal_output[5]);
  const double identification_duration = g_rollout_duration;
  g_rollout_duration = 0.1;
  const State hold_output = Rollout(model, ids, equilibrium, 0.0, 0.0,
                                    paper_t_bias, paper_tp_bias, x_origin);
  g_rollout_duration = identification_duration;
  std::printf("Static-input 0.1 s hold state [theta dtheta x dx phi dphi]:\n  ");
  for (double value : hold_output) std::printf("% .8g ", value);
  std::printf("\n");
  const Gain26 identified_gain = SolveDiscreteLqr(ad, bd);
  std::printf("\nIdentified 1 ms discrete LQR K = [\n");
  for (int input = 0; input < kInputDim; ++input) {
    std::printf("  ");
    for (int state = 0; state < kStateDim; ++state) {
      std::printf("% .12g%s", identified_gain[input][state],
                  state + 1 == kStateDim ? "" : " ");
    }
    std::printf("%s\n", input + 1 == kInputDim ? "" : ";");
  }
  std::printf("];\n");
  State linear_state = {1e-3, 0.0, 1e-3, 0.0, 1e-3, 0.0};
  double max_linear_norm = 0.0;
  for (int step = 0; step < 10000; ++step) {
    State next{};
    for (int row = 0; row < kStateDim; ++row) {
      for (int col = 0; col < kStateDim; ++col) {
        double closed_loop = ad[row][col];
        for (int input = 0; input < kInputDim; ++input) {
          closed_loop -= bd[row][input] * identified_gain[input][col];
        }
        next[row] += closed_loop * linear_state[col];
      }
    }
    linear_state = next;
    double norm = 0.0;
    for (double value : linear_state) norm = std::hypot(norm, value);
    max_linear_norm = std::fmax(max_linear_norm, norm);
  }
  double final_linear_norm = 0.0;
  for (double value : linear_state) {
    final_linear_norm = std::hypot(final_linear_norm, value);
  }
  std::printf("Linear closed-loop 10 s: max norm=%.9g final norm=%.9g\n",
              max_linear_norm, final_linear_norm);
  RunNonlinearClosedLoopTest(model, ids, equilibrium, plus, minus,
                             paper_t_bias, paper_tp_bias, x_origin, bd,
                             identified_gain, "identified 1 ms discrete");

  mj_deleteModel(model);
  return 0;
}

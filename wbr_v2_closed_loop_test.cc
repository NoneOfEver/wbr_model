#include "simulate/wbr_controller_v2.h"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace {

bool g_yaw_enabled = true;
double g_orientation_perturbation = 0.01;
double g_target_leg_length = 0.18;
double g_test_yaw_rate = 0.20;
double g_test_yaw_duration = 2.0;
constexpr double kYawSettleDuration = 2.0;

double TotalMass(const mjModel* model) {
  double mass = 0.0;
  for (int body = 1; body < model->nbody; ++body) mass += model->body_mass[body];
  return mass;
}

double SubtreeMass(const mjModel* model, const char* root_name) {
  const int root = mj_name2id(model, mjOBJ_BODY, root_name);
  double mass = 0.0;
  for (int body = 1; body < model->nbody; ++body) {
    int ancestor = body;
    while (ancestor > 0 && ancestor != root) ancestor = model->body_parentid[ancestor];
    if (ancestor == root) mass += model->body_mass[body];
  }
  return mass;
}

double RootPitch(const mjData* data, int body_id) {
  const double* xmat = data->xmat + 9 * body_id;
  return std::atan2(xmat[6], std::hypot(xmat[0], xmat[3]));
}

double RootRoll(const mjData* data, int body_id) {
  const double* xmat = data->xmat + 9 * body_id;
  return std::atan2(xmat[7], xmat[8]);
}

double RootYaw(const mjData* data, int body_id) {
  const double* xmat = data->xmat + 9 * body_id;
  return std::atan2(xmat[3], xmat[0]);
}

void ApplyHorizontalFixture(const mjModel* model, mjData* data,
                            const double target_wheel_x[2],
                            const int wheel_body[2]) {
  const int root = mj_name2id(model, mjOBJ_JOINT, "root_free");
  const int q = model->jnt_qposadr[root];
  const int v = model->jnt_dofadr[root];
  data->qfrc_applied[v] = -1000.0 * data->qpos[q] - 100.0 * data->qvel[v];
  const int plate = mj_name2id(model, mjOBJ_BODY, "plate");
  if (model->jnt_type[root] == mjJNT_FREE) {
    data->qfrc_applied[v + 1] =
        -1000.0 * data->qpos[q + 1] - 100.0 * data->qvel[v + 1];
    data->qfrc_applied[v + 3] = -30.0 * data->qvel[v + 3];
    data->qfrc_applied[v + 4] =
        300.0 * RootPitch(data, plate) - 30.0 * data->qvel[v + 4];
    data->qfrc_applied[v + 5] = -30.0 * data->qvel[v + 5];
  } else {
    const int pitch = mj_name2id(model, mjOBJ_JOINT, "root_pitch");
    const int pitch_v = model->jnt_dofadr[pitch];
    data->qfrc_applied[pitch_v] =
        300.0 * RootPitch(data, plate) - 30.0 * data->qvel[pitch_v];
  }
  for (int leg = 0; leg < 2; ++leg) {
    mjtNum velocity[6];
    mj_objectVelocity(model, data, mjOBJ_BODY, wheel_body[leg], velocity, 0);
    const mjtNum force[3] = {
        -1000.0 * (data->xpos[3 * wheel_body[leg]] - target_wheel_x[leg]) -
            100.0 * velocity[3],
        0.0, 0.0};
    const mjtNum torque[3] = {0.0, 0.0, 0.0};
    mj_applyFT(model, data, force, torque,
               data->xpos + 3 * wheel_body[leg], wheel_body[leg],
               data->qfrc_applied);
  }
}

void RunLegVmcFreeCheck(const mjModel* model) {
  mjData* data = mj_makeData(model);
  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);
  WbrControllerV2 controller;
  controller.Reset(model);
  controller.SetYawEnabled(g_yaw_enabled);
  controller.SetLqrEnabled(false);
  double target_length = g_target_leg_length;
  double target_angle = 0.0;
  double max_t = 0.0;
  double max_tp = 0.0;
  double max_wheel_ctrl = 0.0;
  int joint_saturation_steps = 0;
  const int steps = static_cast<int>(std::lround(8.0 / model->opt.timestep));
  for (int step = 0; step < steps; ++step) {
    controller.Apply(model, data, target_length, target_angle);
    const auto& telemetry = controller.telemetry();
    max_t = std::max(max_t, std::abs(telemetry.applied_wheel_torque));
    max_tp = std::max(max_tp,
                      std::abs(telemetry.applied_leg_angle_torque));
    max_wheel_ctrl = std::max(
        max_wheel_ctrl,
        std::max(std::abs(data->ctrl[4]), std::abs(data->ctrl[5])));
    bool saturated = false;
    for (int actuator = 0; actuator < 4; ++actuator) {
      saturated = saturated || std::abs(data->ctrl[actuator]) >= 19.999;
    }
    joint_saturation_steps += saturated ? 1 : 0;
    mj_step(model, data);
  }
  const auto& telemetry = controller.telemetry();
  std::printf("\nLEG VMC FREE CHECK (balance feedback disabled)\n");
  std::printf("  final L1/L2       = %.9f / %.9f m\n",
              telemetry.leg_length[0], telemetry.leg_length[1]);
  std::printf("  max T/Tp          = %.9f / %.9f N.m\n", max_t, max_tp);
  std::printf("  max wheel ctrl    = %.9f N.m\n", max_wheel_ctrl);
  std::printf("  joint saturation  = %.2f %% of steps\n",
              100.0 * joint_saturation_steps / steps);
  std::printf("  final root z      = %.6f m\n",
              data->xpos[3 * mj_name2id(model, mjOBJ_BODY, "plate") + 2]);
  mj_deleteData(data);
}

void RunLegLengthCheck(const mjModel* model) {
  mjData* data = mj_makeData(model);
  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);
  const int wheel_body[2] = {
      mj_name2id(model, mjOBJ_BODY, "H_wheel_body"),
      mj_name2id(model, mjOBJ_BODY, "H_wheel_body_2")};
  const double target_wheel_x[2] = {
      data->xpos[3 * wheel_body[0]], data->xpos[3 * wheel_body[1]]};

  WbrControllerV2 controller;
  controller.Reset(model);
  controller.SetYawEnabled(g_yaw_enabled);
  controller.SetLqrEnabled(false);
  double target_length = g_target_leg_length;
  double target_angle = 0.0;
  double error_sum[2] = {};
  double max_rate[2] = {};
  double max_force[2] = {};
  int samples = 0;
  int joint_saturation_steps = 0;
  int total_steps = 0;
  int time_resets = 0;
  double previous_time = data->time;
  const int leg_check_steps =
      static_cast<int>(std::lround(8.0 / model->opt.timestep));
  for (int step = 0; step < leg_check_steps; ++step) {
    mju_zero(data->qfrc_applied, model->nv);
    controller.Apply(model, data, target_length, target_angle);
    ApplyHorizontalFixture(model, data, target_wheel_x, wheel_body);
    const auto& telemetry = controller.telemetry();
    if (step >= leg_check_steps -
                    static_cast<int>(std::lround(1.0 / model->opt.timestep))) {
      for (int leg = 0; leg < 2; ++leg) {
        error_sum[leg] += telemetry.leg_length[leg] - target_length;
        max_rate[leg] = std::max(max_rate[leg],
                                 std::abs(telemetry.leg_length_rate[leg]));
        max_force[leg] = std::max(max_force[leg],
                                  std::abs(telemetry.axial_force[leg]));
      }
      ++samples;
    }
    bool saturated = false;
    for (int actuator = 0; actuator < 4; ++actuator) {
      saturated = saturated || std::abs(data->ctrl[actuator]) >= 19.999;
    }
    joint_saturation_steps += saturated ? 1 : 0;
    ++total_steps;
    mj_step(model, data);
    time_resets += data->time < previous_time ? 1 : 0;
    previous_time = data->time;
  }
  const auto& telemetry = controller.telemetry();
  std::printf("\nLEG-LENGTH CHECK (LQR off, z unrestrained)\n");
  std::printf("  final L1/L2       = %.9f / %.9f m\n",
              telemetry.leg_length[0], telemetry.leg_length[1]);
  std::printf("  mean error L1/L2  = %+.9f / %+.9f m (last 1 s)\n",
              error_sum[0] / samples, error_sum[1] / samples);
  std::printf("  max |dL| L1/L2    = %.6f / %.6f m/s (last 1 s)\n",
              max_rate[0], max_rate[1]);
  std::printf("  max |F| L1/L2     = %.6f / %.6f N (last 1 s)\n",
              max_force[0], max_force[1]);
  std::printf("  final integral F  = %.6f / %.6f N\n",
              telemetry.integral_force[0], telemetry.integral_force[1]);
  std::printf("  joint saturation  = %.2f %% of steps\n",
              100.0 * joint_saturation_steps / total_steps);
  std::printf("  final root z      = %.6f m\n",
              data->xpos[3 * mj_name2id(model, mjOBJ_BODY, "plate") + 2]);
  std::printf("  numerical resets  = %d\n", time_resets);
  mj_deleteData(data);
}

const char* ContactSafetyStateName(WbrContactSafetyState state) {
  switch (state) {
    case WbrContactSafetyState::kDualSupport:
      return "dual";
    case WbrContactSafetyState::kSingleSupportFirst:
      return "single-first";
    case WbrContactSafetyState::kSingleSupportSecond:
      return "single-second";
    case WbrContactSafetyState::kAirborne:
      return "airborne";
    case WbrContactSafetyState::kRecovery:
      return "recovery";
  }
  return "unknown";
}

void RunSingleWheelLiftSafetyCheck(const mjModel* model) {
  mjData* data = mj_makeData(model);
  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);
  WbrControllerV2 controller;
  controller.Reset(model);
  controller.SetYawEnabled(true);
  double target_length = g_target_leg_length;
  double target_angle = 0.0;
  const int lifted_wheel = mj_name2id(
      model, mjOBJ_BODY, "H_wheel_body_2");
  const int wheel_actuator[2] = {
      mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel"),
      mj_name2id(model, mjOBJ_ACTUATOR, "act_H_wheel_2")};
  bool saw_single_support = false;
  bool saw_recovery = false;
  bool returned_to_dual_support = false;
  double max_airborne_wheel_torque = 0.0;
  double max_single_support_yaw_torque = 0.0;
  double minimum_single_support_authority = 1.0;
  WbrContactSafetyState previous_state = WbrContactSafetyState::kAirborne;
  const int steps = static_cast<int>(std::lround(3.0 / model->opt.timestep));
  for (int step = 0; step < steps; ++step) {
    mju_zero(data->qfrc_applied, model->nv);
    controller.SetVelocityCommand(0.0, data->time < 2.05 ? 0.20 : 0.0);
    controller.Apply(model, data, target_length, target_angle);
    const auto& telemetry = controller.telemetry();
    if (telemetry.contact_safety_state != previous_state) {
      std::printf("  safety transition t=%.3f: %s -> %s, contact=%d%d, "
                  "pitch=%+.3f roll=%+.3f\n",
                  data->time, ContactSafetyStateName(previous_state),
                  ContactSafetyStateName(telemetry.contact_safety_state),
                  telemetry.wheel_grounded[0] ? 1 : 0,
                  telemetry.wheel_grounded[1] ? 1 : 0,
                  RootPitch(data, mj_name2id(model, mjOBJ_BODY, "plate")),
                  RootRoll(data, mj_name2id(model, mjOBJ_BODY, "plate")));
      previous_state = telemetry.contact_safety_state;
    }
    const bool single_support =
        telemetry.contact_safety_state ==
            WbrContactSafetyState::kSingleSupportFirst ||
        telemetry.contact_safety_state ==
            WbrContactSafetyState::kSingleSupportSecond;
    if (single_support) {
      saw_single_support = true;
      const bool state_matches_raw_contact =
          (telemetry.contact_safety_state ==
               WbrContactSafetyState::kSingleSupportFirst &&
           telemetry.wheel_grounded[0] && !telemetry.wheel_grounded[1]) ||
          (telemetry.contact_safety_state ==
               WbrContactSafetyState::kSingleSupportSecond &&
           !telemetry.wheel_grounded[0] && telemetry.wheel_grounded[1]);
      if (state_matches_raw_contact) {
        const int airborne_leg =
            telemetry.contact_safety_state ==
                    WbrContactSafetyState::kSingleSupportFirst
                ? 1
                : 0;
        max_airborne_wheel_torque = std::max(
            max_airborne_wheel_torque,
            std::abs(data->ctrl[wheel_actuator[airborne_leg]]));
      }
      max_single_support_yaw_torque = std::max(
          max_single_support_yaw_torque,
          std::abs(telemetry.applied_yaw_torque));
      minimum_single_support_authority = std::min(
          minimum_single_support_authority,
          telemetry.contact_authority_scale);
    }
    saw_recovery = saw_recovery ||
        telemetry.contact_safety_state == WbrContactSafetyState::kRecovery;
    returned_to_dual_support = returned_to_dual_support ||
        (saw_single_support &&
         telemetry.contact_safety_state == WbrContactSafetyState::kDualSupport);

    if (data->time >= 2.0 && data->time < 2.080) {
      const mjtNum force[3] = {0.0, 0.0, 39.0};
      const mjtNum torque[3] = {0.0, 0.0, 0.0};
      mj_applyFT(model, data, force, torque,
                 data->xpos + 3 * lifted_wheel, lifted_wheel,
                 data->qfrc_applied);
    }
    mj_step(model, data);
  }
  const auto& final_telemetry = controller.telemetry();
  const int plate = mj_name2id(model, mjOBJ_BODY, "plate");
  std::printf("\nSINGLE-WHEEL-LIFT SAFETY CHECK\n");
  std::printf("  saw single/recovery = %d / %d\n",
              saw_single_support ? 1 : 0, saw_recovery ? 1 : 0);
  std::printf("  returned dual       = %d\n",
              returned_to_dual_support ? 1 : 0);
  std::printf("  airborne wheel ctrl = %.6f N.m\n",
              max_airborne_wheel_torque);
  std::printf("  single yaw torque   = %.6f N.m\n",
              max_single_support_yaw_torque);
  std::printf("  single authority    = %.3f\n",
              minimum_single_support_authority);
  std::printf("  final state/contact = %s / %d%d\n",
              ContactSafetyStateName(final_telemetry.contact_safety_state),
              final_telemetry.wheel_grounded[0] ? 1 : 0,
              final_telemetry.wheel_grounded[1] ? 1 : 0);
  std::printf("  final z/roll/pitch  = %.6f / %.6f / %.6f\n",
              data->xpos[3 * plate + 2], RootRoll(data, plate),
              RootPitch(data, plate));
  mj_deleteData(data);
}

bool RunTorqueMarginCheck(const mjModel* model, int perturbation_mode) {
  mjData* data = mj_makeData(model);
  mj_resetDataKeyframe(model, data, 0);
  const int root = mj_name2id(model, mjOBJ_JOINT, "root_free");
  const int root_qpos = model->jnt_qposadr[root];
  mj_forward(model, data);

  WbrControllerV2 controller;
  controller.Reset(model);
  controller.SetYawEnabled(g_yaw_enabled);
  controller.SetLqrEnabled(false);
  double target_length = g_target_leg_length;
  double target_angle = 0.0;
  const int wheel_body[2] = {
      mj_name2id(model, mjOBJ_BODY, "H_wheel_body"),
      mj_name2id(model, mjOBJ_BODY, "H_wheel_body_2")};
  const double target_wheel_x[2] = {
      data->xpos[3 * wheel_body[0]], data->xpos[3 * wheel_body[1]]};
  if (perturbation_mode == 6) {
    // Match simulate/main.cc: synchronize the UI targets from the keyframe
    // state and enable balance without the test-only horizontal fixture.
    controller.SyncTargetsFromState(model, data, target_length, target_angle);
    controller.SetLqrEnabled(true);
  } else {
    const int settle_steps =
        static_cast<int>(std::lround(8.0 / model->opt.timestep));
    for (int step = 0; step < settle_steps; ++step) {
      mju_zero(data->qfrc_applied, model->nv);
      controller.Apply(model, data, target_length, target_angle);
      ApplyHorizontalFixture(model, data, target_wheel_x, wheel_body);
      mj_step(model, data);
    }
  }

  const double perturbation = g_orientation_perturbation;
  if (model->jnt_type[root] == mjJNT_FREE) {
    const mjtNum pitch_quat[4] = {
        std::cos(0.5 * perturbation), 0.0, -std::sin(0.5 * perturbation), 0.0};
    if (perturbation_mode == 1) {
      const mjtNum roll_quat[4] = {
          std::cos(0.5 * perturbation), std::sin(0.5 * perturbation), 0.0, 0.0};
      mjtNum combined_quat[4];
      mju_mulQuat(combined_quat, roll_quat, pitch_quat);
      mju_copy(data->qpos + root_qpos + 3, combined_quat, 4);
    } else if (perturbation_mode == 2) {
      const mjtNum yaw_quat[4] = {
          std::cos(0.5 * perturbation), 0.0, 0.0,
          std::sin(0.5 * perturbation)};
      mjtNum combined_quat[4];
      mju_mulQuat(combined_quat, yaw_quat, pitch_quat);
      mju_copy(data->qpos + root_qpos + 3, combined_quat, 4);
    } else if (perturbation_mode == 0) {
      mju_copy(data->qpos + root_qpos + 3, pitch_quat, 4);
    }
  } else if (perturbation_mode == 0) {
    const int pitch = mj_name2id(model, mjOBJ_JOINT, "root_pitch");
    data->qpos[model->jnt_qposadr[pitch]] = -perturbation;
  }
  mju_zero(data->qvel, model->nv);
  mju_zero(data->qfrc_applied, model->nv);
  mj_forward(model, data);
  controller.SetLqrEnabled(true);
  controller.ResetLqrReference();
  const double test_start_time = data->time;
  const int plate = mj_name2id(model, mjOBJ_BODY, "plate");
  double initial_plate_x = data->xpos[3 * plate];
  double initial_plate_y = data->xpos[3 * plate + 1];
  double max_planar_displacement = 0.0;
  double previous_yaw = RootYaw(data, plate);
  double accumulated_yaw = 0.0;
  double yaw_at_command_start = 0.0;
  double yaw_at_command_end = 0.0;
  double commanded_yaw_integral = 0.0;
  double max_requested_t = 0.0;
  double max_requested_tp = 0.0;
  double early_max_requested_t = 0.0;
  double early_max_requested_tp = 0.0;
  double first_requested_t = 0.0;
  double first_requested_tp = 0.0;
  double max_spin_mode_blend = 0.0;
  double max_yaw_reserve_per_wheel = 0.0;
  double min_balance_torque_authority = 1.0;
  int wheel_limited_steps = 0;
  int hip_limited_steps = 0;
  int joint_saturation_steps = 0;
  int steps = 0;
  std::printf("\nGROUND-BALANCE TRACE\n");
  std::printf("  time    theta   dtheta        x       dx      phi     dphi "
              "    roll     yaw       y    int1    int2      F1      F2    T_req   Tp_req "
              "  yaw_e  yaw_rate  yr_err ycoord auth att ctc tqm spl splr spla Fn1 Fn2 margin split_e split_dr  Tsplit "
              " dv_ref      dv   Tyaw  vxhat  vodo conf contact active\n");
  const int torque_check_steps =
      static_cast<int>(std::lround(
          (perturbation_mode == 1 ? 0.5 :
           perturbation_mode == 6
               ? kYawSettleDuration + g_test_yaw_duration + 4.0 :
           perturbation_mode >= 5 ? 6.0 :
           perturbation_mode >= 2 ? 5.0 : 20.0) / model->opt.timestep));
  const int yaw_command_steps = static_cast<int>(std::lround(
      g_test_yaw_duration / model->opt.timestep));
  const int yaw_settle_steps = static_cast<int>(std::lround(
      kYawSettleDuration / model->opt.timestep));
  for (int step = 0; step < torque_check_steps &&
                     data->xpos[3 * plate + 2] > 0.08;
       ++step) {
    mju_zero(data->qfrc_applied, model->nv);
    if (perturbation_mode == 5) {
      controller.SetVelocityCommand(
          step < static_cast<int>(std::lround(2.0 / model->opt.timestep))
              ? 0.35 : 0.0,
          0.0);
    } else if (perturbation_mode == 6) {
      if (step == yaw_settle_steps) {
        yaw_at_command_start = accumulated_yaw;
        initial_plate_x = data->xpos[3 * plate];
        initial_plate_y = data->xpos[3 * plate + 1];
        max_planar_displacement = 0.0;
      }
      controller.SetVelocityCommand(
          0.0,
          step >= yaw_settle_steps &&
                  step < yaw_settle_steps + yaw_command_steps
              ? g_test_yaw_rate : 0.0);
    }
    controller.Apply(model, data, target_length, target_angle);
    if (perturbation_mode == 6 && step >= yaw_settle_steps &&
        step < yaw_settle_steps + yaw_command_steps) {
      commanded_yaw_integral +=
          controller.telemetry().commanded_yaw_rate * model->opt.timestep;
    }
    if ((perturbation_mode == 3 || perturbation_mode == 4) &&
        step < static_cast<int>(std::lround(0.05 / model->opt.timestep))) {
      const mjtNum force[3] = {
          perturbation_mode == 3 ? 20.0 : 0.0,
          perturbation_mode == 4 ? 20.0 : 0.0,
          0.0};
      const mjtNum torque[3] = {0.0, 0.0, 0.0};
      mj_applyFT(model, data, force, torque, data->xpos + 3 * plate,
                 plate, data->qfrc_applied);
    }
    const auto& telemetry = controller.telemetry();
    const int trace_stride =
        std::max(1, static_cast<int>(std::lround(0.25 / model->opt.timestep)));
    if (step == 0 || (step + 1) % trace_stride == 0 ||
        step + 1 == torque_check_steps) {
      const double* e = telemetry.state_error;
      std::printf("  %5.3f %+.5f %+.5f %+.5f %+.5f %+.5f %+.5f %+.5f "
                  "%+.5f %+.5f "
                  "%+7.2f %+7.2f %+7.2f %+7.2f %+8.3f %+8.3f "
                  "%+.5f %+.5f %+.5f %+.5f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
                  "%4.1f %4.1f %4.2f %+.5f %+.5f %+7.3f "
                  "%+.4f %+.4f %+7.3f %+.4f %+.4f %.2f    %d%d      %d\n",
                  data->time - test_start_time, e[0], e[1], e[2], e[3],
                  e[4], e[5], RootRoll(data, plate), RootYaw(data, plate),
                  data->xpos[3 * plate + 1],
                  telemetry.integral_force[0], telemetry.integral_force[1],
                  telemetry.axial_force[0], telemetry.axial_force[1],
                  telemetry.requested_wheel_torque,
                  telemetry.requested_leg_angle_torque,
                  telemetry.yaw_error, telemetry.yaw_rate,
                  telemetry.yaw_rate_error, telemetry.coordinated_yaw_rate,
                  telemetry.yaw_authority_scale,
                  telemetry.yaw_attitude_authority,
                  telemetry.yaw_contact_authority,
                  telemetry.yaw_torque_authority,
                  telemetry.yaw_split_authority,
                  telemetry.yaw_split_residual_authority,
                  telemetry.yaw_split_absolute_authority,
                  telemetry.wheel_normal_force[0],
                  telemetry.wheel_normal_force[1],
                  telemetry.wheel_torque_margin,
                  telemetry.differential_leg_angle_error,
                  telemetry.differential_leg_angle_rate,
                  telemetry.differential_leg_angle_torque,
                  telemetry.target_wheel_speed_difference,
                  telemetry.wheel_speed_difference,
                  telemetry.applied_yaw_torque,
                  telemetry.estimated_x_speed,
                  telemetry.wheel_odometry_x_speed,
                  telemetry.wheel_odometry_confidence,
                  telemetry.wheel_grounded[0] ? 1 : 0,
                  telemetry.wheel_grounded[1] ? 1 : 0,
                  telemetry.balance_active ? 1 : 0);
    }
    if (step == 0) {
      first_requested_t = telemetry.requested_wheel_torque;
      first_requested_tp = telemetry.requested_leg_angle_torque;
    }
    max_requested_t = std::max(max_requested_t,
                               std::abs(telemetry.requested_wheel_torque));
    max_requested_tp = std::max(max_requested_tp,
                                std::abs(telemetry.requested_leg_angle_torque));
    max_spin_mode_blend = std::max(max_spin_mode_blend,
                                   telemetry.spin_mode_blend);
    max_yaw_reserve_per_wheel = std::max(
        max_yaw_reserve_per_wheel,
        telemetry.reserved_yaw_torque_per_wheel);
    min_balance_torque_authority = std::min(
        min_balance_torque_authority, telemetry.balance_torque_authority);
    if (step < static_cast<int>(std::lround(0.05 / model->opt.timestep))) {
      early_max_requested_t =
          std::max(early_max_requested_t,
                   std::abs(telemetry.requested_wheel_torque));
      early_max_requested_tp =
          std::max(early_max_requested_tp,
                   std::abs(telemetry.requested_leg_angle_torque));
    }
    wheel_limited_steps +=
        std::abs(telemetry.requested_wheel_torque) > 10.0 ? 1 : 0;
    hip_limited_steps +=
        std::abs(telemetry.requested_leg_angle_torque) > 20.0 ? 1 : 0;
    bool joint_saturated = false;
    for (int actuator = 0; actuator < 4; ++actuator) {
      joint_saturated = joint_saturated ||
                        std::abs(data->ctrl[actuator]) >= 19.999;
    }
    joint_saturation_steps += joint_saturated ? 1 : 0;
    ++steps;
    mj_step(model, data);
    max_planar_displacement = std::max(
        max_planar_displacement,
        std::hypot(data->xpos[3 * plate] - initial_plate_x,
                   data->xpos[3 * plate + 1] - initial_plate_y));
    const double current_yaw = RootYaw(data, plate);
    accumulated_yaw +=
        std::remainder(current_yaw - previous_yaw, 2.0 * M_PI);
    previous_yaw = current_yaw;
    if (perturbation_mode == 6 &&
        step + 1 == yaw_settle_steps + yaw_command_steps) {
      yaw_at_command_end = accumulated_yaw;
    }
  }
  const char* perturbation_name =
      perturbation_mode == 1 ? "pitch/roll" :
      perturbation_mode == 2 ? "pitch/yaw" :
      perturbation_mode == 3 ? "20 N x push for 50 ms" :
      perturbation_mode == 4 ? "20 N y push for 50 ms" : "pitch";
  if (perturbation_mode == 5) {
    std::printf("\nVELOCITY-COMMAND CHECK (0.35 m/s for 2 s, then stop)\n");
  } else if (perturbation_mode == 6) {
    std::printf("\nYAW-RATE-COMMAND CHECK (%.2f rad/s for %.1f s, then stop)\n",
                g_test_yaw_rate, g_test_yaw_duration);
  } else if (perturbation_mode >= 3) {
    std::printf("\nPUSH-RECOVERY CHECK (%s)\n", perturbation_name);
  } else {
    std::printf("\nTORQUE-MARGIN CHECK (%.3f rad %s perturbation)\n",
                perturbation, perturbation_name);
  }
  std::printf("  simulated time    = %.3f s\n", data->time - test_start_time);
  std::printf("  peak requested T  = %.6f N.m (limit 10 total)\n", max_requested_t);
  std::printf("  peak requested Tp = %.6f N.m (limit 20 total)\n", max_requested_tp);
  std::printf("  first 50 ms T/Tp  = %.6f / %.6f N.m\n",
              early_max_requested_t, early_max_requested_tp);
  std::printf("  first cycle T/Tp  = %+.6f / %+.6f N.m\n",
              first_requested_t, first_requested_tp);
  std::printf("  T/Tp limited      = %.2f %% / %.2f %% of steps\n",
              100.0 * wheel_limited_steps / steps,
              100.0 * hip_limited_steps / steps);
  std::printf("  joint saturation  = %.2f %% of steps\n",
              100.0 * joint_saturation_steps / steps);
  std::printf("  final z/pitch     = %.6f m / %.6f rad\n",
              data->xpos[3 * plate + 2], RootPitch(data, plate));
  const auto& final_telemetry = controller.telemetry();
  std::printf("  final roll/yaw    = %.6f / %.6f rad\n",
              RootRoll(data, plate), RootYaw(data, plate));
  std::printf("  max planar drift  = %.6f m\n", max_planar_displacement);
  std::printf("  final contact     = %d%d (balance active=%d)\n",
              final_telemetry.wheel_grounded[0] ? 1 : 0,
              final_telemetry.wheel_grounded[1] ? 1 : 0,
              final_telemetry.balance_active ? 1 : 0);
  mjtNum final_wheel_velocity_1[6];
  mjtNum final_wheel_velocity_2[6];
  mj_objectVelocity(model, data, mjOBJ_BODY, wheel_body[0],
                    final_wheel_velocity_1, 0);
  mj_objectVelocity(model, data, mjOBJ_BODY, wheel_body[1],
                    final_wheel_velocity_2, 0);
  std::printf("  final wheel x/dx  = %.6f / %.6f m, m/s\n",
              0.5 * (data->xpos[3 * wheel_body[0]] +
                     data->xpos[3 * wheel_body[1]]),
              0.5 * (final_wheel_velocity_1[3] +
                     final_wheel_velocity_2[3]));
  bool passed = true;
  if (perturbation_mode == 6) {
    constexpr double kMinimumYawTrackingRatio = 0.24;
    const double requested_yaw_change = commanded_yaw_integral;
    const double tracking_ratio = std::abs(requested_yaw_change) > 1e-9
        ? (yaw_at_command_end - yaw_at_command_start) /
              requested_yaw_change
        : 1.0;
    const bool direction_correct =
        requested_yaw_change == 0.0 ||
        (yaw_at_command_end - yaw_at_command_start) *
            requested_yaw_change > 0.0;
    const bool remained_upright =
        data->xpos[3 * plate + 2] > 0.15 &&
        std::fabs(RootRoll(data, plate)) < 0.20 &&
        std::fabs(RootPitch(data, plate)) < 0.35 &&
        final_telemetry.balance_active;
    const bool remained_in_place = max_planar_displacement < 0.15;
    passed = direction_correct &&
             tracking_ratio >= kMinimumYawTrackingRatio &&
             remained_upright && remained_in_place;
    std::printf("  commanded yaw int = %+.6f rad\n", requested_yaw_change);
    std::printf("  command yaw       = %+.6f rad\n",
                yaw_at_command_end - yaw_at_command_start);
    std::printf("  post-stop yaw     = %+.6f rad\n",
                accumulated_yaw - yaw_at_command_end);
    std::printf("  final yaw change  = %+.6f rad\n", accumulated_yaw);
    std::printf("  spin/reserve      = %.2f / %.3f N.m per wheel\n",
                max_spin_mode_blend, max_yaw_reserve_per_wheel);
    std::printf("  min balance auth  = %.3f\n",
                min_balance_torque_authority);
    std::printf("  yaw tracking      = %.1f %% (minimum %.1f %%)\n",
                100.0 * tracking_ratio,
                100.0 * kMinimumYawTrackingRatio);
    std::printf("  remained upright  = %s\n",
                remained_upright ? "yes" : "no");
    std::printf("  remained in place = %s\n",
                remained_in_place ? "yes" : "no");
    std::printf("  yaw result        = %s\n", passed ? "PASS" : "FAIL");
  }
  mj_deleteData(data);
  return passed;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string path = argc > 1 ? argv[1] : "wbr_free.xml";
  char error[1024] = {};
  mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Failed to load %s: %s\n", path.c_str(), error);
    return 1;
  }
  if (argc > 2 && std::string(argv[2]) == "--yaw") {
    g_yaw_enabled = true;
    if (argc > 4) {
      const double sliding_friction_1 = std::atof(argv[3]);
      const double sliding_friction_2 = std::atof(argv[4]);
      const char* pair_names[2] = {"wheel_floor_1", "wheel_floor_2"};
      for (const char* pair_name : pair_names) {
        const int pair = mj_name2id(model, mjOBJ_PAIR, pair_name);
        if (pair < 0) {
          std::fprintf(stderr, "Missing contact pair %s.\n", pair_name);
          mj_deleteModel(model);
          return 1;
        }
        model->pair_friction[5 * pair] = sliding_friction_1;
        model->pair_friction[5 * pair + 1] = sliding_friction_2;
      }
      std::printf("CONTACT OVERRIDE sliding1=%.3f sliding2=%.3f\n",
                  sliding_friction_1, sliding_friction_2);
      if (argc > 5) {
        g_orientation_perturbation = std::atof(argv[5]);
      }
      if (argc > 6) {
        g_target_leg_length = std::atof(argv[6]);
      }
      if (argc > 7) {
        g_test_yaw_rate = std::atof(argv[7]);
      }
      if (argc > 8) {
        g_test_yaw_duration = std::atof(argv[8]);
      }
      if (argc > 9) {
        model->opt.timestep = std::atof(argv[9]);
        std::printf("TIMESTEP OVERRIDE %.6f s\n", model->opt.timestep);
      }
    } else if (argc > 3) {
      g_target_leg_length = std::atof(argv[3]);
    }
  } else if (argc > 2) {
    g_target_leg_length = std::atof(argv[2]);
  }
  std::printf("TARGET LEG LENGTH %.3f m\n", g_target_leg_length);
  std::printf("MASS CHECK\n");
  std::printf("  total             = %.9f kg\n", TotalMass(model));
  std::printf("  A/A_2 subtrees    = %.9f / %.9f kg\n",
              SubtreeMass(model, "A"), SubtreeMass(model, "A_2"));
  std::printf("  wheel bodies      = %.9f / %.9f kg\n",
              model->body_mass[mj_name2id(model, mjOBJ_BODY, "H_wheel_body")],
              model->body_mass[mj_name2id(model, mjOBJ_BODY,
                                          "H_wheel_body_2")]);
  RunLegVmcFreeCheck(model);
  RunLegLengthCheck(model);
  RunSingleWheelLiftSafetyCheck(model);
  bool passed = RunTorqueMarginCheck(model, 0);
  const int root = mj_name2id(model, mjOBJ_JOINT, "root_free");
  if (root >= 0 && model->jnt_type[root] == mjJNT_FREE) {
    passed = RunTorqueMarginCheck(model, 1) && passed;
    passed = RunTorqueMarginCheck(model, 2) && passed;
    passed = RunTorqueMarginCheck(model, 3) && passed;
    passed = RunTorqueMarginCheck(model, 4) && passed;
    passed = RunTorqueMarginCheck(model, 5) && passed;
    passed = RunTorqueMarginCheck(model, 6) && passed;
  }
  mj_deleteModel(model);
  return passed ? 0 : 1;
}

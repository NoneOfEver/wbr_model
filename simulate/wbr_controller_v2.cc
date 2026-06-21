#include "wbr_controller_v2.h"

#include <cmath>
#include <cstdio>

namespace {
constexpr double kLengthAB = 0.0945;
constexpr double kLengthBC = 0.1125;
constexpr double kLengthCD = 0.116;
constexpr double kLengthAD = 0.090;
constexpr double kLengthAG = 0.210;
constexpr double kLengthGH = 0.250;
constexpr double kTargetHRadius = 0.4;
constexpr double kJacobianStep = 1e-6;
constexpr double kMinLegLength = 1e-4;
constexpr double kMinTargetLegLength = 0.15;
constexpr double kMaxTargetLegAngle = 0.6;

// Leg-length controller parameters. Tune these before enabling LQR body control.
constexpr double kLegLengthKp = 300.0;
constexpr double kLegLengthKi = 400.0;
constexpr double kLegLengthKd = 20.0;
constexpr double kLegIntegralForceLimit = 60.0;
constexpr double kLegForceLimit = 150.0;
constexpr double kJointTorqueLimit = 20.0;
constexpr double kLegSpeedFilter = 0.2;
constexpr double kStateSpeedFilter = 0.2;
constexpr double kTargetLengthSlewRate = 0.15;
constexpr double kTargetAngleSlewRate = 0.5;
constexpr double kSupportFilterTimeConstant = 0.05;
constexpr double kTotalWheelTorqueLimit = 10.0;
constexpr double kTotalLegAngleTorqueLimit = 20.0;
// Identified on wbr_free.xml: ddroll / differential axial force ~= 0.24
// rad/s^2/N. These gains place the roll pair near wn=4 rad/s, zeta=0.9.
constexpr double kRollForceKp = 67.0;
constexpr double kRollForceKd = 30.0;
constexpr double kRollDifferentialForceLimit = 40.0;
// Article-style decoupled yaw-rate PD. Differential wheel torque is reduced
// when the anti-split leg controller approaches its safe envelope.
constexpr double kYawRateKp = 4.0;
constexpr double kYawAccelerationKd = 0.05;
// Identified 2x2 input matrix on wbr_free.xml:
// [ddyaw, ddsplit]' = [[0.102976, 0.659783],
//                      [103.9650, 18.282555]] * [Uwheel, Tsplit]'.
// Its inverse maps a yaw request to a zero-split input pair.  The 20 ms
// open-loop probe overestimates the closed-loop working-point authority, so a
// common identified gain restores useful yaw response without changing the
// decoupling ratio.
constexpr double kDecoupledYawWorkingPointGain = 30.0;
constexpr double kDecoupledYawWheelInputScale =
    0.028222 * kDecoupledYawWorkingPointGain;
constexpr double kDecoupledYawLegTorquePerCommand =
    0.160481 * kDecoupledYawWorkingPointGain;
// Open-loop identification on wbr_free.xml gives approximately
// ddyaw / Tyaw = 0.103 rad/s^2/N.m.  A 4 N.m limit provides about
// 0.41 rad/s^2 wheel-only yaw acceleration and remains below the 5 N.m
// per-wheel actuator limit.  Command shaping may request a faster ramp, while
// the feedback loop and authority gates enforce the realizable response.
constexpr double kTotalYawTorqueLimit = 4.0;
constexpr double kYawTorqueRiseRate = 1.5;
constexpr double kYawTorqueBrakeRate = 20.0;
constexpr double kPerWheelTorqueLimit = 5.0;
constexpr double kYawRateFilterTimeConstant = 0.02;
constexpr double kYawAccelerationFilterTimeConstant = 0.04;
constexpr double kChassisVelocityCorrectionTimeConstant = 0.02;
constexpr double kContactGraceTime = 0.08;
constexpr double kContactLossDebounceTime = 0.050;
constexpr double kContactRecoveryDebounceTime = 0.020;
constexpr double kContactRecoveryRampTime = 0.15;
constexpr double kSingleSupportLqrScale = 0.0;
constexpr double kRecoveryMaxRoll = 0.12;
constexpr double kRecoveryMaxPitch = 0.25;
constexpr double kAirborneLegSearchExtension = 0.025;
constexpr double kGroundedLegYield = 0.010;
constexpr double kContactLegOffsetSlewRate = 0.08;
constexpr double kEmergencyLinearDeceleration = 1.5;
constexpr double kMaxLinearVelocity = 0.6;
// Match the SPR spin-mode command envelope (5-13 rad/s).  The keyboard uses
// the low end of this range while higher-level commands may request up to 13.
constexpr double kMaxYawRate = 13.0;
constexpr double kSustainedYawRateLimit = 5.0;
constexpr double kLinearAccelerationLimit = 0.6;
constexpr double kYawCommandAcceleration = 1.0;
constexpr double kYawCommandDeceleration = 3.0;
constexpr double kMaxPositionTrackingError = 0.5;
constexpr double kDifferentialLegAngleKp = 16.0;
constexpr double kDifferentialLegAngleKd = 1.5;
constexpr double kDifferentialLegAngleTorqueLimit = 20.0;
// Must track the decoupled yaw feedforward during the 20 N.m/s emergency yaw
// brake: 4.81443 * 20 = 96.3 N.m/s.  Add margin for feedback correction.
constexpr double kDifferentialLegAngleTorqueSlewRate = 120.0;
// Positive yaw commands produce a repeatable negative leg-angle split in the
// free-root model.  Treat this bounded, command-correlated split as the normal
// turning manifold instead of fighting it as an instability.
constexpr double kYawSplitReferencePerRate = -0.60;
constexpr double kMaxYawSplitReference = 0.14;
// A 1 rad/s^2 yaw command ramp corresponds to 0.6 rad/s of expected
// split-reference motion before the reference reaches its bounded envelope.
constexpr double kYawSplitReferenceSlewRate = 0.6;
constexpr double kLegSplitSoftAngle = 0.08;
constexpr double kLegSplitHardAngle = 0.25;
constexpr double kLegSplitSoftRate = 0.25;
constexpr double kLegSplitHardRate = 1.0;
// Residual-based authority is primary; these wider absolute bounds remain as
// an emergency guard if the commanded reference and mechanism diverge.
constexpr double kAbsoluteLegSplitSoftAngle = 0.24;
constexpr double kAbsoluteLegSplitHardAngle = 0.35;
constexpr double kAbsoluteLegSplitSoftRate = 1.0;
constexpr double kAbsoluteLegSplitHardRate = 2.5;
constexpr double kYawRollSoftLimit = 0.08;
constexpr double kYawRollHardLimit = 0.20;
constexpr double kYawPitchSoftLimit = 0.12;
constexpr double kYawPitchHardLimit = 0.35;
constexpr double kYawContactForceHard = 3.0;
constexpr double kYawContactForceSoft = 15.0;
constexpr double kYawLoadRatioHard = 0.15;
constexpr double kYawLoadRatioSoft = 0.45;
constexpr double kYawTorqueMarginHard = 0.20;
constexpr double kYawTorqueMarginSoft = 1.0;
constexpr double kSpinModeEntryYawRate = 0.5;
constexpr double kSpinModeFullYawRate = 2.0;
constexpr double kSpinYawReservePerWheel = 2.0;
constexpr double kSpinYawReserveBufferPerWheel = 0.15;
constexpr double kYawPredictionHorizon = 0.15;
constexpr double kYawPredictedSplitSoft = 0.09;
constexpr double kYawPredictedSplitHard = 0.20;
constexpr double kYawSplitActivitySoft = 0.22;
constexpr double kYawSplitActivityHard = 0.65;
constexpr double kYawPredictedRollSoft = 0.06;
constexpr double kYawPredictedRollHard = 0.16;
constexpr double kYawForceFilterTimeConstant = 0.025;
constexpr double kYawForceRateFilterTimeConstant = 0.08;
constexpr double kYawCoordinatorDeceleration = 3.0;
constexpr double kWheelOdometryCorrectionTimeConstant = 0.08;
constexpr double kWheelSlipSoftSpeed = 0.12;
constexpr double kWheelSlipHardSpeed = 0.50;

// Discrete 1 ms LQR gains identified on the parameterized symmetric reduced
// model. State: [theta, dtheta, x, dx, phi, dphi], input: total [T, Tp].
// Values are already transformed into this controller's angle convention.
struct LqrGainNode {
  double leg_length;
  double gain[2][6];
};

constexpr LqrGainNode kLqrGainTable[] = {
    {0.10,
     {{18.84687443, 2.204597038, -17.56153063, -12.56796125,
       31.16626449, 1.186159514},
      {-13.81218044, -1.89071251, 16.70312151, 11.45732676,
       117.5805767, 2.470307643}}},
    {0.12,
     {{22.07552624, 2.715983417, -18.45828148, -13.42905407,
       26.92045115, 1.066222391},
      {-13.76867062, -1.942127577, 14.73430524, 10.26793733,
       121.513341, 2.586773962}}},
    {0.14,
     {{25.0557649, 3.237061102, -19.05484318, -14.09768435,
       23.55940326, 0.9656142325},
      {-13.55678825, -1.977552188, 13.06332566, 9.248676597,
       124.1497596, 2.670856327}}},
    {0.16,
     {{27.84122288, 3.768083185, -19.46692265, -14.64319442,
       20.87910924, 0.8812332948},
      {-13.2838965, -2.005888484, 11.67669523, 8.396479622,
       125.9681849, 2.732521811}}},
    {0.18,
     {{30.47149246, 4.308957568, -19.76179496, -15.10652983,
       18.71333976, 0.8100099716},
      {-12.99719743, -2.030982927, 10.52775942, 7.685660951,
       127.2610167, 2.778692564}}},
    {0.20,
     {{32.97552788, 4.859391062, -19.97956138, -15.51258003,
       16.93728379, 0.7493584399},
      {-12.71680132, -2.054480461, 9.569434506, 7.089044145,
       128.2069399, 2.813999973}}},
    {0.22,
     {{35.37478692, 5.41900226, -20.14488447, -15.8770245,
       15.45960072, 0.6972202649},
      {-12.45067363, -2.077073812, 8.762423466, 6.583555875,
       128.9171899, 2.841539959}}},
    {0.24,
     {{37.6855076, 5.987387968, -20.27343426, -16.21011469,
       14.213594, 0.6519877194},
      {-12.20134431, -2.099046215, 8.075842608, 6.150901368,
       129.4628112, 2.863409748}}},
    {0.26,
     {{39.92025309, 6.564156905, -20.37548464, -16.51879807,
       13.15016415, 0.612408669},
      {-11.96894475, -2.120507512, 7.485872884, 5.776893073,
       129.890438, 2.88105668}}},
    {0.28,
     {{42.08894783, 7.148944496, -20.45797054, -16.80794292,
       12.23270859, 0.5775034599},
      {-11.75259802, -2.141497005, 6.974181879, 5.450569265,
       130.2315092, 2.895499743}}},
    {0.30,
     {{44.19957906, 7.741417645, -20.52569875, -17.08106575,
       11.4335515, 0.546499517},
      {-11.55105853, -2.162027837, 6.526583989, 5.163422177,
       130.5077549, 2.907470569}}},
    {0.32,
     {{46.25868104, 8.341274727, -20.58208153, -17.34077726,
       10.73146417, 0.5187818697},
      {-11.36300399, -2.182105669, 6.132006844, 4.90879421,
       130.7345427, 2.917504293}}},
    {0.34,
     {{48.27167584, 8.948243345, -20.62959379, -17.58906362,
       10.10993579, 0.4938562326},
      {-11.18716371, -2.201735966, 5.781721262, 4.681424428,
       130.9229689, 2.925999106}}},
    {0.36,
     {{50.24311924, 9.56207744, -20.67006551, -17.82746936,
       9.555956148, 0.4713215114},
      {-11.02236979, -2.220926446, 5.468774115, 4.477113028,
       131.0811976, 2.933256004}}},
    {0.38,
     {{52.17688089, 10.18255419, -20.70487349, -18.05721916,
       9.059148054, 0.4508492309},
      {-10.86757174, -2.239687412, 5.187570553, 4.292473231,
       131.2153387, 2.939505821}}},
    {0.40,
     {{54.07627928, 10.8094711, -20.73506991, -18.27930097,
       8.611141059, 0.4321680523},
      {-10.72183456, -2.25803137, 4.933564819, 4.124747292,
       131.3300357, 2.944927921}}},
};

void InterpolateLqrGain(double leg_length, double gain[2][6]) {
  constexpr int node_count =
      sizeof(kLqrGainTable) / sizeof(kLqrGainTable[0]);
  const double scheduled_length = std::fmin(
      std::fmax(leg_length, kLqrGainTable[0].leg_length),
      kLqrGainTable[node_count - 1].leg_length);
  int upper = 1;
  while (upper < node_count - 1 &&
         scheduled_length > kLqrGainTable[upper].leg_length) {
    ++upper;
  }
  const LqrGainNode& lower_node = kLqrGainTable[upper - 1];
  const LqrGainNode& upper_node = kLqrGainTable[upper];
  const double alpha =
      (scheduled_length - lower_node.leg_length) /
      (upper_node.leg_length - lower_node.leg_length);
  for (int input = 0; input < 2; ++input) {
    for (int state = 0; state < 6; ++state) {
      gain[input][state] = lower_node.gain[input][state] +
          alpha * (upper_node.gain[input][state] -
                   lower_node.gain[input][state]);
    }
  }
}

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
  // Preserve this controller's positive-pitch convention while removing yaw
  // from the denominator.  rotation[0] alone vanishes at +/-90 deg yaw.
  pitch = std::atan2(rotation[6],
                     std::hypot(rotation[0], rotation[3]));
  yaw = std::atan2(rotation[3], rotation[0]);
}

bool GeomsInContact(const mjData* data, int geom_1, int geom_2) {
  for (int i = 0; i < data->ncon; ++i) {
    const mjContact& contact = data->contact[i];
    if ((contact.geom[0] == geom_1 && contact.geom[1] == geom_2) ||
        (contact.geom[0] == geom_2 && contact.geom[1] == geom_1)) {
      return true;
    }
  }
  return false;
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

  double cx;
  double cz;
  if (branch == 1) {
    cx = px - h * dbz / distance;
    cz = pz + h * dbx / distance;
  } else {
    cx = px + h * dbz / distance;
    cz = pz - h * dbx / distance;
  }

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

bool NumericalJacobian(double phi1, double phi2, int branch, double jacobian[2][2]) {
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

struct LegKinematics {
  double hx;
  double hz;
  double length;
  double length_rate;
  double angle;
  double angle_rate;
  double jacobian[2][2];
};

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

  const double h_velocity_x =
      leg.jacobian[0][0] * dphi1 + leg.jacobian[0][1] * dphi2;
  const double h_velocity_z =
      leg.jacobian[1][0] * dphi1 + leg.jacobian[1][1] * dphi2;
  leg.length_rate =
      (leg.hx * h_velocity_x + leg.hz * h_velocity_z) / leg.length;
  leg.angle = std::atan2(leg.hx, -leg.hz);
  leg.angle_rate = (-leg.hz * h_velocity_x + leg.hx * h_velocity_z) /
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
  const double tangent_x = -radial_z;
  const double tangent_z = radial_x;
  const double tangential_force = leg_angle_torque / leg.length;
  const double force_x = axial_force * radial_x + tangential_force * tangent_x;
  const double force_z = axial_force * radial_z + tangential_force * tangent_z;
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
}  // namespace

void WbrControllerV2::SetControlMode(WbrControlMode mode) {
  if (mode == control_mode_) {
    return;
  }
  control_mode_ = mode;
  lqr_enabled_ = mode == WbrControlMode::kGroundBalance;
  lqr_initialized_ = false;
  yaw_initialized_ = false;
  filtered_x_speed_ = 0.0;
  filtered_pitch_speed_ = 0.0;
  filtered_yaw_speed_ = 0.0;
  filtered_yaw_acceleration_ = 0.0;
  previous_yaw_speed_ = 0.0;
  filtered_wheel_speed_difference_ = 0.0;
  commanded_linear_velocity_ = 0.0;
  commanded_yaw_rate_ = 0.0;
  coordinated_yaw_rate_ = 0.0;
  commanded_yaw_torque_ = 0.0;
  commanded_differential_leg_angle_torque_ = 0.0;
  differential_leg_angle_reference_ = 0.0;
  support_factor_1_ = 0.0;
  support_factor_2_ = 0.0;
  filtered_normal_force_[0] = 0.0;
  filtered_normal_force_[1] = 0.0;
  filtered_normal_force_rate_[0] = 0.0;
  filtered_normal_force_rate_[1] = 0.0;
  split_activity_ = 0.0;
  estimated_roll_ = 0.0;
  estimated_pitch_ = 0.0;
  estimated_yaw_ = 0.0;
  estimated_x_ = 0.0;
  estimated_x_speed_ = 0.0;
  wheel_odometry_confidence_ = 0.0;
  leg_length_integral_[0] = 0.0;
  leg_length_integral_[1] = 0.0;
  wheel_contact_grace_[0] = 0.0;
  wheel_contact_grace_[1] = 0.0;
  contact_leg_length_offset_[0] = 0.0;
  contact_leg_length_offset_[1] = 0.0;
  contact_safety_state_ = WbrContactSafetyState::kAirborne;
  contact_candidate_state_ = WbrContactSafetyState::kAirborne;
  contact_state_elapsed_ = 0.0;
  contact_candidate_elapsed_ = 0.0;
  yaw_coordinator_initialized_ = false;
  state_estimator_initialized_ = false;
}

void WbrControllerV2::Reset(const mjModel* model) {
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
  joint_wheel_ = mj_name2id(model, mjOBJ_JOINT, "q_H_wheel");
  joint_wheel_2_ = mj_name2id(model, mjOBJ_JOINT, "q_H_wheel_2");
  joint_root_ = mj_name2id(model, mjOBJ_JOINT, "root_free");
  body_plate_ = mj_name2id(model, mjOBJ_BODY, "plate");
  body_wheel_1_ = mj_name2id(model, mjOBJ_BODY, "H_wheel_body");
  body_wheel_2_ = mj_name2id(model, mjOBJ_BODY, "H_wheel_body_2");
  geom_floor_ = mj_name2id(model, mjOBJ_GEOM, "floor");
  geom_wheel_1_ = mj_name2id(model, mjOBJ_GEOM, "H_wheel");
  geom_wheel_2_ = mj_name2id(model, mjOBJ_GEOM, "H_wheel_2");
  sensor_imu_gyro_ = mj_name2id(model, mjOBJ_SENSOR, "imu_gyro");
  sensor_imu_accelerometer_ =
      mj_name2id(model, mjOBJ_SENSOR, "imu_accelerometer");
  sensor_imu_quaternion_ =
      mj_name2id(model, mjOBJ_SENSOR, "imu_quaternion");
  sensor_wheel_speed_1_ =
      mj_name2id(model, mjOBJ_SENSOR, "wheel_speed_1");
  sensor_wheel_speed_2_ =
      mj_name2id(model, mjOBJ_SENSOR, "wheel_speed_2");

  branch_1_ = 1;
  branch_2_ = 1;
  leg_speed_1_ = 0.0;
  leg_speed_2_ = 0.0;
  leg_angle_speed_1_ = 0.0;
  leg_angle_speed_2_ = 0.0;
  filtered_x_speed_ = 0.0;
  filtered_pitch_speed_ = 0.0;
  filtered_yaw_speed_ = 0.0;
  filtered_yaw_acceleration_ = 0.0;
  previous_yaw_speed_ = 0.0;
  filtered_wheel_speed_difference_ = 0.0;
  x_reference_ = 0.0;
  yaw_reference_ = 0.0;
  target_linear_velocity_ = 0.0;
  target_yaw_rate_ = 0.0;
  commanded_linear_velocity_ = 0.0;
  commanded_yaw_rate_ = 0.0;
  coordinated_yaw_rate_ = 0.0;
  commanded_yaw_torque_ = 0.0;
  commanded_differential_leg_angle_torque_ = 0.0;
  differential_leg_angle_reference_ = 0.0;
  commanded_leg_length_ = 0.18;
  commanded_leg_angle_ = 0.0;
  support_factor_1_ = 0.0;
  support_factor_2_ = 0.0;
  filtered_normal_force_[0] = 0.0;
  filtered_normal_force_[1] = 0.0;
  filtered_normal_force_rate_[0] = 0.0;
  filtered_normal_force_rate_[1] = 0.0;
  split_activity_ = 0.0;
  estimated_roll_ = 0.0;
  estimated_pitch_ = 0.0;
  estimated_yaw_ = 0.0;
  estimated_x_ = 0.0;
  estimated_x_speed_ = 0.0;
  wheel_odometry_confidence_ = 0.0;
  leg_length_integral_[0] = 0.0;
  leg_length_integral_[1] = 0.0;
  wheel_contact_grace_[0] = 0.0;
  wheel_contact_grace_[1] = 0.0;
  contact_leg_length_offset_[0] = 0.0;
  contact_leg_length_offset_[1] = 0.0;
  contact_safety_state_ = WbrContactSafetyState::kAirborne;
  contact_candidate_state_ = WbrContactSafetyState::kAirborne;
  contact_state_elapsed_ = 0.0;
  contact_candidate_elapsed_ = 0.0;
  last_time_ = 0.0;
  command_initialized_ = false;
  lqr_initialized_ = false;
  yaw_initialized_ = false;
  yaw_coordinator_initialized_ = false;
  state_estimator_initialized_ = false;

  if (act_q_b_ < 0 || act_q_d_ < 0 || joint_q_b_ < 0 || joint_q_d_ < 0) {
    std::printf("WbrControllerV2: first leg actuators/joints not found.\n");
  }
  if (act_q_b_2_ < 0 || act_q_d_2_ < 0 || joint_q_b_2_ < 0 || joint_q_d_2_ < 0) {
    std::printf("WbrControllerV2: second leg actuators/joints not found.\n");
  }
  if (joint_root_ >= 0 && body_plate_ >= 0 && body_wheel_1_ >= 0 && body_wheel_2_ >= 0) {
    std::printf("WbrControllerV2: fixed-gain LQR and leg-length VMC enabled.\n");
  }
}

void WbrControllerV2::SyncTargetsFromState(const mjModel* model, const mjData* data,
                                            double& target_leg_length,
                                            double& target_leg_angle) {
  if (!model || !data) {
    return;
  }
  
  telemetry_ = {};
  target_leg_length = 0.18;
  target_leg_angle = 0.0;

  double length_sum = 0.0;
  double angle_sin_sum = 0.0;
  double angle_cos_sum = 0.0;
  int valid_legs = 0;
  const int joint_b[2] = {joint_q_b_, joint_q_b_2_};
  const int joint_d[2] = {joint_q_d_, joint_q_d_2_};
  const int branch[2] = {branch_1_, branch_2_};
  const double mirror[2] = {1.0, -1.0};
  const double body_pitch = body_plate_ >= 0 ? RootPitch(data, body_plate_) : 0.0;
  for (int leg = 0; leg < 2; ++leg) {
    if (joint_b[leg] < 0 || joint_d[leg] < 0) {
      continue;
    }
    const double phi1 = mirror[leg] * data->qpos[model->jnt_qposadr[joint_d[leg]]];
    const double phi2 = mirror[leg] * data->qpos[model->jnt_qposadr[joint_b[leg]]];
    double hx;
    double hz;
    if (ForwardKinematics(phi1, phi2, branch[leg], hx, hz)) {
      const double angle = std::atan2(hx, -hz) + body_pitch;
      length_sum += std::hypot(hx, hz);
      angle_sin_sum += std::sin(angle);
      angle_cos_sum += std::cos(angle);
      ++valid_legs;
    }
  }
  if (valid_legs > 0) {
    commanded_leg_length_ = length_sum / valid_legs;
    commanded_leg_angle_ = std::atan2(angle_sin_sum, angle_cos_sum);
    target_leg_length = commanded_leg_length_;
    target_leg_angle = commanded_leg_angle_;
    command_initialized_ = true;
  }
}

void WbrControllerV2::Apply(const mjModel* model, mjData* data,
                            double& target_leg_length, double& target_leg_angle) {
  if (!model || !data) {
    return;
  }
  telemetry_ = {};

  target_leg_length = Clamp(target_leg_length, kMinTargetLegLength, kTargetHRadius);
  target_leg_angle = Clamp(target_leg_angle, -kMaxTargetLegAngle, kMaxTargetLegAngle);
  const bool time_reset = lqr_initialized_ && data->time < last_time_;
  double control_dt = model->opt.timestep;
  if (lqr_initialized_ && !time_reset && data->time > last_time_) {
    control_dt = Clamp(data->time - last_time_, 1e-4, 0.02);
  }
  target_linear_velocity_ = Clamp(target_linear_velocity_,
                                  -kMaxLinearVelocity, kMaxLinearVelocity);
  target_yaw_rate_ = Clamp(target_yaw_rate_, -kMaxYawRate, kMaxYawRate);
  const double safe_target_yaw_rate = Clamp(
      target_yaw_rate_, -kSustainedYawRateLimit, kSustainedYawRateLimit);
  commanded_linear_velocity_ = MoveTowards(
      commanded_linear_velocity_, target_linear_velocity_,
      kLinearAccelerationLimit * control_dt);
  const bool yaw_rate_braking =
      std::fabs(safe_target_yaw_rate) < std::fabs(commanded_yaw_rate_) ||
      safe_target_yaw_rate * commanded_yaw_rate_ < 0.0;
  commanded_yaw_rate_ = MoveTowards(
      commanded_yaw_rate_, safe_target_yaw_rate,
      (yaw_rate_braking ? kYawCommandDeceleration
                        : kYawCommandAcceleration) * control_dt);
  telemetry_.commanded_linear_velocity = commanded_linear_velocity_;
  telemetry_.commanded_yaw_rate = commanded_yaw_rate_;

  const bool first_leg_available =
      act_q_b_ >= 0 && act_q_d_ >= 0 && joint_q_b_ >= 0 && joint_q_d_ >= 0;
  const bool second_leg_available =
      act_q_b_2_ >= 0 && act_q_d_2_ >= 0 && joint_q_b_2_ >= 0 && joint_q_d_2_ >= 0;
  LegKinematics leg_1{};
  LegKinematics leg_2{};
  const bool first_leg_valid = first_leg_available &&
      ComputeLegKinematics(model, data, joint_q_b_, joint_q_d_, branch_1_, 1.0, leg_1);
  const bool second_leg_valid = second_leg_available &&
      ComputeLegKinematics(model, data, joint_q_b_2_, joint_q_d_2_, branch_2_, -1.0, leg_2);

  if (first_leg_valid) {
    telemetry_.leg_length[0] = leg_1.length;
    telemetry_.leg_length_rate[0] = leg_1.length_rate;
    leg_speed_1_ += kLegSpeedFilter * (leg_1.length_rate - leg_speed_1_);
    leg_angle_speed_1_ +=
        kStateSpeedFilter * (leg_1.angle_rate - leg_angle_speed_1_);
  }
  if (second_leg_valid) {
    telemetry_.leg_length[1] = leg_2.length;
    telemetry_.leg_length_rate[1] = leg_2.length_rate;
    leg_speed_2_ += kLegSpeedFilter * (leg_2.length_rate - leg_speed_2_);
    leg_angle_speed_2_ +=
        kStateSpeedFilter * (leg_2.angle_rate - leg_angle_speed_2_);
  }

  if (!command_initialized_ && first_leg_valid && second_leg_valid) {
    commanded_leg_length_ = 0.5 * (leg_1.length + leg_2.length);
    commanded_leg_angle_ =
        std::atan2(std::sin(leg_1.angle) + std::sin(leg_2.angle),
                   std::cos(leg_1.angle) + std::cos(leg_2.angle));
    command_initialized_ = true;
  }
  commanded_leg_length_ =
      MoveTowards(commanded_leg_length_, target_leg_length,
                  kTargetLengthSlewRate * control_dt);
  commanded_leg_angle_ =
      MoveTowards(commanded_leg_angle_, target_leg_angle,
                  kTargetAngleSlewRate * control_dt);

  double total_wheel_torque = 0.0;
  double total_leg_angle_torque = 0.0;
  double total_yaw_torque = 0.0;
  double differential_leg_angle_torque = 0.0;
  double support_feedforward_1 = 0.0;
  double support_feedforward_2 = 0.0;
  bool effective_wheel_grounded[2] = {false, false};
  const bool lqr_state_valid =
      first_leg_valid && second_leg_valid && joint_root_ >= 0 && body_plate_ >= 0 &&
      body_wheel_1_ >= 0 && body_wheel_2_ >= 0;
  if (lqr_state_valid) {
    mjtNum wheel_velocity_1[6];
    mjtNum wheel_velocity_2[6];
    mjtNum plate_velocity[6];
    mj_objectVelocity(model, data, mjOBJ_BODY, body_wheel_1_, wheel_velocity_1, 0);
    mj_objectVelocity(model, data, mjOBJ_BODY, body_wheel_2_, wheel_velocity_2, 0);
    mj_objectVelocity(model, data, mjOBJ_BODY, body_plate_, plate_velocity, 0);
    const bool wheel_1_grounded =
        geom_floor_ >= 0 && geom_wheel_1_ >= 0 &&
        GeomsInContact(data, geom_floor_, geom_wheel_1_);
    const bool wheel_2_grounded =
        geom_floor_ >= 0 && geom_wheel_2_ >= 0 &&
        GeomsInContact(data, geom_floor_, geom_wheel_2_);

    double imu_roll = RootRoll(data, body_plate_);
    double imu_pitch = RootPitch(data, body_plate_);
    double imu_yaw = RootYaw(data, body_plate_);
    double imu_roll_rate = plate_velocity[0];
    double imu_pitch_rate = -plate_velocity[1];
    double imu_yaw_rate = plate_velocity[2];
    double imu_forward_acceleration = 0.0;
    mjtNum imu_rotation[9];
    mju_copy(imu_rotation, data->xmat + 9 * body_plate_, 9);
    const mjtNum* imu_quaternion = SensorData(
        model, data, sensor_imu_quaternion_, 4);
    if (imu_quaternion) {
      QuaternionToEuler(imu_quaternion, imu_roll, imu_pitch, imu_yaw,
                        imu_rotation);
    }
    const mjtNum* imu_gyro = SensorData(model, data, sensor_imu_gyro_, 3);
    if (imu_gyro) {
      imu_roll_rate = imu_gyro[0];
      imu_pitch_rate = -imu_gyro[1];
      imu_yaw_rate = imu_gyro[2];
    }
    const double* plate_xmat = data->xmat + 9 * body_plate_;
    const double forward[3] = {
        plate_xmat[0], plate_xmat[3], plate_xmat[6]};
    const double lateral[3] = {
        plate_xmat[1], plate_xmat[4], plate_xmat[7]};
    // MuJoCo exposes chassis translational velocity directly.  Use its
    // body-forward projection as the simulator's longitudinal observation;
    // wheel angular odometry becomes unusable during differential spin and
    // accelerometer-only integration otherwise builds a false speed state.
    const double chassis_forward_speed =
        plate_velocity[3] * forward[0] +
        plate_velocity[4] * forward[1] +
        plate_velocity[5] * forward[2];
    const mjtNum* imu_accelerometer = SensorData(
        model, data, sensor_imu_accelerometer_, 3);
    if (imu_accelerometer) {
      const double world_acceleration[3] = {
          imu_rotation[0] * imu_accelerometer[0] +
              imu_rotation[1] * imu_accelerometer[1] +
              imu_rotation[2] * imu_accelerometer[2] + model->opt.gravity[0],
          imu_rotation[3] * imu_accelerometer[0] +
              imu_rotation[4] * imu_accelerometer[1] +
              imu_rotation[5] * imu_accelerometer[2] + model->opt.gravity[1],
          imu_rotation[6] * imu_accelerometer[0] +
              imu_rotation[7] * imu_accelerometer[1] +
              imu_rotation[8] * imu_accelerometer[2] + model->opt.gravity[2]};
      imu_forward_acceleration =
          world_acceleration[0] * forward[0] +
          world_acceleration[1] * forward[1] +
          world_acceleration[2] * forward[2];
    }
    double raw_wheel_speed_difference = 0.0;
    double wheel_odometry_forward_speed = 0.0;
    if (joint_wheel_ >= 0 && joint_wheel_2_ >= 0 &&
        geom_wheel_1_ >= 0 && geom_wheel_2_ >= 0) {
      const double wheel_radius = 0.5 *
          (model->geom_size[3 * geom_wheel_1_] +
           model->geom_size[3 * geom_wheel_2_]);
      const mjtNum* measured_wheel_speed_1 = SensorData(
          model, data, sensor_wheel_speed_1_, 1);
      const mjtNum* measured_wheel_speed_2 = SensorData(
          model, data, sensor_wheel_speed_2_, 1);
      const double wheel_speed_1 = measured_wheel_speed_1
          ? measured_wheel_speed_1[0]
          : data->qvel[model->jnt_dofadr[joint_wheel_]];
      const double wheel_speed_2 = measured_wheel_speed_2
          ? measured_wheel_speed_2[0]
          : data->qvel[model->jnt_dofadr[joint_wheel_2_]];
      raw_wheel_speed_difference =
          wheel_radius * (wheel_speed_1 + wheel_speed_2);
      wheel_odometry_forward_speed =
          0.5 * wheel_radius * (wheel_speed_1 - wheel_speed_2);
    }

    const double orientation_alpha = 1.0 - std::exp(-control_dt / 0.03);
    if (!state_estimator_initialized_ || time_reset) {
      estimated_roll_ = imu_roll;
      estimated_pitch_ = imu_pitch;
      estimated_yaw_ = imu_yaw;
      // LQR x is body-forward traveled distance, not world-X position.  Using
      // world X makes the sagittal state vanish and reverse sign at 90/270 deg
      // yaw during continuous spin.
      estimated_x_ = 0.0;
      estimated_x_speed_ = chassis_forward_speed;
      state_estimator_initialized_ = true;
    } else {
      estimated_roll_ += imu_roll_rate * control_dt;
      estimated_pitch_ += imu_pitch_rate * control_dt;
      estimated_yaw_ += imu_yaw_rate * control_dt;
      estimated_roll_ += orientation_alpha *
          std::remainder(imu_roll - estimated_roll_, 2.0 * M_PI);
      estimated_pitch_ += orientation_alpha *
          std::remainder(imu_pitch - estimated_pitch_, 2.0 * M_PI);
      estimated_yaw_ += orientation_alpha *
          std::remainder(imu_yaw - estimated_yaw_, 2.0 * M_PI);

      estimated_x_speed_ += imu_forward_acceleration * control_dt;
      const double chassis_velocity_alpha = 1.0 - std::exp(
          -control_dt / kChassisVelocityCorrectionTimeConstant);
      estimated_x_speed_ += chassis_velocity_alpha *
          (chassis_forward_speed - estimated_x_speed_);
      const double contact_confidence =
          wheel_1_grounded && wheel_2_grounded
              ? 1.0
              : ((wheel_1_grounded || wheel_2_grounded) ? 0.2 : 0.0);
      const double slip_confidence = FadeAuthority(
          std::fabs(wheel_odometry_forward_speed - estimated_x_speed_),
          kWheelSlipSoftSpeed, kWheelSlipHardSpeed);
      wheel_odometry_confidence_ = contact_confidence * slip_confidence;
      const double odometry_alpha = wheel_odometry_confidence_ *
          (1.0 - std::exp(
              -control_dt / kWheelOdometryCorrectionTimeConstant));
      estimated_x_speed_ += odometry_alpha *
          (wheel_odometry_forward_speed - estimated_x_speed_);
      estimated_x_ += estimated_x_speed_ * control_dt;
    }
    double x = estimated_x_;
    double raw_x_speed = estimated_x_speed_;
    const double raw_pitch_speed = imu_pitch_rate;
    const double phi = estimated_pitch_;
    telemetry_.estimated_roll = estimated_roll_;
    telemetry_.estimated_pitch = estimated_pitch_;
    telemetry_.estimated_yaw = estimated_yaw_;
    telemetry_.estimated_x = estimated_x_;
    telemetry_.estimated_x_speed = estimated_x_speed_;
    telemetry_.wheel_odometry_x_speed =
        wheel_odometry_forward_speed;
    telemetry_.wheel_odometry_confidence = wheel_odometry_confidence_;
    const double wheel_separation[3] = {
        data->xpos[3 * body_wheel_1_] - data->xpos[3 * body_wheel_2_],
        data->xpos[3 * body_wheel_1_ + 1] - data->xpos[3 * body_wheel_2_ + 1],
        data->xpos[3 * body_wheel_1_ + 2] - data->xpos[3 * body_wheel_2_ + 2]};
    const double track_width = std::fmax(
        std::fabs(wheel_separation[0] * lateral[0] +
                  wheel_separation[1] * lateral[1] +
                  wheel_separation[2] * lateral[2]),
        0.1);

    wheel_contact_grace_[0] = wheel_1_grounded
        ? kContactGraceTime
        : std::fmax(0.0, wheel_contact_grace_[0] - control_dt);
    wheel_contact_grace_[1] = wheel_2_grounded
        ? kContactGraceTime
        : std::fmax(0.0, wheel_contact_grace_[1] - control_dt);
    const bool wheel_1_effectively_grounded = wheel_contact_grace_[0] > 0.0;
    const bool wheel_2_effectively_grounded = wheel_contact_grace_[1] > 0.0;
    effective_wheel_grounded[0] = wheel_1_effectively_grounded;
    effective_wheel_grounded[1] = wheel_2_effectively_grounded;

    const WbrContactSafetyState previous_contact_safety_state =
        contact_safety_state_;
    WbrContactSafetyState observed_contact_state =
        WbrContactSafetyState::kAirborne;
    if (wheel_1_grounded && wheel_2_grounded) {
      observed_contact_state = WbrContactSafetyState::kDualSupport;
    } else if (wheel_1_grounded) {
      observed_contact_state = WbrContactSafetyState::kSingleSupportFirst;
    } else if (wheel_2_grounded) {
      observed_contact_state = WbrContactSafetyState::kSingleSupportSecond;
    }
    contact_state_elapsed_ += control_dt;
    WbrContactSafetyState requested_contact_state = observed_contact_state;
    if (observed_contact_state == WbrContactSafetyState::kDualSupport &&
        contact_safety_state_ != WbrContactSafetyState::kDualSupport) {
      requested_contact_state = WbrContactSafetyState::kRecovery;
    }
    if (requested_contact_state != contact_candidate_state_) {
      contact_candidate_state_ = requested_contact_state;
      contact_candidate_elapsed_ = 0.0;
    } else {
      contact_candidate_elapsed_ += control_dt;
    }
    const double transition_debounce =
        requested_contact_state == WbrContactSafetyState::kRecovery
            ? kContactRecoveryDebounceTime
            : kContactLossDebounceTime;
    if (requested_contact_state != contact_safety_state_ &&
        contact_candidate_elapsed_ >= transition_debounce) {
      contact_safety_state_ = requested_contact_state;
      contact_state_elapsed_ = 0.0;
    }
    if (contact_safety_state_ == WbrContactSafetyState::kRecovery &&
        observed_contact_state == WbrContactSafetyState::kDualSupport &&
        contact_state_elapsed_ >= kContactRecoveryRampTime &&
        std::fabs(estimated_roll_) <= kRecoveryMaxRoll &&
        std::fabs(estimated_pitch_) <= kRecoveryMaxPitch) {
      contact_safety_state_ = WbrContactSafetyState::kDualSupport;
      contact_candidate_state_ = WbrContactSafetyState::kDualSupport;
      contact_state_elapsed_ = 0.0;
      contact_candidate_elapsed_ = 0.0;
    }
    const bool single_support =
        contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst ||
        contact_safety_state_ == WbrContactSafetyState::kSingleSupportSecond;
    const bool contact_airborne =
        contact_safety_state_ == WbrContactSafetyState::kAirborne;
    if (contact_safety_state_ != previous_contact_safety_state) {
      // Re-anchor when support changes so stale position debt is not carried
      // across a wheel-slip or airborne interval.
      x_reference_ = x;
      filtered_x_speed_ = raw_x_speed;
    }
    double contact_authority_scale = 1.0;
    if (single_support) {
      contact_authority_scale = kSingleSupportLqrScale;
    } else if (contact_airborne) {
      contact_authority_scale = 0.0;
    } else if (contact_safety_state_ == WbrContactSafetyState::kRecovery) {
      contact_authority_scale = 0.5 + 0.5 * Clamp(
          contact_state_elapsed_ / kContactRecoveryRampTime, 0.0, 1.0);
    }
    telemetry_.contact_safety_state = contact_safety_state_;
    telemetry_.contact_authority_scale = contact_authority_scale;

    if (single_support || contact_airborne) {
      commanded_linear_velocity_ = MoveTowards(
          commanded_linear_velocity_, 0.0,
          kEmergencyLinearDeceleration * control_dt);
      commanded_yaw_rate_ = MoveTowards(
          commanded_yaw_rate_, 0.0,
          kYawCommandDeceleration * control_dt);
      telemetry_.commanded_linear_velocity = commanded_linear_velocity_;
      telemetry_.commanded_yaw_rate = commanded_yaw_rate_;
    }
    const bool balance_active =
        lqr_enabled_ && control_mode_ == WbrControlMode::kGroundBalance &&
        !contact_airborne &&
        (wheel_1_effectively_grounded || wheel_2_effectively_grounded);
    telemetry_.wheel_grounded[0] = wheel_1_grounded;
    telemetry_.wheel_grounded[1] = wheel_2_grounded;
    telemetry_.balance_active = balance_active;
    if (balance_active) {
      const bool reset_lqr = !lqr_initialized_ || time_reset;
      if (reset_lqr) {
        x_reference_ = x;
        filtered_x_speed_ = raw_x_speed;
        filtered_pitch_speed_ = raw_pitch_speed;
        filtered_yaw_speed_ = imu_yaw_rate;
        filtered_wheel_speed_difference_ = raw_wheel_speed_difference;
        lqr_initialized_ = true;
      } else {
        filtered_x_speed_ +=
            kStateSpeedFilter * (raw_x_speed - filtered_x_speed_);
        filtered_pitch_speed_ +=
            kStateSpeedFilter * (raw_pitch_speed - filtered_pitch_speed_);
        const double yaw_rate_alpha =
            1.0 - std::exp(-control_dt / kYawRateFilterTimeConstant);
        filtered_yaw_speed_ +=
            yaw_rate_alpha * (imu_yaw_rate - filtered_yaw_speed_);
        filtered_wheel_speed_difference_ +=
            yaw_rate_alpha * (raw_wheel_speed_difference -
                              filtered_wheel_speed_difference_);
      }
      if (!yaw_initialized_ || time_reset) {
        yaw_reference_ = estimated_yaw_;
        filtered_yaw_speed_ = imu_yaw_rate;
        previous_yaw_speed_ = filtered_yaw_speed_;
        filtered_yaw_acceleration_ = 0.0;
        filtered_wheel_speed_difference_ = raw_wheel_speed_difference;
        yaw_initialized_ = true;
      }
      if (!reset_lqr && !time_reset && !single_support) {
        // Keep the body-forward position reference active during pure spin.
        // Releasing it with x_reference=x permitted a constant sagittal drift,
        // which appears as a curved left-forward path while yawing.
        x_reference_ += commanded_linear_velocity_ * control_dt;
        x_reference_ = Clamp(x_reference_, x - kMaxPositionTrackingError,
                             x + kMaxPositionTrackingError);
        // A/D is a yaw-rate command, not an angle trajectory. While turning,
        // follow the measured heading so slip does not accumulate angle debt;
        // when the command reaches zero, the last measured heading is held.
        if (std::fabs(commanded_yaw_rate_) > 1e-5 ||
            std::fabs(target_yaw_rate_) > 1e-5) {
          yaw_reference_ = estimated_yaw_;
        }
      }
    } else {
      lqr_initialized_ = false;
      filtered_x_speed_ = raw_x_speed;
      filtered_pitch_speed_ = raw_pitch_speed;
      filtered_yaw_speed_ = imu_yaw_rate;
      filtered_wheel_speed_difference_ = raw_wheel_speed_difference;
    }
    const double world_leg_angle_1 =
        std::remainder(leg_1.angle + phi, 2.0 * M_PI);
    const double world_leg_angle_2 =
        std::remainder(leg_2.angle + phi, 2.0 * M_PI);
    const double theta =
        std::atan2(std::sin(world_leg_angle_1) + std::sin(world_leg_angle_2),
                   std::cos(world_leg_angle_1) + std::cos(world_leg_angle_2));
    const double theta_rate =
        0.5 * (leg_1.angle_rate + leg_2.angle_rate) + raw_pitch_speed;

    telemetry_.differential_leg_angle_error = std::remainder(
        leg_1.angle - leg_2.angle, 2.0 * M_PI);
    telemetry_.differential_leg_angle_rate =
        leg_angle_speed_1_ - leg_angle_speed_2_;
    const bool split_reference_enabled =
        yaw_enabled_ &&
        contact_safety_state_ == WbrContactSafetyState::kDualSupport &&
        ((wheel_1_grounded || wheel_contact_grace_[0] > 0.0) &&
         (wheel_2_grounded || wheel_contact_grace_[1] > 0.0));
    const double split_reference_yaw_rate =
        filtered_yaw_speed_ +
        0.10 * (commanded_yaw_rate_ - filtered_yaw_speed_);
    const double desired_split_reference = split_reference_enabled
        ? Clamp(kYawSplitReferencePerRate * split_reference_yaw_rate,
                -kMaxYawSplitReference, kMaxYawSplitReference)
        : 0.0;
    const double previous_split_reference =
        differential_leg_angle_reference_;
    differential_leg_angle_reference_ = MoveTowards(
        differential_leg_angle_reference_, desired_split_reference,
        kYawSplitReferenceSlewRate * control_dt);
    const double split_reference_rate = control_dt > 0.0
        ? (differential_leg_angle_reference_ - previous_split_reference) /
              control_dt
        : 0.0;
    const double split_residual_error = std::remainder(
        telemetry_.differential_leg_angle_error -
            differential_leg_angle_reference_,
        2.0 * M_PI);
    const double split_residual_rate =
        telemetry_.differential_leg_angle_rate - split_reference_rate;
    telemetry_.differential_leg_angle_reference =
        differential_leg_angle_reference_;
    telemetry_.differential_leg_angle_residual = split_residual_error;
    telemetry_.differential_leg_angle_rate_residual = split_residual_rate;
    const double roll = estimated_roll_;
    telemetry_.wheel_normal_force[0] = ContactNormalForce(
        model, data, geom_floor_, geom_wheel_1_);
    telemetry_.wheel_normal_force[1] = ContactNormalForce(
        model, data, geom_floor_, geom_wheel_2_);

    const double force_alpha = 1.0 - std::exp(
        -control_dt / kYawForceFilterTimeConstant);
    const double force_rate_alpha = 1.0 - std::exp(
        -control_dt / kYawForceRateFilterTimeConstant);
    if (!yaw_coordinator_initialized_ || time_reset) {
      for (int wheel = 0; wheel < 2; ++wheel) {
        filtered_normal_force_[wheel] = telemetry_.wheel_normal_force[wheel];
        filtered_normal_force_rate_[wheel] = 0.0;
      }
      split_activity_ = std::fabs(split_residual_rate);
      coordinated_yaw_rate_ = 0.0;
      yaw_coordinator_initialized_ = true;
    } else {
      for (int wheel = 0; wheel < 2; ++wheel) {
        const double previous_force = filtered_normal_force_[wheel];
        filtered_normal_force_[wheel] += force_alpha *
            (telemetry_.wheel_normal_force[wheel] -
             filtered_normal_force_[wheel]);
        const double raw_force_rate =
            (filtered_normal_force_[wheel] - previous_force) / control_dt;
        filtered_normal_force_rate_[wheel] += force_rate_alpha *
            (raw_force_rate - filtered_normal_force_rate_[wheel]);
      }
      const double split_rate_magnitude = std::fabs(split_residual_rate);
      const double split_activity_time_constant =
          split_rate_magnitude > split_activity_ ? 0.02 : 0.8;
      const double split_activity_alpha = 1.0 - std::exp(
          -control_dt / split_activity_time_constant);
      split_activity_ += split_activity_alpha *
          (split_rate_magnitude - split_activity_);
    }

    telemetry_.predicted_split_error =
        split_residual_error + kYawPredictionHorizon * split_residual_rate;
    telemetry_.split_activity = split_activity_;
    const double residual_split_authority = std::fmin(
        FadeAuthority(std::fabs(telemetry_.predicted_split_error),
                      kYawPredictedSplitSoft, kYawPredictedSplitHard),
        FadeAuthority(split_activity_,
                      kYawSplitActivitySoft, kYawSplitActivityHard));
    const double absolute_split_authority = std::fmin(
        FadeAuthority(std::fabs(telemetry_.differential_leg_angle_error),
                      kAbsoluteLegSplitSoftAngle,
                      kAbsoluteLegSplitHardAngle),
        FadeAuthority(std::fabs(telemetry_.differential_leg_angle_rate),
                      kAbsoluteLegSplitSoftRate,
                      kAbsoluteLegSplitHardRate));
    telemetry_.yaw_split_authority =
        std::fmin(residual_split_authority, absolute_split_authority);
    telemetry_.predicted_roll =
        roll + kYawPredictionHorizon * imu_roll_rate;
    telemetry_.yaw_attitude_authority = std::fmin(
        FadeAuthority(std::fabs(telemetry_.predicted_roll),
                      kYawPredictedRollSoft, kYawPredictedRollHard),
        FadeAuthority(std::fabs(phi),
                      kYawPitchSoftLimit, kYawPitchHardLimit));
    for (int wheel = 0; wheel < 2; ++wheel) {
      telemetry_.predicted_normal_force[wheel] = std::fmax(
          0.0, filtered_normal_force_[wheel] + kYawPredictionHorizon *
              std::fmin(filtered_normal_force_rate_[wheel], 0.0));
    }
    const double minimum_normal_force = std::fmin(
        telemetry_.predicted_normal_force[0],
        telemetry_.predicted_normal_force[1]);
    const double maximum_normal_force = std::fmax(
        telemetry_.predicted_normal_force[0],
        telemetry_.predicted_normal_force[1]);
    const double load_ratio = maximum_normal_force > 1e-6
        ? minimum_normal_force / maximum_normal_force
        : 0.0;
    telemetry_.yaw_contact_authority = std::fmin(
        RiseAuthority(minimum_normal_force,
                      kYawContactForceHard, kYawContactForceSoft),
        RiseAuthority(load_ratio,
                      kYawLoadRatioHard, kYawLoadRatioSoft));
    telemetry_.yaw_authority_scale = std::fmin(
        telemetry_.yaw_split_authority,
        std::fmin(telemetry_.yaw_attitude_authority,
                  telemetry_.yaw_contact_authority));
    if (control_mode_ == WbrControlMode::kGroundBalance &&
        (wheel_1_grounded || wheel_2_grounded)) {
      differential_leg_angle_torque = Clamp(
          -kDifferentialLegAngleKp *
              telemetry_.differential_leg_angle_error -
          kDifferentialLegAngleKd *
              telemetry_.differential_leg_angle_rate,
          -kDifferentialLegAngleTorqueLimit,
          kDifferentialLegAngleTorqueLimit);
    }
    const double support_alpha =
        1.0 - std::exp(-control_dt / kSupportFilterTimeConstant);
    const bool ground_support_enabled =
        control_mode_ == WbrControlMode::kGroundBalance;
    support_factor_1_ +=
        support_alpha *
        (((ground_support_enabled && wheel_1_grounded) ? 1.0 : 0.0) -
         support_factor_1_);
    support_factor_2_ +=
        support_alpha *
        (((ground_support_enabled && wheel_2_grounded) ? 1.0 : 0.0) -
         support_factor_2_);
    const double half_weight =
        0.5 * mj_getTotalmass(model) * std::fabs(model->opt.gravity[2]);
    if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst) {
      support_feedforward_1 = half_weight /
          std::fmax(std::cos(world_leg_angle_1), 0.5);
      support_feedforward_2 = 0.0;
    } else if (contact_safety_state_ ==
               WbrContactSafetyState::kSingleSupportSecond) {
      support_feedforward_1 = 0.0;
      support_feedforward_2 = half_weight /
          std::fmax(std::cos(world_leg_angle_2), 0.5);
    } else {
      support_feedforward_1 =
          support_factor_1_ * half_weight /
          std::fmax(std::cos(world_leg_angle_1), 0.5);
      support_feedforward_2 =
          support_factor_2_ * half_weight /
          std::fmax(std::cos(world_leg_angle_2), 0.5);
    }

    // The sagittal LQR controls the common leg force. Stabilize the orthogonal
    // roll mode with differential axial force, whose sign and gain are
    // identified by lqr_open_loop_test on the free-root model.
    if (ground_support_enabled && wheel_1_grounded && wheel_2_grounded &&
        !single_support && !contact_airborne) {
      const double roll = estimated_roll_;
      const double roll_rate = imu_roll_rate;
      const double differential_force = Clamp(
          -kRollForceKp * roll - kRollForceKd * roll_rate,
          -kRollDifferentialForceLimit, kRollDifferentialForceLimit);
      support_feedforward_1 += 0.5 * differential_force;
      support_feedforward_2 -= 0.5 * differential_force;
    }

    if (time_reset) {
      leg_length_integral_[0] = 0.0;
      leg_length_integral_[1] = 0.0;
    }
    const LegKinematics* legs[2] = {&leg_1, &leg_2};
    const double support_feedforward[2] = {
        support_feedforward_1, support_feedforward_2};
    const double support_factor[2] = {support_factor_1_, support_factor_2_};
    for (int leg = 0; leg < 2; ++leg) {
      const bool integral_enabled =
          control_mode_ == WbrControlMode::kStandLeg ||
          support_factor[leg] > 0.5;
      if (integral_enabled) {
        const double error = commanded_leg_length_ - legs[leg]->length;
        const double integral_limit =
            kLegIntegralForceLimit / kLegLengthKi;
        const double candidate = Clamp(
            leg_length_integral_[leg] + error * control_dt,
            -integral_limit, integral_limit);
        const double candidate_force =
            kLegLengthKp * error - kLegLengthKd *
                (leg == 0 ? leg_speed_1_ : leg_speed_2_) +
            support_feedforward[leg] + kLegLengthKi * candidate;
        const bool pushes_further_into_saturation =
            (candidate_force > kLegForceLimit && error > 0.0) ||
            (candidate_force < -kLegForceLimit && error < 0.0);
        if (!pushes_further_into_saturation) {
          leg_length_integral_[leg] = candidate;
        }
      } else {
        leg_length_integral_[leg] *= std::exp(-control_dt / 0.25);
      }
      telemetry_.integral_force[leg] =
          kLegLengthKi * leg_length_integral_[leg];
    }

    const double state_error[6] = {
        std::remainder(theta - commanded_leg_angle_, 2.0 * M_PI),
        theta_rate,
        x - x_reference_,
        raw_x_speed - commanded_linear_velocity_,
        phi,
        raw_pitch_speed,
    };
    for (int state = 0; state < 6; ++state) {
      telemetry_.state_error[state] = state_error[state];
    }
    if (balance_active) {
      double scheduled_lqr_gain[2][6];
      InterpolateLqrGain(0.5 * (leg_1.length + leg_2.length),
                         scheduled_lqr_gain);
      for (int state = 0; state < 6; ++state) {
        total_wheel_torque -=
            scheduled_lqr_gain[0][state] * state_error[state];
        total_leg_angle_torque -=
            scheduled_lqr_gain[1][state] * state_error[state];
      }
      total_wheel_torque *= contact_authority_scale;
      total_leg_angle_torque *= contact_authority_scale;
      telemetry_.requested_wheel_torque = total_wheel_torque;

      // First form the yaw request from attitude/contact/split safety.  Wheel
      // torque availability is handled by the allocator below; feeding it
      // back here creates a deadlock where yaw cannot request the headroom it
      // needs in order to start.
      telemetry_.spin_mode_blend = Clamp(
          (std::fabs(commanded_yaw_rate_) - kSpinModeEntryYawRate) /
              (kSpinModeFullYawRate - kSpinModeEntryYawRate),
          0.0, 1.0);
      const double coordinated_yaw_target =
          telemetry_.yaw_authority_scale * commanded_yaw_rate_;
      const bool coordinator_braking =
          std::fabs(coordinated_yaw_target) <
          std::fabs(coordinated_yaw_rate_) ||
          coordinated_yaw_target * coordinated_yaw_rate_ < 0.0;
      coordinated_yaw_rate_ = MoveTowards(
          coordinated_yaw_rate_, coordinated_yaw_target,
          (coordinator_braking ? kYawCoordinatorDeceleration
                               : kYawCommandAcceleration) * control_dt);
      telemetry_.coordinated_yaw_rate = coordinated_yaw_rate_;
      bool yaw_braking_request = false;
      if (yaw_enabled_ &&
          contact_safety_state_ == WbrContactSafetyState::kDualSupport &&
          wheel_1_grounded && wheel_2_grounded &&
          model->jnt_type[joint_root_] == mjJNT_FREE) {
        telemetry_.yaw_error = std::remainder(
            estimated_yaw_ - yaw_reference_, 2.0 * M_PI);
        telemetry_.yaw_rate = filtered_yaw_speed_;
        telemetry_.yaw_rate_error =
            coordinated_yaw_rate_ - telemetry_.yaw_rate;
        const double raw_yaw_acceleration =
            (filtered_yaw_speed_ - previous_yaw_speed_) / control_dt;
        const double yaw_acceleration_alpha = 1.0 - std::exp(
            -control_dt / kYawAccelerationFilterTimeConstant);
        filtered_yaw_acceleration_ += yaw_acceleration_alpha *
            (raw_yaw_acceleration - filtered_yaw_acceleration_);
        previous_yaw_speed_ = filtered_yaw_speed_;
        // The second wheel joint axis is mirrored.  Positive world yaw therefore
        // corresponds to a negative r*(omega_1 + omega_2) measurement.
        telemetry_.target_wheel_speed_difference =
            -track_width * coordinated_yaw_rate_;
        telemetry_.wheel_speed_difference =
            filtered_wheel_speed_difference_;
        const double yaw_drive_torque =
            kYawRateKp * telemetry_.yaw_rate_error -
            kYawAccelerationKd * filtered_yaw_acceleration_;
        yaw_braking_request =
            std::fabs(telemetry_.yaw_rate) > 0.02 &&
            yaw_drive_torque * telemetry_.yaw_rate < 0.0;
        // Keep wheel-speed difference as slip/kinematics telemetry only.  The
        // yaw-rate loop follows the SPR structure and closes exclusively on
        // IMU wz, avoiding wheel-contact and mirrored-axis errors in feedback.
        // The coordinator shapes the target before the PD. A square-root
        // safety gate remains on the resulting torque: it is mild at partial
        // authority, but removes braking torque completely at a hard limit.
        // Safety authority suppresses torque that would add yaw energy.  Keep
        // counter-yaw braking available when roll/split authority collapses;
        // otherwise the robot coasts through the safety boundary and tips.
        const double yaw_drive_authority = yaw_braking_request
            ? 1.0
            : std::sqrt(telemetry_.yaw_authority_scale);
        telemetry_.requested_yaw_torque =
            yaw_drive_authority * yaw_drive_torque;
        total_yaw_torque = Clamp(telemetry_.requested_yaw_torque,
                                 -kTotalYawTorqueLimit,
                                 kTotalYawTorqueLimit);
        telemetry_.applied_yaw_torque = total_yaw_torque;
      }

      // Jointly allocate the common balance torque and differential yaw
      // torque.  Spin mode reserves only what the current yaw request needs,
      // capped at 2 N.m/side; unsafe yaw posture immediately returns that
      // capacity to sagittal balance.
      const double demanded_yaw_reserve_per_wheel = Clamp(
          0.5 * kDecoupledYawWheelInputScale *
                  std::fmax(std::fabs(total_yaw_torque),
                            std::fabs(commanded_yaw_torque_)) +
              kSpinYawReserveBufferPerWheel,
          0.0, kSpinYawReservePerWheel);
      telemetry_.reserved_yaw_torque_per_wheel =
          demanded_yaw_reserve_per_wheel * telemetry_.spin_mode_blend *
          (yaw_braking_request ? 1.0 : telemetry_.yaw_authority_scale);
      const double balance_torque_limit = 2.0 * std::fmax(
          0.0, kPerWheelTorqueLimit -
                   telemetry_.reserved_yaw_torque_per_wheel);
      const double requested_balance_torque = total_wheel_torque;
      total_wheel_torque = Clamp(total_wheel_torque,
                                 -balance_torque_limit,
                                 balance_torque_limit);
      telemetry_.balance_torque_authority =
          std::fabs(requested_balance_torque) > 1e-9
              ? std::fabs(total_wheel_torque / requested_balance_torque)
              : 1.0;
      const double common_wheel_demand =
          0.5 * std::fabs(total_wheel_torque);
      telemetry_.wheel_torque_margin = std::fmax(
          0.0, kPerWheelTorqueLimit - common_wheel_demand);
      telemetry_.yaw_torque_authority = RiseAuthority(
          telemetry_.wheel_torque_margin,
          kYawTorqueMarginHard, kYawTorqueMarginSoft);
    }
    telemetry_.requested_leg_angle_torque = total_leg_angle_torque;
    total_wheel_torque = Clamp(total_wheel_torque, -kTotalWheelTorqueLimit,
                               kTotalWheelTorqueLimit);
    total_leg_angle_torque =
        Clamp(total_leg_angle_torque, -kTotalLegAngleTorqueLimit,
              kTotalLegAngleTorqueLimit);
    telemetry_.applied_wheel_torque = total_wheel_torque;
    telemetry_.applied_leg_angle_torque = total_leg_angle_torque;
  }

  const bool yaw_command_released = std::fabs(target_yaw_rate_) < 1e-5;
  const bool yaw_command_reversed =
      target_yaw_rate_ * commanded_yaw_torque_ < 0.0;
  const bool yaw_torque_braking =
      yaw_command_reversed ||
      (yaw_command_released &&
       std::fabs(total_yaw_torque) < std::fabs(commanded_yaw_torque_));
  commanded_yaw_torque_ = MoveTowards(
      commanded_yaw_torque_, total_yaw_torque,
      (yaw_torque_braking ? kYawTorqueBrakeRate : kYawTorqueRiseRate) *
          control_dt);
  if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst ||
      contact_safety_state_ == WbrContactSafetyState::kSingleSupportSecond ||
      contact_safety_state_ == WbrContactSafetyState::kAirborne) {
    commanded_yaw_torque_ = 0.0;
  }
  total_yaw_torque = commanded_yaw_torque_;

  differential_leg_angle_torque +=
      kDecoupledYawLegTorquePerCommand * total_yaw_torque;
  differential_leg_angle_torque = Clamp(
      differential_leg_angle_torque, -kDifferentialLegAngleTorqueLimit,
      kDifferentialLegAngleTorqueLimit);
  commanded_differential_leg_angle_torque_ = MoveTowards(
      commanded_differential_leg_angle_torque_, differential_leg_angle_torque,
      kDifferentialLegAngleTorqueSlewRate * control_dt);
  differential_leg_angle_torque = commanded_differential_leg_angle_torque_;
  if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst ||
      contact_safety_state_ == WbrContactSafetyState::kSingleSupportSecond ||
      contact_safety_state_ == WbrContactSafetyState::kAirborne) {
    commanded_differential_leg_angle_torque_ = 0.0;
    differential_leg_angle_torque = 0.0;
  }
  telemetry_.differential_leg_angle_torque = differential_leg_angle_torque;

  // The LQR's Tp uses the paper's body-on-leg torque convention.  The VMC
  // below applies leg-on-body generalized force, which is the opposite side
  // of that action/reaction pair (the identification path uses the same
  // conversion in ApplyLegController).  Convert conventions before J^T F.
  const double common_leg_angle_torque = -0.5 * total_leg_angle_torque;
  const double first_leg_angle_torque =
      common_leg_angle_torque + 0.5 * differential_leg_angle_torque;
  const double second_leg_angle_torque =
      common_leg_angle_torque - 0.5 * differential_leg_angle_torque;
  double first_leg_target_length = commanded_leg_length_;
  double second_leg_target_length = commanded_leg_length_;
  double desired_contact_leg_offset[2] = {0.0, 0.0};
  if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst) {
    desired_contact_leg_offset[0] = -kGroundedLegYield;
    desired_contact_leg_offset[1] = kAirborneLegSearchExtension;
  } else if (contact_safety_state_ ==
             WbrContactSafetyState::kSingleSupportSecond) {
    desired_contact_leg_offset[0] = kAirborneLegSearchExtension;
    desired_contact_leg_offset[1] = -kGroundedLegYield;
  }
  for (int leg = 0; leg < 2; ++leg) {
    contact_leg_length_offset_[leg] = MoveTowards(
        contact_leg_length_offset_[leg], desired_contact_leg_offset[leg],
        kContactLegOffsetSlewRate * control_dt);
  }
  first_leg_target_length = Clamp(
      commanded_leg_length_ + contact_leg_length_offset_[0],
      kMinTargetLegLength, kTargetHRadius);
  second_leg_target_length = Clamp(
      commanded_leg_length_ + contact_leg_length_offset_[1],
      kMinTargetLegLength, kTargetHRadius);
  if (first_leg_valid) {
    telemetry_.axial_force[0] =
        ApplyLegVmc(data, act_q_b_, act_q_d_, leg_1, first_leg_target_length,
                    support_feedforward_1, telemetry_.integral_force[0],
                    first_leg_angle_torque, 1.0,
                    leg_speed_1_);
  } else if (first_leg_available) {
    data->ctrl[act_q_b_] = 0.0;
    data->ctrl[act_q_d_] = 0.0;
  }
  if (second_leg_valid) {
    telemetry_.axial_force[1] =
        ApplyLegVmc(data, act_q_b_2_, act_q_d_2_, leg_2,
                    second_leg_target_length, support_feedforward_2,
                    telemetry_.integral_force[1], second_leg_angle_torque, -1.0,
                    leg_speed_2_);
  } else if (second_leg_available) {
    data->ctrl[act_q_b_2_] = 0.0;
    data->ctrl[act_q_d_2_] = 0.0;
  }

  if (act_wheel_ >= 0 && act_wheel_2_ >= 0) {
    // The upstream allocator has already reserved yaw headroom in spin mode;
    // this final guard enforces the physical per-wheel actuator limit.
    const double common_wheel_torque = 0.5 * total_wheel_torque;
    const double yaw_headroom =
        std::fmax(0.0, kPerWheelTorqueLimit - std::fabs(common_wheel_torque));
    // The mirrored wheel joint axes make positive actuator differential
    // produce negative world yaw. Convert the controller's world-yaw torque
    // convention to the actuator convention here.
    const double per_wheel_yaw_torque = Clamp(
        -0.5 * kDecoupledYawWheelInputScale * total_yaw_torque,
        -yaw_headroom, yaw_headroom);
    telemetry_.applied_yaw_torque = -2.0 * per_wheel_yaw_torque;
    if (!effective_wheel_grounded[0] && !effective_wheel_grounded[1]) {
      data->ctrl[act_wheel_] = 0.0;
      data->ctrl[act_wheel_2_] = 0.0;
      telemetry_.applied_yaw_torque = 0.0;
    } else if (effective_wheel_grounded[0] &&
               !effective_wheel_grounded[1]) {
      const double grounded_torque =
          contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst
              ? total_wheel_torque
              : common_wheel_torque;
      data->ctrl[act_wheel_] = Clamp(
          grounded_torque, -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
      data->ctrl[act_wheel_2_] = 0.0;
      telemetry_.applied_yaw_torque = 0.0;
    } else if (!effective_wheel_grounded[0] &&
               effective_wheel_grounded[1]) {
      const double grounded_torque =
          contact_safety_state_ == WbrContactSafetyState::kSingleSupportSecond
              ? total_wheel_torque
              : common_wheel_torque;
      data->ctrl[act_wheel_] = 0.0;
      data->ctrl[act_wheel_2_] = Clamp(
          -grounded_torque, -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
      telemetry_.applied_yaw_torque = 0.0;
    } else if (contact_safety_state_ == WbrContactSafetyState::kSingleSupportFirst) {
      data->ctrl[act_wheel_] = Clamp(
          total_wheel_torque, -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
      data->ctrl[act_wheel_2_] = 0.0;
      telemetry_.applied_yaw_torque = 0.0;
    } else if (contact_safety_state_ ==
               WbrContactSafetyState::kSingleSupportSecond) {
      data->ctrl[act_wheel_] = 0.0;
      data->ctrl[act_wheel_2_] = Clamp(
          -total_wheel_torque, -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
      telemetry_.applied_yaw_torque = 0.0;
    } else if (contact_safety_state_ == WbrContactSafetyState::kAirborne) {
      data->ctrl[act_wheel_] = 0.0;
      data->ctrl[act_wheel_2_] = 0.0;
      telemetry_.applied_yaw_torque = 0.0;
    } else {
      data->ctrl[act_wheel_] =
          Clamp(common_wheel_torque + per_wheel_yaw_torque,
                -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
      data->ctrl[act_wheel_2_] =
          Clamp(-common_wheel_torque + per_wheel_yaw_torque,
                -kPerWheelTorqueLimit, kPerWheelTorqueLimit);
    }
  }
  last_time_ = data->time;
}

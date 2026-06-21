#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <mujoco/mujoco.h>

namespace {
constexpr double kLengthAB = 0.0945;
constexpr double kLengthBC = 0.1125;
constexpr double kLengthCD = 0.116;
constexpr double kLengthAD = 0.090;
constexpr double kLengthAG = 0.210;
constexpr double kLengthGH = 0.250;
constexpr double kJacobianStep = 1e-6;
constexpr double kProbeDuration = 0.02;
constexpr double kNominalSupportForcePerLeg = 34.0;

constexpr double kPaperB[3][2] = {
    {-15.1389, 13.8563},  // ddtheta
    {2.1208, -0.7158},    // ddx
    {-4.2238, 16.8001},   // ddphi
};

struct Ids {
  int root;
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

struct Response {
  double ddtheta;
  double ddx;
  double ddphi;
};

struct LateralResponse {
  double ddroll;
  double ddyaw;
  double ddy;
  double ddx;
  double ddangle_difference;
};

struct RollResponse {
  double ddroll;
  double ddpitch;
  double ddyaw;
  double ddz;
};

struct DifferentialLegResponse {
  double ddangle_difference;
  double ddroll;
  double ddyaw;
  double ddx;
  double ddy;
};

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

bool NumericalJacobian(double phi1, double phi2, double jacobian[2][2]) {
  double hx_plus;
  double hz_plus;
  double hx_minus;
  double hz_minus;
  if (!ForwardKinematics(phi1 + kJacobianStep, phi2, 1, hx_plus, hz_plus) ||
      !ForwardKinematics(phi1 - kJacobianStep, phi2, 1, hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][0] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][0] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  if (!ForwardKinematics(phi1, phi2 + kJacobianStep, 1, hx_plus, hz_plus) ||
      !ForwardKinematics(phi1, phi2 - kJacobianStep, 1, hx_minus, hz_minus)) {
    return false;
  }
  jacobian[0][1] = (hx_plus - hx_minus) / (2.0 * kJacobianStep);
  jacobian[1][1] = (hz_plus - hz_minus) / (2.0 * kJacobianStep);
  return true;
}

bool FindIds(const mjModel* model, Ids& ids) {
  ids.root = mj_name2id(model, mjOBJ_JOINT, "root_free");
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

  const int values[] = {
      ids.root, ids.joint_b[0], ids.joint_b[1], ids.joint_d[0], ids.joint_d[1],
      ids.actuator_b[0], ids.actuator_b[1], ids.actuator_d[0], ids.actuator_d[1],
      ids.actuator_wheel[0], ids.actuator_wheel[1], ids.body_a[0], ids.body_a[1],
      ids.body_wheel[0], ids.body_wheel[1], ids.body_plate,
      ids.geom_wheel[0], ids.geom_wheel[1],
  };
  for (int value : values) {
    if (value < 0) {
      return false;
    }
  }
  return true;
}

void ApplyWheelTorque(const Ids& ids, mjData* data, double total_torque) {
  data->ctrl[ids.actuator_wheel[0]] = 0.5 * total_torque;
  data->ctrl[ids.actuator_wheel[1]] = -0.5 * total_torque;
}

bool ApplyLegAngleTorque(const mjModel* model, const Ids& ids, mjData* data,
                         double total_torque) {
  const double torque_per_leg = 0.5 * total_torque;
  for (int leg = 0; leg < 2; ++leg) {
    const double mirror = leg == 0 ? 1.0 : -1.0;
    const double phi1 = mirror * data->qpos[model->jnt_qposadr[ids.joint_d[leg]]];
    const double phi2 = mirror * data->qpos[model->jnt_qposadr[ids.joint_b[leg]]];
    double hx;
    double hz;
    double jacobian[2][2];
    if (!ForwardKinematics(phi1, phi2, 1, hx, hz) ||
        !NumericalJacobian(phi1, phi2, jacobian)) {
      return false;
    }
    const double length = std::hypot(hx, hz);
    const double tangent_x = -hz / length;
    const double tangent_z = hx / length;
    const double tangential_force = torque_per_leg / length;
    const double force_x = tangential_force * tangent_x;
    const double force_z = tangential_force * tangent_z;
    const double torque_phi1 =
        jacobian[0][0] * force_x + jacobian[1][0] * force_z;
    const double torque_phi2 =
        jacobian[0][1] * force_x + jacobian[1][1] * force_z;
    data->ctrl[ids.actuator_b[leg]] = mirror * torque_phi2;
    data->ctrl[ids.actuator_d[leg]] = mirror * torque_phi1;
  }
  return true;
}

bool ApplyDifferentialAxialForce(const mjModel* model, const Ids& ids,
                                 mjData* data, double differential_force) {
  for (int leg = 0; leg < 2; ++leg) {
    const double mirror = leg == 0 ? 1.0 : -1.0;
    const double axial_force = (leg == 0 ? 0.5 : -0.5) * differential_force;
    const double phi1 = mirror * data->qpos[model->jnt_qposadr[ids.joint_d[leg]]];
    const double phi2 = mirror * data->qpos[model->jnt_qposadr[ids.joint_b[leg]]];
    double hx;
    double hz;
    double jacobian[2][2];
    if (!ForwardKinematics(phi1, phi2, 1, hx, hz) ||
        !NumericalJacobian(phi1, phi2, jacobian)) {
      return false;
    }
    const double length = std::hypot(hx, hz);
    const double force_x = axial_force * hx / length;
    const double force_z = axial_force * hz / length;
    const double torque_phi1 =
        jacobian[0][0] * force_x + jacobian[1][0] * force_z;
    const double torque_phi2 =
        jacobian[0][1] * force_x + jacobian[1][1] * force_z;
    data->ctrl[ids.actuator_b[leg]] = mirror * torque_phi2;
    data->ctrl[ids.actuator_d[leg]] = mirror * torque_phi1;
  }
  return true;
}

bool ApplyDifferentialLegAngleTorque(const mjModel* model, const Ids& ids,
                                     mjData* data,
                                     double differential_torque) {
  for (int leg = 0; leg < 2; ++leg) {
    const double mirror = leg == 0 ? 1.0 : -1.0;
    const double angle_torque =
        (leg == 0 ? 0.5 : -0.5) * differential_torque;
    const double phi1 = mirror * data->qpos[model->jnt_qposadr[ids.joint_d[leg]]];
    const double phi2 = mirror * data->qpos[model->jnt_qposadr[ids.joint_b[leg]]];
    double hx;
    double hz;
    double jacobian[2][2];
    if (!ForwardKinematics(phi1, phi2, 1, hx, hz) ||
        !NumericalJacobian(phi1, phi2, jacobian)) {
      return false;
    }
    const double length = std::hypot(hx, hz);
    const double force_x =
        kNominalSupportForcePerLeg * hx / length +
        angle_torque * (-hz) / (length * length);
    const double force_z =
        kNominalSupportForcePerLeg * hz / length +
        angle_torque * hx / (length * length);
    const double torque_phi1 =
        jacobian[0][0] * force_x + jacobian[1][0] * force_z;
    const double torque_phi2 =
        jacobian[0][1] * force_x + jacobian[1][1] * force_z;
    data->ctrl[ids.actuator_b[leg]] = mirror * torque_phi2;
    data->ctrl[ids.actuator_d[leg]] = mirror * torque_phi1;
  }
  return true;
}

bool LegAngleRate(const mjModel* model, const Ids& ids, const mjData* data,
                  int leg, double& angle_rate) {
  const double mirror = leg == 0 ? 1.0 : -1.0;
  const double phi1 = mirror * data->qpos[model->jnt_qposadr[ids.joint_d[leg]]];
  const double phi2 = mirror * data->qpos[model->jnt_qposadr[ids.joint_b[leg]]];
  const double dphi1 = mirror * data->qvel[model->jnt_dofadr[ids.joint_d[leg]]];
  const double dphi2 = mirror * data->qvel[model->jnt_dofadr[ids.joint_b[leg]]];
  double hx;
  double hz;
  double jacobian[2][2];
  if (!ForwardKinematics(phi1, phi2, 1, hx, hz) ||
      !NumericalJacobian(phi1, phi2, jacobian)) {
    return false;
  }
  const double vx = jacobian[0][0] * dphi1 + jacobian[0][1] * dphi2;
  const double vz = jacobian[1][0] * dphi1 + jacobian[1][1] * dphi2;
  angle_rate = (-hz * vx + hx * vz) / (hx * hx + hz * hz);
  return true;
}

Response MeasureResponse(const mjModel* model, const Ids& ids,
                         const std::vector<mjtNum>& qpos,
                         double total_wheel_torque, double total_leg_torque) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mju_zero(data->ctrl, model->nu);
  ApplyWheelTorque(ids, data, total_wheel_torque);
  if (!ApplyLegAngleTorque(model, ids, data, total_leg_torque)) {
    std::fprintf(stderr, "VMC Jacobian failed at the test state.\n");
    std::exit(2);
  }
  mj_forward(model, data);
  const int steps = static_cast<int>(std::ceil(kProbeDuration / model->opt.timestep));
  for (int step = 0; step < steps; ++step) {
    mj_step(model, data);
  }
  const double duration = steps * model->opt.timestep;

  mjtNum plate_velocity[6];
  mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_plate, plate_velocity, 0);
  Response response{};
  response.ddphi = -plate_velocity[1] / duration;

  for (int leg = 0; leg < 2; ++leg) {
    mjtNum a_velocity[6];
    mjtNum wheel_velocity[6];
    mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_a[leg], a_velocity, 0);
    mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_wheel[leg], wheel_velocity, 0);
    const double rx = data->xpos[3 * ids.body_wheel[leg]] -
                      data->xpos[3 * ids.body_a[leg]];
    const double rz = data->xpos[3 * ids.body_wheel[leg] + 2] -
                      data->xpos[3 * ids.body_a[leg] + 2];
    const double relative_vx = wheel_velocity[3] - a_velocity[3];
    const double relative_vz = wheel_velocity[5] - a_velocity[5];
    response.ddtheta +=
        (-rz * relative_vx + rx * relative_vz) / (rx * rx + rz * rz) / duration;
    response.ddx += wheel_velocity[3] / duration;
  }
  response.ddtheta *= 0.5;
  response.ddx *= 0.5;
  mj_deleteData(data);
  return response;
}

LateralResponse MeasureDifferentialResponse(
    const mjModel* model, const Ids& ids, const std::vector<mjtNum>& qpos,
    double differential_torque) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mju_zero(data->ctrl, model->nu);
  if (!ApplyDifferentialLegAngleTorque(model, ids, data, 0.0)) {
    std::fprintf(stderr, "Loaded leg support setup failed.\n");
    std::exit(2);
  }
  // The wheel joint axes are opposite. Equal actuator signs therefore probe
  // the differential ground-force channel, while opposite signs are the
  // sagittal common-force channel used by the balance LQR.
  data->ctrl[ids.actuator_wheel[0]] = 0.5 * differential_torque;
  data->ctrl[ids.actuator_wheel[1]] = 0.5 * differential_torque;
  mj_forward(model, data);
  const int steps = static_cast<int>(std::ceil(kProbeDuration / model->opt.timestep));
  for (int step = 0; step < steps; ++step) {
    mj_step(model, data);
  }
  const double duration = steps * model->opt.timestep;
  mjtNum plate_velocity[6];
  mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_plate, plate_velocity, 0);
  double angle_rate[2] = {};
  if (!LegAngleRate(model, ids, data, 0, angle_rate[0]) ||
      !LegAngleRate(model, ids, data, 1, angle_rate[1])) {
    std::fprintf(stderr, "Could not measure wheel-induced leg-angle rate.\n");
    std::exit(2);
  }
  const LateralResponse result = {
      plate_velocity[0] / duration,
      plate_velocity[2] / duration,
      plate_velocity[4] / duration,
      plate_velocity[3] / duration,
      (angle_rate[0] - angle_rate[1]) / duration,
  };
  mj_deleteData(data);
  return result;
}

RollResponse MeasureDifferentialLegForceResponse(
    const mjModel* model, const Ids& ids, const std::vector<mjtNum>& qpos,
    double differential_force) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mju_zero(data->ctrl, model->nu);
  if (!ApplyDifferentialAxialForce(model, ids, data, differential_force)) {
    std::fprintf(stderr, "Differential axial-force VMC failed.\n");
    std::exit(2);
  }
  mj_forward(model, data);
  const int steps = static_cast<int>(std::ceil(kProbeDuration / model->opt.timestep));
  for (int step = 0; step < steps; ++step) {
    mj_step(model, data);
  }
  const double duration = steps * model->opt.timestep;
  mjtNum plate_velocity[6];
  mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_plate, plate_velocity, 0);
  const RollResponse result = {
      plate_velocity[0] / duration,
      plate_velocity[1] / duration,
      plate_velocity[2] / duration,
      plate_velocity[5] / duration,
  };
  mj_deleteData(data);
  return result;
}

DifferentialLegResponse MeasureDifferentialLegAngleResponse(
    const mjModel* model, const Ids& ids, const std::vector<mjtNum>& qpos,
    double differential_torque) {
  mjData* data = mj_makeData(model);
  mju_copy(data->qpos, qpos.data(), model->nq);
  mju_zero(data->qvel, model->nv);
  mju_zero(data->ctrl, model->nu);
  if (!ApplyDifferentialLegAngleTorque(model, ids, data,
                                       differential_torque)) {
    std::fprintf(stderr, "Differential leg-angle VMC failed.\n");
    std::exit(2);
  }
  mj_forward(model, data);
  const int steps = static_cast<int>(std::ceil(kProbeDuration / model->opt.timestep));
  for (int step = 0; step < steps; ++step) {
    mj_step(model, data);
  }
  const double duration = steps * model->opt.timestep;
  double angle_rate[2] = {};
  if (!LegAngleRate(model, ids, data, 0, angle_rate[0]) ||
      !LegAngleRate(model, ids, data, 1, angle_rate[1])) {
    std::fprintf(stderr, "Could not measure differential leg-angle rate.\n");
    std::exit(2);
  }
  mjtNum plate_velocity[6];
  mj_objectVelocity(model, data, mjOBJ_BODY, ids.body_plate, plate_velocity, 0);
  const DifferentialLegResponse result = {
      (angle_rate[0] - angle_rate[1]) / duration,
      plate_velocity[0] / duration,
      plate_velocity[2] / duration,
      plate_velocity[3] / duration,
      plate_velocity[4] / duration,
  };
  mj_deleteData(data);
  return result;
}

LateralResponse CentralDifference(const LateralResponse& plus,
                                   const LateralResponse& minus,
                                   double amplitude) {
  return {
      (plus.ddroll - minus.ddroll) / (2.0 * amplitude),
      (plus.ddyaw - minus.ddyaw) / (2.0 * amplitude),
      (plus.ddy - minus.ddy) / (2.0 * amplitude),
      (plus.ddx - minus.ddx) / (2.0 * amplitude),
      (plus.ddangle_difference - minus.ddangle_difference) /
          (2.0 * amplitude),
  };
}

RollResponse CentralDifference(const RollResponse& plus,
                               const RollResponse& minus,
                               double amplitude) {
  return {
      (plus.ddroll - minus.ddroll) / (2.0 * amplitude),
      (plus.ddpitch - minus.ddpitch) / (2.0 * amplitude),
      (plus.ddyaw - minus.ddyaw) / (2.0 * amplitude),
      (plus.ddz - minus.ddz) / (2.0 * amplitude),
  };
}

DifferentialLegResponse CentralDifference(
    const DifferentialLegResponse& plus,
    const DifferentialLegResponse& minus, double amplitude) {
  return {
      (plus.ddangle_difference - minus.ddangle_difference) /
          (2.0 * amplitude),
      (plus.ddroll - minus.ddroll) / (2.0 * amplitude),
      (plus.ddyaw - minus.ddyaw) / (2.0 * amplitude),
      (plus.ddx - minus.ddx) / (2.0 * amplitude),
      (plus.ddy - minus.ddy) / (2.0 * amplitude),
  };
}

Response CentralDifference(const Response& plus, const Response& minus,
                           double amplitude) {
  return {
      (plus.ddtheta - minus.ddtheta) / (2.0 * amplitude),
      (plus.ddx - minus.ddx) / (2.0 * amplitude),
      (plus.ddphi - minus.ddphi) / (2.0 * amplitude),
  };
}

Response ToPaperCoordinates(const Response& current) {
  return {-current.ddtheta, current.ddx, -current.ddphi};
}

const char* SignResult(double measured, double expected) {
  if (std::fabs(measured) < 1e-6) {
    return "ZERO";
  }
  return std::signbit(measured) == std::signbit(expected) ? "MATCH" : "MISMATCH";
}

void PrintColumn(const char* input, const Response& response, int column) {
  const double values[3] = {response.ddtheta, response.ddx, response.ddphi};
  const char* names[3] = {"ddtheta", "ddx", "ddphi"};
  std::printf("\n%s response per 1 N.m of total input:\n", input);
  std::printf("  state       measured       paper B       sign\n");
  for (int row = 0; row < 3; ++row) {
    std::printf("  %-8s % 12.6f  % 12.6f   %s\n", names[row], values[row],
                kPaperB[row][column], SignResult(values[row], kPaperB[row][column]));
  }
}
}  // namespace

int main(int argc, char** argv) {
  const char* model_path = argc > 1 ? argv[1] : "wbr_free.xml";
  const double amplitude = argc > 2 ? std::atof(argv[2]) : 0.2;
  const double roll_force_amplitude = argc > 3 ? std::atof(argv[3]) : 5.0;
  if (amplitude <= 0.0) {
    std::fprintf(stderr, "Input amplitude must be positive.\n");
    return 1;
  }
  if (roll_force_amplitude <= 0.0) {
    std::fprintf(stderr, "Roll force amplitude must be positive.\n");
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
    std::fprintf(stderr, "Required WBR joints, bodies, geoms, or actuators are missing.\n");
    mj_deleteModel(model);
    return 1;
  }

  mjData* initial = mj_makeData(model);
  const int key = mj_name2id(model, mjOBJ_KEY, "init");
  if (key >= 0) {
    mj_resetDataKeyframe(model, initial, key);
  }
  mju_zero(initial->qvel, model->nv);
  mj_forward(model, initial);

  double wheel_bottom = 1e9;
  for (int leg = 0; leg < 2; ++leg) {
    const double center_z = initial->geom_xpos[3 * ids.geom_wheel[leg] + 2];
    const double radius = model->geom_size[3 * ids.geom_wheel[leg]];
    wheel_bottom = std::fmin(wheel_bottom, center_z - radius);
  }
  const int root_qpos = model->jnt_qposadr[ids.root];
  initial->qpos[root_qpos + 2] -= wheel_bottom + 1e-5;
  mj_forward(model, initial);
  std::vector<mjtNum> qpos(initial->qpos, initial->qpos + model->nq);

  std::printf("Model: %s\n", model_path);
  std::printf("Central-difference amplitude: %.6f N.m\n", amplitude);
  std::printf("Open-loop pulse duration: %.3f s\n", kProbeDuration);
  std::printf("Initial contacts after placing wheels on floor: %d\n", initial->ncon);
  for (int contact_id = 0; contact_id < initial->ncon; ++contact_id) {
    const mjContact& contact = initial->contact[contact_id];
    const bool wheel_contact =
        contact.geom[0] == ids.geom_wheel[0] ||
        contact.geom[1] == ids.geom_wheel[0] ||
        contact.geom[0] == ids.geom_wheel[1] ||
        contact.geom[1] == ids.geom_wheel[1];
    if (!wheel_contact) continue;
    std::printf("  contact %d tangent1=(%+.3f,%+.3f,%+.3f) "
                "tangent2=(%+.3f,%+.3f,%+.3f) friction=(%.3f,%.3f)\n",
                contact_id, contact.frame[3], contact.frame[4], contact.frame[5],
                contact.frame[6], contact.frame[7], contact.frame[8],
                contact.friction[0], contact.friction[1]);
  }
  std::printf("This test compares input signs. Magnitudes are only indicative because\n"
              "the XML initial leg length and physical parameters differ from the paper.\n");

  const Response t_plus =
      ToPaperCoordinates(MeasureResponse(model, ids, qpos, amplitude, 0.0));
  const Response t_minus =
      ToPaperCoordinates(MeasureResponse(model, ids, qpos, -amplitude, 0.0));
  // Positive paper Tp is negative in the counterclockwise-positive VMC coordinates.
  const Response tp_plus =
      ToPaperCoordinates(MeasureResponse(model, ids, qpos, 0.0, -amplitude));
  const Response tp_minus =
      ToPaperCoordinates(MeasureResponse(model, ids, qpos, 0.0, amplitude));
  PrintColumn("paper T", CentralDifference(t_plus, t_minus, amplitude), 0);
  PrintColumn("paper Tp", CentralDifference(tp_plus, tp_minus, amplitude), 1);

  if (model->jnt_type[ids.root] == mjJNT_FREE) {
    const LateralResponse differential_plus =
        MeasureDifferentialResponse(model, ids, qpos, amplitude);
    const LateralResponse differential_minus =
        MeasureDifferentialResponse(model, ids, qpos, -amplitude);
    const LateralResponse differential = CentralDifference(
        differential_plus, differential_minus, amplitude);
    std::printf("\n3D differential wheel response per 1 N.m total input\n");
    std::printf("  actuator pattern  = [+0.5 u, +0.5 u]\n");
    std::printf("  ddroll            = %+12.6f rad/s^2\n", differential.ddroll);
    std::printf("  ddyaw             = %+12.6f rad/s^2\n", differential.ddyaw);
    std::printf("  ddy               = %+12.6f m/s^2\n", differential.ddy);
    std::printf("  ddx leakage       = %+12.6f m/s^2\n", differential.ddx);
    std::printf("  dd(angle1-angle2) = %+12.6f rad/s^2\n",
                differential.ddangle_difference);

    const RollResponse roll_plus = MeasureDifferentialLegForceResponse(
        model, ids, qpos, roll_force_amplitude);
    const RollResponse roll_minus = MeasureDifferentialLegForceResponse(
        model, ids, qpos, -roll_force_amplitude);
    const RollResponse roll =
        CentralDifference(roll_plus, roll_minus, roll_force_amplitude);
    std::printf("\n3D differential leg-force response per 1 N total input\n");
    std::printf("  axial pattern     = [+0.5 u, -0.5 u]\n");
    std::printf("  ddroll            = %+12.6f rad/s^2\n", roll.ddroll);
    std::printf("  ddpitch leakage   = %+12.6f rad/s^2\n", roll.ddpitch);
    std::printf("  ddyaw leakage     = %+12.6f rad/s^2\n", roll.ddyaw);
    std::printf("  ddz leakage       = %+12.6f m/s^2\n", roll.ddz);

    const DifferentialLegResponse leg_angle_plus =
        MeasureDifferentialLegAngleResponse(model, ids, qpos, amplitude);
    const DifferentialLegResponse leg_angle_minus =
        MeasureDifferentialLegAngleResponse(model, ids, qpos, -amplitude);
    const DifferentialLegResponse leg_angle = CentralDifference(
        leg_angle_plus, leg_angle_minus, amplitude);
    std::printf("\n3D differential leg-angle response per 1 N.m total input\n");
    std::printf("  VMC pattern       = [+0.5 u, -0.5 u]\n");
    std::printf("  dd(angle1-angle2) = %+12.6f rad/s^2\n",
                leg_angle.ddangle_difference);
    std::printf("  ddroll leakage    = %+12.6f rad/s^2\n", leg_angle.ddroll);
    std::printf("  ddyaw leakage     = %+12.6f rad/s^2\n", leg_angle.ddyaw);
    std::printf("  ddx leakage       = %+12.6f m/s^2\n", leg_angle.ddx);
    std::printf("  ddy leakage       = %+12.6f m/s^2\n", leg_angle.ddy);
    const double determinant =
        differential.ddyaw * leg_angle.ddangle_difference -
        leg_angle.ddyaw * differential.ddangle_difference;
    std::printf("\nYaw/leg-angle input matrix B\n");
    std::printf("  [ddyaw]   = [%+10.6f %+10.6f] [Tyaw]\n",
                differential.ddyaw, leg_angle.ddyaw);
    std::printf("  [ddsplit]   [%+10.6f %+10.6f] [Tsplit]\n",
                differential.ddangle_difference,
                leg_angle.ddangle_difference);
    std::printf("  determinant       = %+12.6f\n", determinant);
  }

  mj_deleteData(initial);
  mj_deleteModel(model);
  return 0;
}

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

namespace {

constexpr int kReducedState = 6;
constexpr int kReducedInput = 2;
constexpr double kNominalLegLength = 0.18;
constexpr double kWheelRadius = 0.05;
double g_leg_length = kNominalLegLength;

int JointDof(const mjModel* model, const char* name) {
  const int id = mj_name2id(model, mjOBJ_JOINT, name);
  return id >= 0 ? model->jnt_dofadr[id] : -1;
}

int JointQpos(const mjModel* model, const char* name) {
  const int id = mj_name2id(model, mjOBJ_JOINT, name);
  return id >= 0 ? model->jnt_qposadr[id] : -1;
}

int Actuator(const mjModel* model, const char* name) {
  return mj_name2id(model, mjOBJ_ACTUATOR, name);
}

void PrintMatrix(const char* name, const std::vector<double>& matrix,
                 int rows, int cols) {
  std::printf("%s =\n", name);
  for (int row = 0; row < rows; ++row) {
    std::printf("  ");
    for (int col = 0; col < cols; ++col) {
      std::printf("% .10g%s", matrix[row * cols + col],
                  col + 1 == cols ? "" : "  ");
    }
    std::printf("\n");
  }
}

std::vector<double> Multiply(const std::vector<double>& left, int left_rows,
                             int inner, const std::vector<double>& right,
                             int right_cols) {
  std::vector<double> result(left_rows * right_cols, 0.0);
  for (int row = 0; row < left_rows; ++row) {
    for (int k = 0; k < inner; ++k) {
      for (int col = 0; col < right_cols; ++col) {
        result[row * right_cols + col] +=
            left[row * inner + k] * right[k * right_cols + col];
      }
    }
  }
  return result;
}

int MatrixRank(std::vector<double> matrix, int rows, int cols,
               double tolerance) {
  int rank = 0;
  for (int col = 0; col < cols && rank < rows; ++col) {
    int pivot = rank;
    for (int row = rank + 1; row < rows; ++row) {
      if (std::abs(matrix[row * cols + col]) >
          std::abs(matrix[pivot * cols + col])) {
        pivot = row;
      }
    }
    if (std::abs(matrix[pivot * cols + col]) <= tolerance) {
      continue;
    }
    for (int k = col; k < cols; ++k) {
      std::swap(matrix[rank * cols + k], matrix[pivot * cols + k]);
    }
    const double divisor = matrix[rank * cols + col];
    for (int k = col; k < cols; ++k) {
      matrix[rank * cols + k] /= divisor;
    }
    for (int row = 0; row < rows; ++row) {
      if (row == rank) {
        continue;
      }
      const double factor = matrix[row * cols + col];
      for (int k = col; k < cols; ++k) {
        matrix[row * cols + k] -= factor * matrix[rank * cols + k];
      }
    }
    ++rank;
  }
  return rank;
}

std::vector<double> Transpose(const std::vector<double>& matrix, int rows,
                              int cols) {
  std::vector<double> result(cols * rows, 0.0);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      result[col * rows + row] = matrix[row * cols + col];
    }
  }
  return result;
}

std::vector<double> SolveDare(const std::vector<double>& a,
                              const std::vector<double>& b) {
  const std::array<double, kReducedState> q = {1, 1, 500, 100, 5000, 1};
  const std::array<double, kReducedInput> r = {1, 0.25};
  std::vector<double> p(kReducedState * kReducedState, 0.0);
  for (int i = 0; i < kReducedState; ++i) {
    p[i * kReducedState + i] = q[i];
  }
  const auto at = Transpose(a, kReducedState, kReducedState);
  const auto bt = Transpose(b, kReducedState, kReducedInput);

  for (int iteration = 0; iteration < 200000; ++iteration) {
    const auto pb = Multiply(p, kReducedState, kReducedState, b,
                             kReducedInput);
    const auto bt_pb = Multiply(bt, kReducedInput, kReducedState, pb,
                                kReducedInput);
    const double s00 = r[0] + bt_pb[0];
    const double s01 = bt_pb[1];
    const double s10 = bt_pb[2];
    const double s11 = r[1] + bt_pb[3];
    const double determinant = s00 * s11 - s01 * s10;
    const std::vector<double> s_inverse = {
        s11 / determinant, -s01 / determinant,
        -s10 / determinant, s00 / determinant};
    const auto pa = Multiply(p, kReducedState, kReducedState, a,
                             kReducedState);
    const auto bt_pa = Multiply(bt, kReducedInput, kReducedState, pa,
                                kReducedState);
    const auto gain = Multiply(s_inverse, kReducedInput, kReducedInput,
                               bt_pa, kReducedState);
    const auto at_pa = Multiply(at, kReducedState, kReducedState, pa,
                                kReducedState);
    const auto at_pb = Multiply(at, kReducedState, kReducedState, pb,
                                kReducedInput);
    const auto correction = Multiply(at_pb, kReducedState, kReducedInput,
                                     gain, kReducedState);
    std::vector<double> next = at_pa;
    double max_change = 0.0;
    for (int row = 0; row < kReducedState; ++row) {
      for (int col = 0; col < kReducedState; ++col) {
        const int index = row * kReducedState + col;
        next[index] -= correction[index];
        if (row == col) {
          next[index] += q[row];
        }
        max_change = std::max(max_change, std::abs(next[index] - p[index]));
      }
    }
    p = std::move(next);
    if (max_change < 1e-10) {
      break;
    }
  }

  const auto pb = Multiply(p, kReducedState, kReducedState, b,
                           kReducedInput);
  const auto bt_pb = Multiply(bt, kReducedInput, kReducedState, pb,
                              kReducedInput);
  const double s00 = r[0] + bt_pb[0];
  const double s01 = bt_pb[1];
  const double s10 = bt_pb[2];
  const double s11 = r[1] + bt_pb[3];
  const double determinant = s00 * s11 - s01 * s10;
  const std::vector<double> s_inverse = {
      s11 / determinant, -s01 / determinant,
      -s10 / determinant, s00 / determinant};
  const auto pa = Multiply(p, kReducedState, kReducedState, a,
                           kReducedState);
  const auto bt_pa = Multiply(bt, kReducedInput, kReducedState, pa,
                              kReducedState);
  return Multiply(s_inverse, kReducedInput, kReducedInput, bt_pa,
                  kReducedState);
}

std::array<double, kReducedState> MeasureState(
    const mjModel* model, const mjData* data,
    const std::vector<double>& nominal_qpos, int root_x, int root_pitch,
    int leg_1, int leg_2) {
  const double phi = data->qpos[root_pitch] - nominal_qpos[root_pitch];
  const double theta = phi + 0.5 *
      ((data->qpos[leg_1] - nominal_qpos[leg_1]) +
       (data->qpos[leg_2] - nominal_qpos[leg_2]));
  const double dphi = data->qvel[root_pitch];
  const double dtheta = dphi + 0.5 *
      (data->qvel[leg_1] + data->qvel[leg_2]);
  const double x = data->qpos[root_x] - nominal_qpos[root_x] -
                   g_leg_length * theta;
  const double dx = data->qvel[root_x] - g_leg_length * dtheta;
  return {theta, dtheta, x, dx, phi, dphi};
}

void ValidateClosedLoop(const mjModel* model,
                        const std::vector<double>& nominal_qpos,
                        const std::vector<double>& gain, int root_x,
                        int root_pitch, int leg_1, int leg_2,
                        const std::array<double, 3>& perturbation,
                        const char* name) {
  mjData* data = mj_makeData(model);
  mj_resetDataKeyframe(model, data, 0);
  mju_copy(data->qpos, nominal_qpos.data(), model->nq);
  const double theta = perturbation[0];
  const double x = perturbation[1];
  const double phi = perturbation[2];
  data->qpos[root_x] += x + g_leg_length * theta;
  data->qpos[root_pitch] += phi;
  data->qpos[leg_1] += theta - phi;
  data->qpos[leg_2] += theta - phi;
  mj_forward(model, data);

  const int wheel_motor_1 = Actuator(model, "wheel_motor_1");
  const int wheel_motor_2 = Actuator(model, "wheel_motor_2");
  const int hip_motor_1 = Actuator(model, "hip_motor_1");
  const int hip_motor_2 = Actuator(model, "hip_motor_2");
  double max_wheel_torque = 0.0;
  double max_hip_torque = 0.0;
  bool finite = true;
  while (data->time < 2.0) {
    const auto state = MeasureState(model, data, nominal_qpos, root_x,
                                    root_pitch, leg_1, leg_2);
    double input[kReducedInput] = {};
    for (int row = 0; row < kReducedInput; ++row) {
      for (int col = 0; col < kReducedState; ++col) {
        input[row] -= gain[row * kReducedState + col] * state[col];
      }
    }
    input[0] = std::clamp(input[0], -10.0, 10.0);
    input[1] = std::clamp(input[1], -40.0, 40.0);
    data->ctrl[wheel_motor_1] = 0.5 * input[0];
    data->ctrl[wheel_motor_2] = 0.5 * input[0];
    data->ctrl[hip_motor_1] = 0.5 * input[1];
    data->ctrl[hip_motor_2] = 0.5 * input[1];
    max_wheel_torque = std::max(max_wheel_torque, std::abs(input[0]));
    max_hip_torque = std::max(max_hip_torque, std::abs(input[1]));
    mj_step(model, data);
    finite = finite && !mju_isBad(data->qpos[root_x]) &&
             !mju_isBad(data->qpos[root_pitch]);
    if (!finite) {
      break;
    }
  }
  const auto final_state = MeasureState(model, data, nominal_qpos, root_x,
                                        root_pitch, leg_1, leg_2);
  std::printf("  %-6s finite=%s  max|T|=%7.3f max|Tp|=%7.3f  "
              "final(theta,x,phi)=(% .6g,% .6g,% .6g)\n",
              name, finite ? "yes" : "no", max_wheel_torque,
              max_hip_torque, final_state[0], final_state[2], final_state[4]);
  mj_deleteData(data);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string path = argc > 1 ? argv[1] : "wbr_reduced.xml";
  if (argc > 2) {
    g_leg_length = std::atof(argv[2]);
  }
  if (g_leg_length < 0.10 || g_leg_length > 0.40) {
    std::fprintf(stderr, "Leg length must be in [0.10, 0.40] m.\n");
    return 1;
  }
  char error[1024] = {};
  mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Failed to load %s: %s\n", path.c_str(), error);
    return 1;
  }
  const double length_scale = g_leg_length / kNominalLegLength;
  for (int leg = 1; leg <= 2; ++leg) {
    const std::string suffix = std::to_string(leg);
    const int leg_body = mj_name2id(
        model, mjOBJ_BODY, ("leg_" + suffix).c_str());
    const int wheel_body = mj_name2id(
        model, mjOBJ_BODY, ("wheel_" + suffix).c_str());
    const int leg_geom = mj_name2id(
        model, mjOBJ_GEOM, ("leg_geom_" + suffix).c_str());
    if (leg_body < 0 || wheel_body < 0 || leg_geom < 0) {
      std::fprintf(stderr, "Reduced model is missing leg geometry %d.\n", leg);
      mj_deleteModel(model);
      return 1;
    }
    model->body_pos[3 * wheel_body + 2] = -g_leg_length;
    model->body_ipos[3 * leg_body + 2] = -0.5 * g_leg_length;
    model->body_inertia[3 * leg_body] *= length_scale * length_scale;
    model->body_inertia[3 * leg_body + 1] *= length_scale * length_scale;
    model->geom_pos[3 * leg_geom + 2] = -0.5 * g_leg_length;
    model->geom_size[3 * leg_geom + 1] = 0.5 * g_leg_length;
  }
  mjData* data = mj_makeData(model);
  if (!data) {
    mj_deleteModel(model);
    return 1;
  }

  const int root_x = JointDof(model, "root_x");
  const int root_z_qpos = JointQpos(model, "root_z");
  const int root_pitch = JointDof(model, "root_pitch");
  const int leg_1 = JointDof(model, "leg_theta_1");
  const int wheel_1 = JointDof(model, "wheel_joint_1");
  const int leg_2 = JointDof(model, "leg_theta_2");
  const int wheel_2 = JointDof(model, "wheel_joint_2");
  const std::array<int, 7> required = {
      root_x, root_z_qpos, root_pitch, leg_1, wheel_1, leg_2, wheel_2};
  if (std::any_of(required.begin(), required.end(), [](int id) { return id < 0; })) {
    std::fprintf(stderr, "Reduced model is missing a required joint.\n");
    mj_deleteData(data);
    mj_deleteModel(model);
    return 1;
  }

  mj_resetDataKeyframe(model, data, 0);
  const int platform_body = mj_name2id(model, mjOBJ_BODY, "platform");
  data->qpos[root_z_qpos] =
      g_leg_length + kWheelRadius - model->body_pos[3 * platform_body + 2];
  mj_forward(model, data);
  for (int i = 0; i < 500; ++i) {
    mj_step(model, data);
  }
  mj_forward(model, data);

  const int full_state = 2 * model->nv + model->na;
  std::vector<double> full_a(full_state * full_state, 0.0);
  std::vector<double> full_b(full_state * model->nu, 0.0);
  mjd_transitionFD(model, data, 1e-6, 1, full_a.data(), full_b.data(),
                   nullptr, nullptr);

  // G lifts [theta,dtheta,x,dx,phi,dphi] into the symmetric full state.
  std::vector<double> lift(full_state * kReducedState, 0.0);
  auto set_lift = [&](int full_row, int reduced_col, double value) {
    lift[full_row * kReducedState + reduced_col] = value;
  };
  set_lift(root_x, 0, g_leg_length);
  set_lift(root_x, 2, 1.0);
  set_lift(root_pitch, 4, 1.0);
  set_lift(leg_1, 0, 1.0);
  set_lift(leg_1, 4, -1.0);
  set_lift(leg_2, 0, 1.0);
  set_lift(leg_2, 4, -1.0);
  // Wheel rotation is chosen consistently with positive forward rolling.
  set_lift(wheel_1, 2, 1.0 / kWheelRadius);
  set_lift(wheel_2, 2, 1.0 / kWheelRadius);

  const int velocity_offset = model->nv;
  set_lift(velocity_offset + root_x, 1, g_leg_length);
  set_lift(velocity_offset + root_x, 3, 1.0);
  set_lift(velocity_offset + root_pitch, 5, 1.0);
  set_lift(velocity_offset + leg_1, 1, 1.0);
  set_lift(velocity_offset + leg_1, 5, -1.0);
  set_lift(velocity_offset + leg_2, 1, 1.0);
  set_lift(velocity_offset + leg_2, 5, -1.0);
  set_lift(velocity_offset + wheel_1, 3, 1.0 / kWheelRadius);
  set_lift(velocity_offset + wheel_2, 3, 1.0 / kWheelRadius);

  // H observes the same six quantities from the full state; H*G = I.
  std::vector<double> observe(kReducedState * full_state, 0.0);
  auto set_observe = [&](int reduced_row, int full_col, double value) {
    observe[reduced_row * full_state + full_col] = value;
  };
  set_observe(0, root_pitch, 1.0);
  set_observe(0, leg_1, 0.5);
  set_observe(0, leg_2, 0.5);
  set_observe(2, root_x, 1.0);
  set_observe(2, root_pitch, -g_leg_length);
  set_observe(2, leg_1, -0.5 * g_leg_length);
  set_observe(2, leg_2, -0.5 * g_leg_length);
  set_observe(4, root_pitch, 1.0);
  set_observe(1, velocity_offset + root_pitch, 1.0);
  set_observe(1, velocity_offset + leg_1, 0.5);
  set_observe(1, velocity_offset + leg_2, 0.5);
  set_observe(3, velocity_offset + root_x, 1.0);
  set_observe(3, velocity_offset + root_pitch, -g_leg_length);
  set_observe(3, velocity_offset + leg_1, -0.5 * g_leg_length);
  set_observe(3, velocity_offset + leg_2, -0.5 * g_leg_length);
  set_observe(5, velocity_offset + root_pitch, 1.0);

  std::vector<double> input_map(model->nu * kReducedInput, 0.0);
  input_map[Actuator(model, "wheel_motor_1") * kReducedInput + 0] = 0.5;
  input_map[Actuator(model, "wheel_motor_2") * kReducedInput + 0] = 0.5;
  input_map[Actuator(model, "hip_motor_1") * kReducedInput + 1] = 0.5;
  input_map[Actuator(model, "hip_motor_2") * kReducedInput + 1] = 0.5;

  const auto a_lifted = Multiply(full_a, full_state, full_state, lift,
                                 kReducedState);
  const auto ad = Multiply(observe, kReducedState, full_state, a_lifted,
                           kReducedState);
  const auto b_mapped = Multiply(full_b, full_state, model->nu, input_map,
                                 kReducedInput);
  const auto bd = Multiply(observe, kReducedState, full_state, b_mapped,
                           kReducedInput);

  std::vector<double> ac = ad;
  std::vector<double> bc = bd;
  for (int row = 0; row < kReducedState; ++row) {
    for (int col = 0; col < kReducedState; ++col) {
      ac[row * kReducedState + col] -= row == col ? 1.0 : 0.0;
      ac[row * kReducedState + col] /= model->opt.timestep;
    }
  }
  for (double& value : bc) {
    value /= model->opt.timestep;
  }

  std::vector<double> controllability(kReducedState *
                                      (kReducedState * kReducedInput), 0.0);
  std::vector<double> block = bd;
  for (int power = 0; power < kReducedState; ++power) {
    for (int row = 0; row < kReducedState; ++row) {
      for (int input = 0; input < kReducedInput; ++input) {
        controllability[row * (kReducedState * kReducedInput) +
                        power * kReducedInput + input] =
            block[row * kReducedInput + input];
      }
    }
    block = Multiply(ad, kReducedState, kReducedState, block, kReducedInput);
  }

  std::printf("Reduced symmetric identification at L=%.3f m, t=%.3f s, "
              "dt=%.6g s\n", g_leg_length, data->time,
              model->opt.timestep);
  double max_qacc = 0.0;
  for (int i = 0; i < model->nv; ++i) {
    max_qacc = std::max(max_qacc, std::abs(data->qacc[i]));
  }
  std::printf("contact_count=%d  max_qacc=%.6g\n", data->ncon,
              max_qacc);
  PrintMatrix("Ad", ad, kReducedState, kReducedState);
  PrintMatrix("Bd", bd, kReducedState, kReducedInput);
  PrintMatrix("A ~= (Ad-I)/dt", ac, kReducedState, kReducedState);
  PrintMatrix("B ~= Bd/dt", bc, kReducedState, kReducedInput);
  std::printf("controllability rank = %d / %d\n",
              MatrixRank(controllability, kReducedState,
                         kReducedState * kReducedInput, 1e-8),
              kReducedState);

  const auto gain = SolveDare(ad, bd);
  PrintMatrix("Discrete LQR K", gain, kReducedInput, kReducedState);
  const std::vector<double> nominal_qpos(data->qpos, data->qpos + model->nq);
  std::printf("Nonlinear closed-loop validation (2.0 s):\n");
  ValidateClosedLoop(model, nominal_qpos, gain, root_x, root_pitch, leg_1,
                     leg_2, {0.01, 0.0, 0.0}, "theta");
  ValidateClosedLoop(model, nominal_qpos, gain, root_x, root_pitch, leg_1,
                     leg_2, {0.0, 0.01, 0.0}, "x");
  ValidateClosedLoop(model, nominal_qpos, gain, root_x, root_pitch, leg_1,
                     leg_2, {0.0, 0.0, 0.01}, "phi");

  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}

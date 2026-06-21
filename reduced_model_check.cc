#include <mujoco/mujoco.h>

#include <cmath>
#include <cstdio>
#include <string>

namespace {

int JointQposAddress(const mjModel* model, const char* name) {
  const int id = mj_name2id(model, mjOBJ_JOINT, name);
  return id >= 0 ? model->jnt_qposadr[id] : -1;
}

double TotalMass(const mjModel* model) {
  double mass = 0.0;
  for (int i = 1; i < model->nbody; ++i) {
    mass += model->body_mass[i];
  }
  return mass;
}

void PrintState(const mjModel* model, const mjData* data, const char* label) {
  const int x = JointQposAddress(model, "root_x");
  const int z = JointQposAddress(model, "root_z");
  const int phi = JointQposAddress(model, "root_pitch");
  const int theta_1 = JointQposAddress(model, "leg_theta_1");
  const int theta_2 = JointQposAddress(model, "leg_theta_2");
  std::printf("%-12s t=%7.3f  x=% .6f  z=% .6f  phi=% .6f  "
              "theta=(% .6f,% .6f)  contacts=%d\n",
              label, data->time, data->qpos[x], 0.23 + data->qpos[z],
              data->qpos[phi], data->qpos[theta_1], data->qpos[theta_2],
              data->ncon);
}

}  // namespace

int main(int argc, char** argv) {
  const std::string path = argc > 1 ? argv[1] : "wbr_reduced.xml";
  char error[1024] = {};
  mjModel* model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Failed to load %s: %s\n", path.c_str(), error);
    return 1;
  }
  mjData* data = mj_makeData(model);
  if (!data) {
    mj_deleteModel(model);
    return 1;
  }

  mj_resetDataKeyframe(model, data, 0);
  mj_forward(model, data);
  std::printf("model=%s  nq=%lld nv=%lld nu=%lld neq=%lld  total_mass=%.6f kg\n",
              path.c_str(), static_cast<long long>(model->nq),
              static_cast<long long>(model->nv),
              static_cast<long long>(model->nu),
              static_cast<long long>(model->neq),
              TotalMass(model));
  PrintState(model, data, "initial");

  // Lean the equivalent pendulum while keeping the wheel-center x unchanged.
  constexpr double kThetaPerturbation = 0.01;
  constexpr double kLegLength = 0.18;
  data->qpos[JointQposAddress(model, "root_x")] =
      kLegLength * kThetaPerturbation;
  data->qpos[JointQposAddress(model, "leg_theta_1")] = kThetaPerturbation;
  data->qpos[JointQposAddress(model, "leg_theta_2")] = kThetaPerturbation;
  mj_forward(model, data);
  PrintState(model, data, "perturbed");

  double next_report = 0.1;
  while (data->time < 1.0) {
    mj_step(model, data);
    if (data->time + 1e-12 >= next_report) {
      PrintState(model, data, "passive");
      next_report += 0.1;
    }
  }

  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}

#include <array>
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

struct Point2 {
  double x;
  double z;
};

struct AnalyticPoints {
  Point2 b;
  Point2 c;
  Point2 d;
  Point2 g;
  Point2 h;
};

double Distance(Point2 a, Point2 b) {
  return std::hypot(a.x - b.x, a.z - b.z);
}

bool ComputeAnalyticPoints(double phi1, double phi2, int branch,
                           AnalyticPoints& points) {
  points.b = {kLengthAB * std::cos(phi2), -kLengthAB * std::sin(phi2)};
  points.d = {kLengthAD * std::cos(phi1), -kLengthAD * std::sin(phi1)};
  points.g = {kLengthAG * std::cos(phi1), -kLengthAG * std::sin(phi1)};
  const double dx = points.d.x - points.b.x;
  const double dz = points.d.z - points.b.z;
  const double distance = std::hypot(dx, dz);
  if (distance > kLengthBC + kLengthCD - 1e-9 || distance < 1e-9) {
    return false;
  }
  const double a = (kLengthBC * kLengthBC - kLengthCD * kLengthCD +
                    distance * distance) /
                   (2.0 * distance);
  const double h = std::sqrt(std::fmax(0.0, kLengthBC * kLengthBC - a * a));
  const double px = points.b.x + a * dx / distance;
  const double pz = points.b.z + a * dz / distance;
  if (branch == 1) {
    points.c = {px - h * dz / distance, pz + h * dx / distance};
  } else {
    points.c = {px + h * dz / distance, pz - h * dx / distance};
  }
  const double dcx = points.c.x - points.d.x;
  const double dcz = points.c.z - points.d.z;
  const double dc_length = std::hypot(dcx, dcz);
  if (dc_length < 1e-12) {
    return false;
  }
  points.h = {
      points.g.x + kLengthGH * dcx / dc_length,
      points.g.z + kLengthGH * dcz / dc_length,
  };
  return true;
}

bool InverseKinematics(double target_hx, double target_hz,
                       double phi1_initial, double phi2_initial,
                       double& phi1, double& phi2, int& branch_out) {
  const std::array<std::array<double, 2>, 5> guesses = {{
      {phi1_initial, phi2_initial}, {1.05, 2.44}, {-1.05, 2.44},
      {1.05, -2.44}, {-1.05, -2.44},
  }};
  double best_distance = 1e20;
  bool found = false;
  for (int branch = 1; branch <= 2; ++branch) {
    for (const auto& guess : guesses) {
      double p1 = guess[0];
      double p2 = guess[1];
      for (int iteration = 0; iteration < 60; ++iteration) {
        AnalyticPoints base;
        if (!ComputeAnalyticPoints(p1, p2, branch, base)) {
          break;
        }
        const double ex = base.h.x - target_hx;
        const double ez = base.h.z - target_hz;
        if (ex * ex + ez * ez < 1e-12) {
          const double solution_distance =
              (p1 - phi1_initial) * (p1 - phi1_initial) +
              (p2 - phi2_initial) * (p2 - phi2_initial);
          if (solution_distance < best_distance) {
            best_distance = solution_distance;
            phi1 = p1;
            phi2 = p2;
            branch_out = branch;
            found = true;
          }
          break;
        }
        constexpr double step = 1e-7;
        AnalyticPoints p1_points;
        AnalyticPoints p2_points;
        if (!ComputeAnalyticPoints(p1 + step, p2, branch, p1_points) ||
            !ComputeAnalyticPoints(p1, p2 + step, branch, p2_points)) {
          break;
        }
        const double j11 = (p1_points.h.x - base.h.x) / step;
        const double j21 = (p1_points.h.z - base.h.z) / step;
        const double j12 = (p2_points.h.x - base.h.x) / step;
        const double j22 = (p2_points.h.z - base.h.z) / step;
        const double determinant = j11 * j22 - j12 * j21;
        if (std::fabs(determinant) < 1e-12) {
          break;
        }
        p1 -= 0.5 * (j22 * ex - j12 * ez) / determinant;
        p2 -= 0.5 * (-j21 * ex + j11 * ez) / determinant;
      }
    }
  }
  return found;
}

Point2 WorldPointInBodyXZ(const mjData* data, int body_id,
                          const mjtNum world_point[3]) {
  const mjtNum* origin = data->xpos + 3 * body_id;
  const mjtNum* rotation = data->xmat + 9 * body_id;
  const double dx = world_point[0] - origin[0];
  const double dy = world_point[1] - origin[1];
  const double dz = world_point[2] - origin[2];
  return {
      rotation[0] * dx + rotation[3] * dy + rotation[6] * dz,
      rotation[2] * dx + rotation[5] * dy + rotation[8] * dz,
  };
}

Point2 BodyOriginInBodyXZ(const mjData* data, int reference_body, int point_body) {
  return WorldPointInBodyXZ(data, reference_body, data->xpos + 3 * point_body);
}

Point2 SiteInBodyXZ(const mjData* data, int reference_body, int site_id) {
  return WorldPointInBodyXZ(data, reference_body, data->site_xpos + 3 * site_id);
}

int RequiredId(const mjModel* model, int type, const char* name) {
  const int id = mj_name2id(model, type, name);
  if (id < 0) {
    std::fprintf(stderr, "Missing model object: %s\n", name);
    std::exit(2);
  }
  return id;
}

void PrintPoint(const char* name, Point2 analytic, Point2 actual) {
  std::printf("  %-2s analytic=(% .8f,% .8f) actual=(% .8f,% .8f) "
              "error=(% .8f,% .8f) norm=%.9g\n",
              name, analytic.x, analytic.z, actual.x, actual.z,
              actual.x - analytic.x, actual.z - analytic.z,
              Distance(analytic, actual));
}

void PrintEqualityResiduals(const mjModel* model, const mjData* data) {
  std::vector<double> maximum(model->neq, 0.0);
  for (int row = 0; row < data->nefc; ++row) {
    if (data->efc_type[row] == mjCNSTR_EQUALITY &&
        data->efc_id[row] >= 0 && data->efc_id[row] < model->neq) {
      maximum[data->efc_id[row]] =
          std::fmax(maximum[data->efc_id[row]], std::fabs(data->efc_pos[row]));
    }
  }
  std::printf("Equality residuals (max |efc_pos|):\n");
  for (int equality = 0; equality < model->neq; ++equality) {
    const char* name = mj_id2name(model, mjOBJ_EQUALITY, equality);
    int object_type = model->eq_objtype[equality];
    if (model->eq_type[equality] == mjEQ_JOINT) object_type = mjOBJ_JOINT;
    if (model->eq_type[equality] == mjEQ_TENDON) object_type = mjOBJ_TENDON;
    if (model->eq_type[equality] == mjEQ_CONNECT) object_type = mjOBJ_SITE;
    const char* object_1 = model->eq_obj1id[equality] >= 0
        ? mj_id2name(model, object_type, model->eq_obj1id[equality]) : "world";
    const char* object_2 = model->eq_obj2id[equality] >= 0
        ? mj_id2name(model, object_type, model->eq_obj2id[equality]) : "world";
    const char* type = "other";
    if (model->eq_type[equality] == mjEQ_CONNECT) type = "connect";
    if (model->eq_type[equality] == mjEQ_JOINT) type = "joint";
    if (model->eq_type[equality] == mjEQ_TENDON) type = "tendon";
    std::printf("  [%d] %-7s %-12s <-> %-12s residual=%.9g%s%s\n",
                equality, type, object_1 ? object_1 : "(unnamed)",
                object_2 ? object_2 : "(unnamed)", maximum[equality],
                name ? " name=" : "", name ? name : "");
  }
}

void ReportLeg(const mjModel* model, const mjData* data, int leg) {
  const char* suffix = leg == 0 ? "" : "_2";
  char name[64];
  std::snprintf(name, sizeof(name), "A%s", suffix);
  const int body_a = RequiredId(model, mjOBJ_BODY, name);
  std::snprintf(name, sizeof(name), "BC%s", suffix);
  const int body_bc = RequiredId(model, mjOBJ_BODY, name);
  std::snprintf(name, sizeof(name), "C%s", suffix);
  const int body_c = RequiredId(model, mjOBJ_BODY, name);
  std::snprintf(name, sizeof(name), "D_site%s", suffix);
  const int site_d = RequiredId(model, mjOBJ_SITE, name);
  std::snprintf(name, sizeof(name), "G_site%s", suffix);
  const int site_g = RequiredId(model, mjOBJ_SITE, name);
  std::snprintf(name, sizeof(name), "H_site%s", suffix);
  const int site_h = RequiredId(model, mjOBJ_SITE, name);
  std::snprintf(name, sizeof(name), "q_B%s", suffix);
  const int joint_b = RequiredId(model, mjOBJ_JOINT, name);
  std::snprintf(name, sizeof(name), "q_D%s", suffix);
  const int joint_d = RequiredId(model, mjOBJ_JOINT, name);
  const double mirror = leg == 0 ? 1.0 : -1.0;
  const double phi1 = mirror * data->qpos[model->jnt_qposadr[joint_d]];
  const double phi2 = mirror * data->qpos[model->jnt_qposadr[joint_b]];

  const Point2 actual_b = BodyOriginInBodyXZ(data, body_a, body_bc);
  const Point2 actual_c = BodyOriginInBodyXZ(data, body_a, body_c);
  const Point2 actual_d = SiteInBodyXZ(data, body_a, site_d);
  const Point2 actual_g = SiteInBodyXZ(data, body_a, site_g);
  const Point2 actual_h = SiteInBodyXZ(data, body_a, site_h);
  AnalyticPoints branch_1;
  AnalyticPoints branch_2;
  const bool valid_1 = ComputeAnalyticPoints(phi1, phi2, 1, branch_1);
  const bool valid_2 = ComputeAnalyticPoints(phi1, phi2, 2, branch_2);
  if (!valid_1 && !valid_2) {
    std::printf("Leg %d: analytic FK is unreachable.\n", leg + 1);
    return;
  }
  int branch = 1;
  AnalyticPoints analytic = branch_1;
  if (!valid_1 || (valid_2 && Distance(branch_2.c, actual_c) <
                                  Distance(branch_1.c, actual_c))) {
    branch = 2;
    analytic = branch_2;
  }

  std::printf("Leg %d: phi1=% .8f phi2=% .8f selected branch=%d\n",
              leg + 1, phi1, phi2, branch);
  PrintPoint("B", analytic.b, actual_b);
  PrintPoint("C", analytic.c, actual_c);
  PrintPoint("D", analytic.d, actual_d);
  PrintPoint("G", analytic.g, actual_g);
  PrintPoint("H", analytic.h, actual_h);
  std::printf("  H length: analytic=%.9g actual=%.9g delta=% .9g\n",
              std::hypot(analytic.h.x, analytic.h.z),
              std::hypot(actual_h.x, actual_h.z),
              std::hypot(actual_h.x, actual_h.z) -
                  std::hypot(analytic.h.x, analytic.h.z));
}

void Report(const char* stage, const mjModel* model, const mjData* data) {
  std::printf("\n================ %s ================\n", stage);
  ReportLeg(model, data, 0);
  ReportLeg(model, data, 1);
  PrintEqualityResiduals(model, data);
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::printf("Usage: kinematics_checker model.xml [settle_steps] [target_hx target_hz]\n");
    return 0;
  }
  const int settle_steps = argc > 2 ? std::atoi(argv[2]) : 3000;
  const bool has_target = argc > 4;
  const double target_hx = has_target ? std::atof(argv[3]) : 0.0;
  const double target_hz = has_target ? std::atof(argv[4]) : 0.0;
  char error[1024] = {};
  mjModel* model = mj_loadXML(argv[1], nullptr, error, sizeof(error));
  if (!model) {
    std::fprintf(stderr, "Could not load %s: %s\n", argv[1], error);
    return 1;
  }
  mjData* data = mj_makeData(model);
  const int key = mj_name2id(model, mjOBJ_KEY, "init");
  if (key >= 0) {
    mj_resetDataKeyframe(model, data, key);
  }
  mj_forward(model, data);
  Report("RAW KEYFRAME", model, data);

  if (has_target) {
    for (int leg = 0; leg < 2; ++leg) {
      const char* suffix = leg == 0 ? "" : "_2";
      char name[64];
      std::snprintf(name, sizeof(name), "q_B%s", suffix);
      const int joint_b = RequiredId(model, mjOBJ_JOINT, name);
      std::snprintf(name, sizeof(name), "q_D%s", suffix);
      const int joint_d = RequiredId(model, mjOBJ_JOINT, name);
      std::snprintf(name, sizeof(name), "act_q_B%s", suffix);
      const int actuator_b = RequiredId(model, mjOBJ_ACTUATOR, name);
      std::snprintf(name, sizeof(name), "act_q_D%s", suffix);
      const int actuator_d = RequiredId(model, mjOBJ_ACTUATOR, name);
      const double mirror = leg == 0 ? 1.0 : -1.0;
      const double initial_phi1 = mirror * data->qpos[model->jnt_qposadr[joint_d]];
      const double initial_phi2 = mirror * data->qpos[model->jnt_qposadr[joint_b]];
      double phi1;
      double phi2;
      int branch = 1;
      if (!InverseKinematics(target_hx, target_hz, initial_phi1, initial_phi2,
                             phi1, phi2, branch)) {
        std::fprintf(stderr, "IK failed for leg %d target (%g,%g).\n",
                     leg + 1, target_hx, target_hz);
        mj_deleteData(data);
        mj_deleteModel(model);
        return 2;
      }
      data->ctrl[actuator_b] = mirror * phi2;
      data->ctrl[actuator_d] = mirror * phi1;
      std::printf("Leg %d target IK: branch=%d phi1=%g phi2=%g\n",
                  leg + 1, branch, phi1, phi2);
    }
  }

  for (int step = 0; step < settle_steps; ++step) {
    mj_step(model, data);
  }
  Report(has_target ? "TARGET SETTLED" : "KEYFRAME SETTLED", model, data);
  mj_deleteData(data);
  mj_deleteModel(model);
  return 0;
}

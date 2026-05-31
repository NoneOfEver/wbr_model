#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>

#include <GLFW/glfw3.h>
#include <mujoco/mujoco.h>

// ================================
// 连杆参数 (与 MATLAB IKS.m 一致)
// ================================
constexpr double L_AB = 0.0945;
constexpr double L_BC = 0.1125;
constexpr double L_CD = 0.116;
constexpr double L_AD = 0.090;
constexpr double L_AG = 0.210;
constexpr double L_FG = 0.060;
constexpr double L_GH = 0.250;

inline void ClampTargetHStart(double& Hx, double& Hz)
{
    // 先计算四边形顶点 A/D/G 的世界平面坐标
    // 我们只做局部 x-z 平面限制
    // H 最远不能超过 G + L_GH
    double r = 0.4 + 1e-6;  // 安全缓冲
    double dist = std::sqrt(Hx*Hx + Hz*Hz);
    if(dist > r) {
        Hx *= r / dist;
        Hz *= r / dist;
    }
}

inline bool ClampTargetH(
  double& Hx,
  double& Hz,
  double prev_Hx,
  double prev_Hz)
{
  double r = 0.4;
  double dist = std::sqrt(Hx * Hx + Hz * Hz);

  if(dist > r)
  {
      Hx = prev_Hx;
      Hz = prev_Hz;
      return false;
  }

  return true;
}

// 将角度 phi 尽量靠近 ref，避免跨 pi/2 的跳变
inline double unwrap_angle(double phi, double ref)
{
    while (phi - ref > M_PI)  phi -= 2*M_PI;
    while (phi - ref < -M_PI) phi += 2*M_PI;
    return phi;
}

// ================================
// 正运动学
//   branch: 1=C1 (MATLAB 上方解), 2=C2
//   返回 H 点相对于 A 的局部坐标 (x, z)
// ================================
bool ForwardKinematics(double phi1, double phi2, int branch, double& Hx, double& Hz) {
  double Bx = L_AB * std::cos(phi2);
  double Bz = -L_AB * std::sin(phi2);

  double Dx = L_AD * std::cos(phi1);
  double Dz = -L_AD * std::sin(phi1);

  double Gx = L_AG * std::cos(phi1);
  double Gz = -L_AG * std::sin(phi1);

  double dx = Dx - Bx;
  double dz = Dz - Bz;
  double d = std::sqrt(dx * dx + dz * dz);

  if (d > L_BC + L_CD - 1e-9 || d < 1e-9) {
    return false;
  }

  double a = (L_BC * L_BC - L_CD * L_CD + d * d) / (2.0 * d);
  double h2 = L_BC * L_BC - a * a;
  if (h2 < 0) h2 = 0;
  double h = std::sqrt(h2);

  double Px = Bx + a * dx / d;
  double Pz = Bz + a * dz / d;

  double Cx, Cz;
  if (branch == 1) {
    Cx = Px - h * dz / d;
    Cz = Pz + h * dx / d;
  } else {
    Cx = Px + h * dz / d;
    Cz = Pz - h * dx / d;
  }

  double dcx = Cx - Dx;
  double dcz = Cz - Dz;
  double dc_len = std::sqrt(dcx * dcx + dcz * dcz);
  if (dc_len < 1e-12) {
    return false;
  }
  double FG_dx = dcx / dc_len;
  double FG_dz = dcz / dc_len;

  Hx = Gx + L_GH * FG_dx;
  Hz = Gz + L_GH * FG_dz;
  return true;
}

// ================================
// 逆运动学 (阻尼牛顿法 + 多初始值 + 双分支)
// ================================
bool InverseKinematics(double Hx_target, double Hz_target,
                       double phi1_init, double phi2_init,
                       double& phi1, double& phi2,int& branch_out) {
  // 多组初始猜测，类似 IKS.m 的 phi_init_set
  const std::vector<std::pair<double, double>> inits = {
    {phi1_init, phi2_init},
    {1.05, 2.44},     // ~60°, 120°
    {-1.05, 2.44},    // ~-60°, 120°
    {1.05, -2.44},    // ~60°, -120°
    {-1.05, -2.44}    // ~-60°, -120°
  };

  double best_dist = 1e20;
  bool found = false;

  for (int branch = 1; branch <= 2; ++branch) {
    for (const auto& init : inits) {
      double p1 = init.first;
      double p2 = init.second;

      const int max_iter = 50;
      const double dphi = 1e-7;

      for (int iter = 0; iter < max_iter; ++iter) {
        double Hx, Hz;
        if (!ForwardKinematics(p1, p2, branch, Hx, Hz)) {
          break;
        }

        double ex = Hx - Hx_target;
        double ez = Hz - Hz_target;
        double err = ex * ex + ez * ez;

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

        // 数值雅可比
        double Hx1, Hz1, Hx2, Hz2;
        ForwardKinematics(p1 + dphi, p2, branch, Hx1, Hz1);
        ForwardKinematics(p1, p2 + dphi, branch, Hx2, Hz2);

        double J11 = (Hx1 - Hx) / dphi;
        double J21 = (Hz1 - Hz) / dphi;
        double J12 = (Hx2 - Hx) / dphi;
        double J22 = (Hz2 - Hz) / dphi;

        double det = J11 * J22 - J12 * J21;
        if (std::fabs(det) < 1e-12) {
          // 奇异：梯度下降
          p1 -= 0.05 * (J11 * ex + J21 * ez);
          p2 -= 0.05 * (J12 * ex + J22 * ez);
        } else {
          // 阻尼牛顿法
          double d1 = (J22 * ex - J12 * ez) / det;
          double d2 = (-J21 * ex + J11 * ez) / det;
          double lambda = 0.5;
          p1 -= lambda * d1;
          p2 -= lambda * d2;
        }
      }
    }
  }

  return found;
}

// ================================
// MuJoCo viewer 全局变量
// ================================
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// H 点目标 (相对于各自 A 的局部坐标 x, z)
double target_Hx = 0.15;
double target_Hz = 0.10;
double target_Hx_2 = 0.15;
double target_Hz_2 = 0.10;
double step_size = 0.01;
bool paused = false;

// ================================
// 键盘回调
// ================================
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
  if (act != GLFW_PRESS && act != GLFW_REPEAT) return;
  double newHz, newHx, newHz_2, newHx_2;
  switch (key) {
    case GLFW_KEY_UP:
      newHz = target_Hz + step_size;
      if(std::sqrt(target_Hx*target_Hx +
                  newHz*newHz) <= 0.4)
      {
          target_Hz = newHz;
      }
      break;
    case GLFW_KEY_DOWN:
      newHz = target_Hz - step_size;
      if(std::sqrt(target_Hx*target_Hx + newHz*newHz) <= 0.4)
      {
          target_Hz = newHz;
      }
      break;
    case GLFW_KEY_LEFT:
      newHx = target_Hx - step_size;
      if(std::sqrt(newHx*newHx + target_Hz*target_Hz) <= 0.4)
      {
          target_Hx = newHx;
      }
      break;
    case GLFW_KEY_RIGHT:
      newHx = target_Hx + step_size;
      if(std::sqrt(newHx*newHx + target_Hz*target_Hz) <= 0.4)
      {
          target_Hx = newHx;
      }
      break;
    case GLFW_KEY_W:
      newHz_2 = target_Hz_2 + step_size;
      if(std::sqrt(target_Hx_2*target_Hx_2 +
                  newHz_2*newHz_2) <= 0.4)
      {
          target_Hz_2 = newHz_2;
      }
      break;
    case GLFW_KEY_S:
      newHz_2 = target_Hz_2 - step_size;
      if(std::sqrt(target_Hx_2*target_Hx_2 + newHz_2*newHz_2) <= 0.4)
      {
          target_Hz_2 = newHz_2;
      }
      break;
    case GLFW_KEY_A:
      newHx_2 = target_Hx_2 - step_size;
      if(std::sqrt(newHx_2*newHx_2 + target_Hz_2*target_Hz_2) <= 0.4)
      {
          target_Hx_2 = newHx_2;
      }
      break;
    case GLFW_KEY_D:
      newHx_2 = target_Hx_2 + step_size;
      if(std::sqrt(newHx_2*newHx_2 + target_Hz_2*target_Hz_2) <= 0.4)
      {
          target_Hx_2 = newHx_2;
      }
      break;
    case GLFW_KEY_PAGE_UP:
      step_size *= 2.0;
      if (step_size > 0.1) step_size = 0.1;
      std::printf("Step size: %.4f\n", step_size);
      break;
    case GLFW_KEY_PAGE_DOWN:
      step_size *= 0.5;
      if (step_size < 0.001) step_size = 0.001;
      std::printf("Step size: %.4f\n", step_size);
      break;
    case GLFW_KEY_R:
      target_Hx = 0.15;
      target_Hz = 0.10;
      target_Hx_2 = 0.15;
      target_Hz_2 = 0.10;
      std::printf("Reset target H1/H2 to (%.3f, %.3f)\n", target_Hx, target_Hz);
      break;
    case GLFW_KEY_SPACE:
      paused = !paused;
      break;

    case GLFW_KEY_BACKSPACE:
      mj_resetData(m, d);
      mj_forward(m, d);
      break;
  }
}

// ================================
// 鼠标回调
// ================================
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
  button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
  button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
  button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
  glfwGetCursorPos(window, &lastx, &lasty);
}

void mouse_move(GLFWwindow* window, double xpos, double ypos) {
  if (!button_left && !button_middle && !button_right) return;

  double dx = xpos - lastx;
  double dy = ypos - lasty;
  lastx = xpos;
  lasty = ypos;

  int width, height;
  glfwGetWindowSize(window, &width, &height);

  bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);

  mjtMouse action;
  if (button_right) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (button_left) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  } else {
    action = mjMOUSE_ZOOM;
  }

  mjv_moveCamera(m, action, dx / height, dy / height, &scn, &cam);
}

void scroll(GLFWwindow* window, double xoffset, double yoffset) {
  mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// ================================
// 在场景中添加标记球
// ================================
void AddMarker(mjvScene* s, const double pos[3], float r, float grn, float b, float size) {
  if (s->ngeom >= s->maxgeom) return;
  mjvGeom* geom = s->geoms + s->ngeom++;
  mjv_initGeom(geom, mjGEOM_SPHERE, nullptr, nullptr, nullptr, nullptr);
  geom->size[0] = size;
  geom->size[1] = size;
  geom->size[2] = size;
  geom->pos[0] = pos[0];
  geom->pos[1] = pos[1];
  geom->pos[2] = pos[2];
  geom->rgba[0] = r;
  geom->rgba[1] = grn;
  geom->rgba[2] = b;
  geom->rgba[3] = 1.0f;
}

// ================================
// 主函数
// ================================
int main(int argc, const char** argv) {
  if (argc != 2) {
    std::printf("USAGE: ik_viewer modelfile\n");
    std::printf("  Controls:\n");
    std::printf("    Arrow keys : move H1 target\n");
    std::printf("    WASD : move H2 target\n");
    std::printf("    Page Up/Down : adjust step size\n");
    std::printf("    R : reset H target\n");
    std::printf("    Space : pause/unpause\n");
    std::printf("    Backspace : reset simulation\n");
    return 0;
  }

  char error[1024] = "";
  if (std::strlen(argv[1]) > 4 &&
      !std::strcmp(argv[1] + std::strlen(argv[1]) - 4, ".mjb")) {
    m = mj_loadModel(argv[1], 0);
  } else {
    m = mj_loadXML(argv[1], 0, error, sizeof(error));
  }
  if (!m) {
    mju_error("Load model error: %s", error);
  }

  d = mj_makeData(m);

  // 查找 actuator / body / site ID
  int act_q_b = mj_name2id(m, mjOBJ_ACTUATOR, "act_q_B");
  int act_q_d = mj_name2id(m, mjOBJ_ACTUATOR, "act_q_D");
  int act_q_b_2 = mj_name2id(m, mjOBJ_ACTUATOR, "act_q_B_2");
  int act_q_d_2 = mj_name2id(m, mjOBJ_ACTUATOR, "act_q_D_2");
  int joint_B = mj_name2id(m, mjOBJ_JOINT, "q_B");
  int joint_D = mj_name2id(m, mjOBJ_JOINT, "q_D");
  int joint_B_2 = mj_name2id(m, mjOBJ_JOINT, "q_B_2");
  int joint_D_2 = mj_name2id(m, mjOBJ_JOINT, "q_D_2");
  int body_a = mj_name2id(m, mjOBJ_BODY, "A");
  int body_a_2 = mj_name2id(m, mjOBJ_BODY, "A_2");
  int h_site = mj_name2id(m, mjOBJ_SITE, "H_site");
  int h_site_2 = mj_name2id(m, mjOBJ_SITE, "H_site_2");

  if (act_q_b < 0 || act_q_d < 0 || act_q_b_2 < 0 || act_q_d_2 < 0) {
    mju_error("act_q_B/act_q_D or act_q_B_2/act_q_D_2 not found");
  }
  if (body_a < 0 || body_a_2 < 0) {
    mju_error("Body A/A_2 not found");
  }

  // 初始状态（优先加载 init keyframe）
  mj_resetData(m, d);
  int key_id = mj_name2id(m, mjOBJ_KEY, "init");
  if (key_id >= 0) {
    mj_resetDataKeyframe(m, d, key_id);
  }
  mj_forward(m, d);

  // A 点 world 位置 (固定)
  double A_pos[3];
  A_pos[0] = d->xpos[3 * body_a + 0];
  A_pos[1] = d->xpos[3 * body_a + 1];
  A_pos[2] = d->xpos[3 * body_a + 2];

  double A2_pos[3];
  A2_pos[0] = d->xpos[3 * body_a_2 + 0];
  A2_pos[1] = d->xpos[3 * body_a_2 + 1];
  A2_pos[2] = d->xpos[3 * body_a_2 + 2];

  // 初始化 GLFW
  if (!glfwInit()) {
    mju_error("Could not initialize GLFW");
  }

  GLFWwindow* window = glfwCreateWindow(1200, 900, "IK Viewer - H Point Control", nullptr, nullptr);
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);

  mjv_defaultCamera(&cam);
  mjv_defaultOption(&opt);
  mjv_defaultScene(&scn);
  mjr_defaultContext(&con);

  mjv_makeScene(m, &scn, 2000);
  mjr_makeContext(m, &con, mjFONTSCALE_150);

  glfwSetKeyCallback(window, keyboard);
  glfwSetCursorPosCallback(window, mouse_move);
  glfwSetMouseButtonCallback(window, mouse_button);
  glfwSetScrollCallback(window, scroll);

  // 上一帧的解 (弧度)
  double phi1_prev = d->qpos[m->jnt_qposadr[joint_D]];;  
  double phi2_prev = d->qpos[m->jnt_qposadr[joint_B]];  
  double phi1_prev_2 = d->qpos[m->jnt_qposadr[joint_D_2]]; 
  double phi2_prev_2 = d->qpos[m->jnt_qposadr[joint_B_2]];
  bool ik_failed = false;
  bool ik_failed_2 = false;
  int fail_count = 0;
  int fail_count_2 = 0;

  ForwardKinematics(phi1_prev, phi2_prev, 1, target_Hx, target_Hz);
  ForwardKinematics(phi1_prev_2, phi2_prev_2, 1, target_Hx_2, target_Hz_2);
  while (!glfwWindowShouldClose(window)) {
    // ---- 逆运动学求解 ----
    double target_Hx_local = target_Hx;
    double target_Hz_local = target_Hz;
    double target_Hx_local_2 = target_Hx_2;
    double target_Hz_local_2 = target_Hz_2;

    ClampTargetHStart(target_Hx_local, target_Hz_local);
    ClampTargetHStart(target_Hx_local_2, target_Hz_local_2);

    static bool first_print = true;
    if(first_print)
    {
      printf("\n=== IK INPUT ===\n");
      printf("target_Hx=%.6f\n", target_Hx);
      printf("target_Hx_local=%.6f\n", target_Hx_local);
      printf("target_Hz=%.6f\n", target_Hz);
      printf("phi1_prev=%.6f\n", phi1_prev);
      printf("phi2_prev=%.6f\n", phi2_prev);
    }


    double phi1, phi2;
    int branch_used;
    bool ok = InverseKinematics(target_Hx_local, target_Hz_local, phi1_prev, phi2_prev, phi1, phi2,branch_used);

    double phi1_2, phi2_2;
    int branch_used_2;
    bool ok_2 = InverseKinematics(target_Hx_local_2, target_Hz_local_2,
                                  phi1_prev_2, phi2_prev_2, phi1_2, phi2_2,branch_used_2);

    if (ok) {
      // 保持角度连续
      phi1 = unwrap_angle(phi1, phi1_prev);
      phi2 = unwrap_angle(phi2, phi2_prev);
      
      phi1_prev = phi1;
      phi2_prev = phi2;

      if(first_print)
      {
          printf("\n=== IK OUTPUT ===\n");
          printf("branch=%d phi1=%.6f rad (%.2f deg)\n",
                branch_used,
                phi1,
                phi1*180/M_PI);
          printf("phi2=%.6f rad (%.2f deg)\n",
                phi2,
                phi2*180/M_PI);
          double fkx, fkz;
          ForwardKinematics(
              phi1,
              phi2,
              branch_used,
              fkx,
              fkz);
          printf("\n=== FK CHECK ===\n");
          printf("target : %.6f %.6f\n",
                  target_Hx_local,
                  target_Hz);
          printf("fk     : %.6f %.6f\n",
                  fkx,
                  fkz);
          printf("error  : %.6f %.6f\n",
                  fkx - target_Hx_local,
                  fkz - target_Hz);
      }
      d->ctrl[act_q_b] = phi2;
      d->ctrl[act_q_d] = phi1;
      if(first_print)
      {
        int joint_B = mj_name2id(m, mjOBJ_JOINT, "q_B");
        int joint_D = mj_name2id(m, mjOBJ_JOINT, "q_D");

        printf("\n=== MUJOCO ===\n");

        printf("ctrl phi1 = %.6f\n", phi1);
        printf("ctrl phi2 = %.6f\n", phi2);

        printf("joint_D qpos = %.6f\n",
                d->qpos[m->jnt_qposadr[joint_D]]);

        printf("joint_B qpos = %.6f\n",
                d->qpos[m->jnt_qposadr[joint_B]]);
      }
      first_print = false;
      ik_failed = false;
      fail_count = 0;
    } else {
      if (!ik_failed) {
        std::printf("IK failed for H1 target (%.3f, %.3f)\n", target_Hx, target_Hz);
        ik_failed = true;
      }
      fail_count++;
    }

    if (ok_2) {
      phi1_2 = unwrap_angle(phi1_2, phi1_prev_2);
      phi2_2 = unwrap_angle(phi2_2, phi2_prev_2);
      
      phi1_prev_2 = phi1_2;
      phi2_prev_2 = phi2_2;
      d->ctrl[act_q_b_2] = phi2_2;
      d->ctrl[act_q_d_2] = phi1_2;
      ik_failed_2 = false;
      fail_count_2 = 0;
    } else {
      if (!ik_failed_2) {
        std::printf("IK failed for H2 target (%.3f, %.3f)\n", target_Hx_2, target_Hz_2);
        ik_failed_2 = true;
      }
      fail_count_2++;
    }

    // ---- 仿真步进 ----
    if (!paused) {
      mjtNum simstart = d->time;
      while (d->time - simstart < 1.0 / 60.0) {
        mj_step(m, d);
      }
    }

    // ---- 渲染 ----
    mjrRect viewport{0, 0, 0, 0};
    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

    // 红色标记：H1 目标位置
    const double* a_xmat = d->xmat + 9 * body_a;
    const double* a2_xmat = d->xmat + 9 * body_a_2;
    double a_x[3] = {a_xmat[0], a_xmat[1], a_xmat[2]};
    double a_z[3] = {a_xmat[6], a_xmat[7], a_xmat[8]};
    double a2_x[3] = {a2_xmat[0], a2_xmat[1], a2_xmat[2]};
    double a2_z[3] = {a2_xmat[6], a2_xmat[7], a2_xmat[8]};

    double target_pos[3] = {
      A_pos[0] + a_x[0] * target_Hx_local + a_z[0] * target_Hz,
      A_pos[1] + a_x[1] * target_Hx_local + a_z[1] * target_Hz,
      A_pos[2] + a_x[2] * target_Hx_local + a_z[2] * target_Hz
    };
    AddMarker(&scn, target_pos, 1.0f, 0.0f, 0.0f, 0.015f);
    // 紫色标记：H2 目标位置
    double target_pos_2[3] = {
      A2_pos[0] + a2_x[0] * target_Hx_local_2 + a2_z[0] * target_Hz_local_2,
      A2_pos[1] + a2_x[1] * target_Hx_local_2 + a2_z[1] * target_Hz_local_2,
      A2_pos[2] + a2_x[2] * target_Hx_local_2 + a2_z[2] * target_Hz_local_2
    };
    AddMarker(&scn, target_pos_2, 1.0f, 0.0f, 0.6f, 0.015f);

    // IK 计算的 H 点位置（相对于 A 的局部坐标）
    double ik_hx = 0.0;
    double ik_hz = 0.0;
    bool has_ik_h = ForwardKinematics(phi1_prev, phi2_prev, branch_used, ik_hx, ik_hz);
    double ik_hx_2 = 0.0;
    double ik_hz_2 = 0.0;
    bool has_ik_h_2 = ForwardKinematics(phi1_prev_2, phi2_prev_2, branch_used_2, ik_hx_2, ik_hz_2);

    // 绿色标记：实际 H 点位置
    double actual_hx = 0.0;
    double actual_hz = 0.0;
    bool has_actual_h = false;
    if (h_site >= 0) {
      double actual_pos[3] = {
        d->site_xpos[3 * h_site + 0],
        d->site_xpos[3 * h_site + 1],
        d->site_xpos[3 * h_site + 2]
      };
      AddMarker(&scn, actual_pos, 0.0f, 1.0f, 0.0f, 0.012f);
      double dx[3] = {
        actual_pos[0] - A_pos[0],
        actual_pos[1] - A_pos[1],
        actual_pos[2] - A_pos[2]
      };
      actual_hx = dx[0] * a_x[0] + dx[1] * a_x[1] + dx[2] * a_x[2];
      actual_hz = dx[0] * a_z[0] + dx[1] * a_z[1] + dx[2] * a_z[2];
      has_actual_h = true;
    }

    double actual_hx_2 = 0.0;
    double actual_hz_2 = 0.0;
    bool has_actual_h_2 = false;
    if (h_site_2 >= 0) {
      double actual_pos_2[3] = {
        d->site_xpos[3 * h_site_2 + 0],
        d->site_xpos[3 * h_site_2 + 1],
        d->site_xpos[3 * h_site_2 + 2]
      };
      AddMarker(&scn, actual_pos_2, 0.0f, 0.7f, 1.0f, 0.012f);
      double dx_2[3] = {
        actual_pos_2[0] - A2_pos[0],
        actual_pos_2[1] - A2_pos[1],
        actual_pos_2[2] - A2_pos[2]
      };
      actual_hx_2 = dx_2[0] * a2_x[0] + dx_2[1] * a2_x[1] + dx_2[2] * a2_x[2];
      actual_hz_2 = dx_2[0] * a2_z[0] + dx_2[1] * a2_z[1] + dx_2[2] * a2_z[2];
      has_actual_h_2 = true;
    }

    mjr_render(viewport, &scn, &con);

    // 状态文字
    char status[256];
    if (has_actual_h && has_actual_h_2 && has_ik_h && has_ik_h_2) {
      double err_hx = actual_hx - target_Hx_local;
      double err_hz = actual_hz - target_Hz;
      double err_hx_2 = actual_hx_2 - target_Hx_local_2;
      double err_hz_2 = actual_hz_2 - target_Hz_local_2;
      double ik_err_hx = ik_hx - target_Hx_local;
      double ik_err_hz = ik_hz - target_Hz;
      double ik_err_hx_2 = ik_hx_2 - target_Hx_local_2;
      double ik_err_hz_2 = ik_hz_2 - target_Hz_local_2;
      std::snprintf(status, sizeof(status),
        "H1 T:(%.3f, %.3f) IK:(%.3f, %.3f) A:(%.3f, %.3f) E:(%.3f, %.3f) | H2 T:(%.3f, %.3f) IK:(%.3f, %.3f) A:(%.3f, %.3f) E:(%.3f, %.3f) | Step=%.4f %s%s",
        target_Hx_local, target_Hz, ik_hx, ik_hz, actual_hx, actual_hz, err_hx, err_hz,
        target_Hx_local_2, target_Hz_local_2, ik_hx_2, ik_hz_2, actual_hx_2, actual_hz_2, err_hx_2, err_hz_2,
        step_size, ik_failed ? "| IK1 FAILED" : "", ik_failed_2 ? "| IK2 FAILED" : "");
    } else if (has_actual_h) {
      double err_hx = actual_hx - target_Hx_local;
      double err_hz = actual_hz - target_Hz;
      std::snprintf(status, sizeof(status),
        "Target H: (%.3f, %.3f) | IK H: (n/a) | Actual H: (%.3f, %.3f) | Error: (%.3f, %.3f) | ctrl: phi1=%.3f phi2=%.3f | Step=%.4f %s",
        target_Hx_local, target_Hz, actual_hx, actual_hz, err_hx, err_hz, phi1_prev, phi2_prev, step_size,
        ik_failed ? "| IK FAILED" : "");
    } else if (has_ik_h) {
      double ik_err_hx = ik_hx - target_Hx_local;
      double ik_err_hz = ik_hz - target_Hz;
      std::snprintf(status, sizeof(status),
        "Target H: (%.3f, %.3f) | IK H: (%.3f, %.3f) | Actual H: (n/a) | IK Err: (%.3f, %.3f) | ctrl: phi1=%.3f phi2=%.3f | Step=%.4f %s",
        target_Hx_local, target_Hz, ik_hx, ik_hz, ik_err_hx, ik_err_hz, phi1_prev, phi2_prev, step_size,
        ik_failed ? "| IK FAILED" : "");
    } else {
      std::snprintf(status, sizeof(status),
        "Target H: (%.3f, %.3f) | IK H: (n/a) | Actual H: (n/a) | ctrl: phi1=%.3f phi2=%.3f | Step=%.4f %s",
        target_Hx_local, target_Hz, phi1_prev, phi2_prev, step_size,
        ik_failed ? "| IK FAILED" : "");
    }
    mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport, status, nullptr, &con);

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  mjv_freeScene(&scn);
  mjr_freeContext(&con);
  mj_deleteData(d);
  mj_deleteModel(m);

#if defined(__APPLE__) || defined(_WIN32)
  glfwTerminate();
#endif

  return 0;
}

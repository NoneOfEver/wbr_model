# WBR 仓库控制代码汇报文档

> 报告范围：当前仓库中所有与控制、状态估计、系统辨识及测试相关的源码与 MATLAB 脚本。重点为 `simulate/wbr_controller_v2/` 下的运行时控制器，同时覆盖离线工具与早期 demo。

---

## 1. 总体结论

当前仓库实现了一套面向 RoboMaster 平衡轮腿机器人的 **地面平衡控制器 v2**，采用经典的 **“状态估计 → 接触安全 → LQR 平衡 + VMC 腿力 + 偏航协调 → 轮力分配”** 分层管线。核心控制器为 `WbrControllerV2`，运行在 `simulate/main.cc` 的物理循环中；其余文件主要用于离线 LQR 系统辨识、开环响应验证、闭环测试、运动学校验与交互 demo。

---

## 2. 控制与状态估计算法清单

### 2.1 运行时主控制器 `WbrControllerV2`

| 模块 | 源文件 | 作用 |
|---|---|---|
| 主调度 | `simulate/wbr_controller_v2.cc` | 每周期依次执行命令整形、运动学观测、状态估计、接触安全、LQR/横滚/偏航、腿 VMC、轮力分配 |
| 模型绑定 | `simulate/wbr_controller_v2/model/controller_binding.cc` | 绑定 MuJoCo 中的 actuator/joint/body/sensor ID，初始化状态 |
| 运动学 | `simulate/wbr_controller_v2/model/leg_kinematics.cc` | 五杆正运动学、数值雅可比、腿长/腿角速度、VMC 关节力矩 |
| 状态估计 | `simulate/wbr_controller_v2/model/state_estimator.cc` | IMU/轮速/车体速度融合，输出姿态、速度、航向、轮里程置信度 |
| 接触安全 | `simulate/wbr_controller_v2/control/contact_safety.cc` | 带 grace time 与防抖的接触状态机，输出 authority scale |
| 偏航协调 | `simulate/wbr_controller_v2/control/yaw_coordinator.cc` | 偏航率指令整形、腿分叉参考、预测安全 authority |
| LQR 增益调度 | `simulate/wbr_controller_v2/control/lqr_schedule.cc` | 腿长相关的离散 LQR 增益查表与线性插值 |
| 轮力分配 | `simulate/wbr_controller_v2/control/wheel_allocator.cc` | 平衡/偏航力矩到左右轮电机力矩的分配与限幅 |
| 公共工具 | `simulate/wbr_controller_v2/common/controller_math.cc` | Clamp、MoveTowards、FadeAuthority、接触力查询、四元数转欧拉角 |
| 配置参数 | `simulate/wbr_controller_v2/common/controller_config.h` | 全部几何、增益、限幅、滤波时间常数 |
| 遥测类型 | `simulate/wbr_controller_v2/common/controller_types.h` | 接触状态枚举与 `WbrControllerV2Telemetry` |

---

### 2.2 各算法详细说明

#### 1) 命令整形（Command Shaping）

- **位置**：`simulate/wbr_controller_v2.cc:22-46`
- **输入**：`target_leg_length`、`target_leg_angle`、键盘/上层给出的 `target_linear_velocity_`、`target_yaw_rate_`
- **输出**：经限幅与速率限制后的 `commanded_linear_velocity_`、`commanded_yaw_rate_`
- **算法**：
  - `Clamp` 限幅腿长 `[0.15, 0.4] m`、腿角 `[-0.6, 0.6] rad`
  - `MoveTowards` 对线速度加减速率限制（加速 `0.6 m/s²`，紧急减速 `1.5 m/s²`）
  - 偏航率受持续偏航上限 `5 rad/s` 约束，并区分加速/制动速率
- **关键参数**：`kMaxLinearVelocity=0.6`、`kMaxYawRate=13`、`kSustainedYawRateLimit=5`、`kLinearAccelerationLimit=0.6`、`kYawCommandAcceleration=1.0`、`kYawCommandDeceleration=3.0`

#### 2) 五杆腿运动学（Five-Bar Leg Kinematics）

- **位置**：`simulate/wbr_controller_v2/model/leg_kinematics.cc:10-98`
- **输入**：关节角度 `phi1=q_D`、`phi2=q_B`、分支 `branch`
- **输出**：足端 H 点坐标 `(hx, hz)`、腿长 `length`、腿角 `angle=atan2(hx, -hz)`、速度、数值雅可比 `J`
- **算法**：基于 B-D 距离与圆交点的解析正运动学，双分支选择；再用中心差分求 `J`
- **关键参数**：`kLengthAB=0.0945`、`kLengthBC=0.1125`、`kLengthCD=0.116`、`kLengthAD=0.090`、`kLengthAG=0.210`、`kLengthGH=0.250`、`kJacobianStep=1e-6`

#### 3) 状态估计器 `StateEstimator`

- **位置**：`simulate/wbr_controller_v2/model/state_estimator.cc:17-149`
- **输入**：
  - IMU：`imu_gyro`、`imu_accelerometer`、`imu_quaternion`（site）
  - 轮速：`wheel_speed_1/2`（jointvel sensor）
  - 车体：plate body 速度、`body_wheel[2]` 位置
  - 接触：`wheel_grounded[2]`、时间 `control_dt`
- **输出**（`StateEstimate`）：
  - 姿态 `roll/pitch/yaw`
  - 角速度 `roll_rate/pitch_rate/yaw_rate`
  - 前进方向与横向向量
  - 前向位置 `x`、速度 `x_speed`
  - 轮里程速度 `wheel_odometry_x_speed`、置信度 `wheel_odometry_confidence`
  - 轮速差 `wheel_speed_difference`、轮距 `track_width`
- **算法**：
  - IMU 积分 + 姿态测量修正（一阶低通，时间常数 0.03 s）
  - 车体前向速度用 `mj_objectVelocity` 得到，并用 chassis velocity 校正（`kChassisVelocityCorrectionTimeConstant=0.02`）
  - 前向速度通过轮里程以置信度加权修正（`kWheelOdometryCorrectionTimeConstant=0.08`）
  - 轮里程置信度 = 接触置信度 × 滑移置信度（`FadeAuthority` 依据车体速度与轮速差）
  - `track_width` 由两轮实际间距在横向投影得到

#### 4) 接触安全状态机 `ContactSafetyMachine`

- **位置**：`simulate/wbr_controller_v2/control/contact_safety.cc:17-83`
- **输入**：原始轮地接触 `grounded[2]`、`roll`、`pitch`、`control_dt`
- **输出**：`ContactSafetyOutput`
  - 有效接触 `effective_grounded[2]`（含 grace time）
  - 接触状态 `state`：`kDualSupport / kSingleSupportFirst / kSingleSupportSecond / kAirborne / kRecovery`
  - `authority`：LQR 作用权值
- **算法**：
  - 每个轮子有 `kContactGraceTime=0.08 s` 的 grace time
  - 状态切换需满足去抖时间：接触丢失 `0.050 s`、恢复 `0.020 s`
  - 从非双触进入双触时先进入 `Recovery`，持续 `0.15 s` 且姿态满足阈值后才回到 `DualSupport`
  - 单支撑时 authority = `kSingleSupportLqrScale=0.0`（目前关闭 LQR），离地时为 0

#### 5) LQR 平衡控制器（增益调度）

- **位置**：`simulate/wbr_controller_v2.cc:376-498`（在线计算），`simulate/wbr_controller_v2/control/lqr_schedule.cc`（增益表）
- **状态**：`[theta, dtheta, x, dx, phi, dphi]`
  - `theta`：双腿合成世界腿角（相对竖直方向）
  - `phi`：车体俯仰角
  - `x`：前向位置（相对参考点）
- **输入**：`state_error[6]`、`leg_length`
- **输出**：`total_wheel_torque`（总轮力矩）、`total_leg_angle_torque`（总腿角力矩）
- **算法**：
  - 按当前平均腿长查表并线性插值得到 2×6 增益 `K(L)`
  - `u = -K * state_error`，其中 `u = [T_wheel_total, T_leg_angle_total]`
  - 结果乘以 `contact_authority_scale`
  - 限幅：`kTotalWheelTorqueLimit=10 N·m`、`kTotalLegAngleTorqueLimit=20 N·m`
- **关键参数**：增益表覆盖腿长 `0.10–0.40 m`，离线 DARE 求解得到（见 2.3）

#### 6) 腿 VMC / 腿长度与角度阻抗控制

- **位置**：`simulate/wbr_controller_v2/model/leg_kinematics.cc:100-123`
- **输入**：
  - 单腿运动学 `leg`
  - 目标腿长 `target_leg_length`
  - 支撑前馈 `support_feedforward`
  - 积分力 `integral_force`
  - 腿角力矩 `leg_angle_torque`
  - 滤波后的腿长速度 `filtered_leg_speed`
- **输出**：`act_q_B`、`act_q_D` 电机力矩
- **算法**：
  - 轴向力 `F = Kp*(L_target - L) - Kd*dL + F_feedforward + F_integral`
  - 切向力 `F_tangential = leg_angle_torque / L`
  - 合成足端力后通过 `τ = J^T F` 映射到关节力矩
  - 限幅：`kLegForceLimit=150 N`、`kJointTorqueLimit=20 N·m`
- **关键参数**：`kLegLengthKp=300`、`kLegLengthKi=400`、`kLegLengthKd=20`、`kLegIntegralForceLimit=60`

#### 7) 腿长积分抗饱和

- **位置**：`simulate/wbr_controller_v2.cc:346-374`
- **输入**：腿长误差、支撑因子 `support_factor`、腿长速度
- **输出**：`integral_force[2]`
- **算法**：仅当 `support_factor > 0.5` 时积分；积分项带限幅，并采用条件积分（避免饱和时继续积分）；离地时按指数衰减

#### 8) 横滚控制器（Roll Differential Force）

- **位置**：`simulate/wbr_controller_v2.cc:331-340`
- **输入**：`roll`、`roll_rate`
- **输出**：左右腿轴向力差值，分别叠加到两腿支撑前馈
- **算法**：`F_diff = Clamp(-kRollForceKp*roll - kRollForceKd*roll_rate, ±kRollDifferentialForceLimit)`
- **关键参数**：`kRollForceKp=67`、`kRollForceKd=30`、`kRollDifferentialForceLimit=40 N`

#### 9) 腿分叉 PD 控制器（Differential Leg Angle）

- **位置**：`simulate/wbr_controller_v2.cc:289-297`、`520-535`
- **输入**：两腿角度差 `differential_leg_angle_error`、角速度差 `differential_leg_angle_rate`
- **输出**：`differential_leg_angle_torque`
- **算法**：PD + 与偏航力矩的解耦叠加（`kDecoupledYawLegTorquePerCommand`）
- **关键参数**：`kDifferentialLegAngleKp=16`、`kDifferentialLegAngleKd=1.5`、力矩限幅 `20 N·m`

#### 10) 偏航协调器 `YawCoordinator`

- **位置**：`simulate/wbr_controller_v2/control/yaw_coordinator.cc:17-115`
- **输入**：
  - 指令偏航率 `commanded_yaw_rate`
  - 测量偏航率 `measured_yaw_rate`
  - 腿分叉角度/速度
  - 横滚/俯仰姿态
  - 两轮法向力 `normal_force[2]`
- **输出**：
  - `coordinated_yaw_rate`：经安全整形后的目标偏航率
  - `split_reference`：腿分叉角度参考
  - `authority`：综合偏航 authority（0~1）
  - 预测量：`predicted_split_error`、`predicted_roll`、`predicted_normal_force`
- **算法**：
  - 偏航率指令通过一阶滞后跟随，制动时更快减速
  - 腿分叉参考与偏航率成正比：`kYawSplitReferencePerRate=-0.60`，限幅 `±0.14 rad`
  - 多维度安全 authority：
    - 分叉残差 authority
    - 绝对分叉角/角速度 authority
    - 姿态 authority（横滚、俯仰）
    - 接触力 authority（法向力大小与均衡度）
  - 最终 authority 取最小值

#### 11) 偏航率反馈控制器

- **位置**：`simulate/wbr_controller_v2.cc:411-457`
- **输入**：`yaw_rate_error = coordinated_yaw_rate - yaw_rate`、滤波偏航加速度
- **输出**：`total_yaw_torque`
- **算法**：`Tyaw = Kp*e_yaw_rate - Kd*yaw_acceleration`，并带制动逻辑与 authority 缩放
- **关键参数**：`kYawRateKp=4.0`、`kYawAccelerationKd=0.05`、`kTotalYawTorqueLimit=4 N·m`

#### 12) 自旋模式预留与平衡力矩限幅

- **位置**：`simulate/wbr_controller_v2.cc:463-490`
- **输入**：`total_yaw_torque`、`commanded_yaw_rate`、偏航 authority
- **输出**：受限的 `total_wheel_torque`
- **算法**：
  - 计算 spin mode blend（`0.5–2.0 rad/s` 区间）
  - 为偏航预留每轮力矩上限：`reserved = blend * (0.5*kDecoupledYawWheelInputScale*max(|Tyaw|,|cmd|) + buffer)`
  - 平衡力矩可用范围 = `2*(kPerWheelTorqueLimit - reserved)`
  - 保证偏航不会完全挤占平衡力矩
- **关键参数**：`kSpinModeEntryYawRate=0.5`、`kSpinModeFullYawRate=2.0`、`kSpinYawReservePerWheel=2.0`、`kSpinYawReserveBufferPerWheel=0.15`

#### 13) 轮力分配器 `AllocateWheelTorque`

- **位置**：`simulate/wbr_controller_v2/control/wheel_allocator.cc:10-65`
- **输入**：`balance_torque`、`yaw_torque`、`grounded[2]`、`contact_state`
- **输出**：左右轮电机力矩 `actuator_torque[2]`、实际施加的偏航力矩
- **算法**：
  - 双触：`(T_common ± T_diff)`，注意左右轮符号相反
  - 单触：仅支撑轮承担平衡力矩，偏航力矩清零
  - 离地/空翻：输出零
  - 限幅 `kPerWheelTorqueLimit=5 N·m`

#### 14) 控制噪声注入（测试用）

- **位置**：`simulate/simulate.cc:3093-3129`
- **输入**：`ctrl_noise_std`、`ctrl_noise_rate`、当前 `d->ctrl`
- **输出**：带 OU 过程噪声的 `d->ctrl`
- **算法**：Ornstein-Uhlenbeck 过程，向 keyframe ctrl 中点回归，并裁剪到 `ctrlrange`
- **关键参数**：UI 中 `Noise scale`、`Noise rate`

---

### 2.3 离线系统辨识与 LQR 设计工具

#### 1) `lqr_identification.cc` — 全模型非线性辨识

- **位置**：`lqr_identification.cc`
- **输入**：`wbr_free.xml`、目标腿长、辨识 rollout 时长
- **输出**：离散/连续 `A`、`B`、DARE 增益 `K`、可控性奇异值、非线性闭环验证结果
- **算法**：
  - 构造平衡构型并做动态修平（dynamic trim）
  - 对 6 维简化状态施加中心差分扰动，通过短时 rollout 得到离散 `A_d`、`B_d`
  - 求解离散 DARE（与 `controller.m` 中 Q/R 一致：Q=diag([1,1,500,100,5000,1])，R=diag([1,0.25])）
  - 非线性闭环测试（theta/x/phi 扰动）
- **关键参数**：`kStatePositionPerturbation=1e-3`、`kStateVelocityPerturbation=2e-2`、`kInputPerturbation=0.2`

#### 2) `reduced_identification.cc` — 简化模型 LQR

- **位置**：`reduced_identification.cc`
- **输入**：`wbr_reduced.xml`、腿长
- **输出**：`A_d`、`B_d`、离散 LQR 增益、可控性秩
- **算法**：对 `wbr_reduced.xml` 的 6 维状态用 `mjd_transitionFD` 做有限差分，再_lift/reduce_到完整状态

#### 3) `lqr_open_loop_test.cc` — 开环响应验证

- **位置**：`lqr_open_loop_test.cc`
- **输入**：`wbr_free.xml`、脉冲幅值
- **输出**：T/Tp 对 theta/x/phi 的开环加速度响应，与论文 `B` 矩阵符号对比；3D 差动轮/腿力/腿角的横滚-偏航-分叉耦合矩阵
- **算法**：中心差分脉冲响应，验证解耦关系与符号，为 `kDecoupledYawWheelInputScale`、`kRollForceKp` 等提供实验依据

#### 4) `wbr_v2_closed_loop_test.cc` — 闭环测试

- **位置**：`wbr_v2_closed_loop_test.cc`
- **输入**：`wbr_free.xml`、目标腿长、偏航测试参数
- **输出**：
  - 腿 VMC 自由检查
  - 腿长度跟踪误差
  - 单轮抬起安全切换
  - 不同姿态/推力扰动下的力矩裕量
  - 持续偏航率跟踪测试（必须满足 tracking ratio ≥ 24%）
- **算法**：直接调用 `WbrControllerV2`，注入力/姿态扰动，统计饱和、authority、位移

#### 5) `kinematics_checker.cc` — 运动学校验

- **位置**：`kinematics_checker.cc`
- **输入**：模型 XML、可选目标 H 点
- **输出**：解析五杆 FK 与 MuJoCo 实际 site/body 位置对比、等式约束残差
- **算法**：解析 FK + 阻尼牛顿 IK，对比 B/C/D/G/H 点误差

#### 6) `ik_viewer.cc` — 交互式 IK 演示

- **位置**：`ik_viewer.cc`
- **输入**：键盘控制目标 H 点（左右腿独立）
- **输出**：`act_q_B`、`act_q_D` 位置/力矩指令
- **算法**：多初始值 + 双分支阻尼牛顿 IK，角度 unwrap 保持连续

#### 7) `basic.cc` — 最简位置 demo

- **位置**：`basic.cc`
- **输入**：硬编码 `phi1=70°`、`phi2=140°`
- **输出**：`d->ctrl[act_q_B]`、`d->ctrl[act_q_D]`
- **说明**：针对 `closed_chain.xml`（position actuator，`kp=0`），直接写关节目标角度

#### 8) `controller.m` — MATLAB 控制骨架

- **位置**：`controller.m`
- **内容**：
  - 符号推导 6 状态 LIP-like 动力学与 `A/B` 矩阵
  - 给定论文数值模型与 LQR（`Q/R` 同上）
  - 腿长相关增益三次多项式拟合示例
  - 五杆 FK 求 `phi2`
  - VMC 公式 `τ = J^T F`
  - 离地支持力估算
- **说明**：离线参考/教学脚本，未在 C++ 运行时直接调用

#### 9) `IKS.m` — MATLAB 交互 IK 可视化

- **位置**：`IKS.m`
- **输入**：滑条目标 Hx/Hy
- **输出**：多解 IK、选择最近解、实时连杆动画
- **说明**：纯 MATLAB 可视化工具

---

## 3. 控制器与估计器连接关系

```text
┌─────────────────────────────────────────────────────────────────────┐
│                         WbrControllerV2::Apply                      │
└─────────────────────────────────────────────────────────────────────┘
                                  │
        ┌─────────────────────────┼─────────────────────────┐
        ▼                         ▼                         ▼
  Command Shaping         Leg Kinematics            StateEstimator
  (vel/yaw rate limits)   (L, dL, angle, J)         (roll/pitch/yaw,
        │                         │                  x, dx, confidence)
        │                         │                         │
        │                         ▼                         ▼
        │                  Leg Length Integral       ContactSafetyMachine
        │                         │                  (effective grounded,
        │                         │                   state, authority)
        │                         │                         │
        ▼                         ▼                         ▼
  LQR Balance          Leg VMC (axial + tangential)   YawCoordinator
  (T_wheel, Tp_leg)        (joint torques)             (coordinated yaw,
        │                         │                    split ref, authority)
        │                         │                         │
        │                         ▼                         ▼
        │              Differential Leg Angle PD      Yaw Rate Feedback
        │              (split torque)                 (total_yaw_torque)
        │                         │                         │
        └─────────────────────────┼─────────────────────────┘
                                  ▼
                     Wheel Allocator
              (balance + yaw → left/right wheel torque)
                                  │
                                  ▼
                          d->ctrl[actuators]
```

### 数据流关键点

1. **命令层**：`simulate/main.cc` 通过键盘 W/S/A/D 设置 `linear_velocity` 与 `yaw_rate`；UI 滑条设置 `target_leg_length` / `target_leg_angle`（`simulate/simulate.h:252-253`）。
2. **状态估计**：所有反馈控制（LQR、横滚、偏航）均依赖 `StateEstimator`；轮里程仅在双轮接触且滑移小时高置信度。
3. **接触安全**：直接决定 LQR 是否激活、偏航力矩是否清零、单腿前馈如何切换。
4. **LQR → VMC**：LQR 输出的总腿角力矩取反并均分到两腿，再与 PD 腿分叉力矩相加，最终通过 `J^T F` 映射到 `q_B/q_D` 电机。
5. **偏航 → 腿分叉 → 轮力**：`YawCoordinator` 产生腿分叉参考；PD 控制器跟踪该参考产生差动力矩；同时偏航反馈产生总偏航力矩，由 `WheelAllocator` 分配到左右轮。
6. **力矩优先级**：先满足平衡 LQR，再为偏航预留裕量，最后剩余力矩用于偏航差动。

---

## 4. 控制与状态估计架构

### 4.1 架构层次

| 层级 | 代表文件 | 职责 |
|---|---|---|
| 上层交互 | `simulate/main.cc`、`simulate/simulate.h` | 键盘/鼠标命令、UI 滑条、噪声注入、调用控制器 |
| 主控制器 | `simulate/wbr_controller_v2.cc/.h` | 1 ms 周期调度、参考管理、遥测汇总 |
| 公共层 | `common/controller_config.h`、`common/controller_math.cc` | 参数、数学工具、传感器读取 |
| 模型层 | `model/leg_kinematics.cc`、`model/state_estimator.cc`、`model/controller_binding.cc` | 运动学、状态估计、MuJoCo 绑定 |
| 控制层 | `control/contact_safety.cc`、`control/yaw_coordinator.cc`、`control/lqr_schedule.cc`、`control/wheel_allocator.cc` | 安全状态机、偏航协调、LQR 增益、力矩分配 |
| 离线工具 | `lqr_identification.cc`、`reduced_identification.cc`、`lqr_open_loop_test.cc`、`wbr_v2_closed_loop_test.cc` | 系统辨识、LQR 设计、解耦验证、闭环测试 |
| Demo/校验 | `basic.cc`、`ik_viewer.cc`、`kinematics_checker.cc`、`controller.m`、`IKS.m` | 位置 demo、交互 IK、运动学校验、MATLAB 参考 |

### 4.2 依赖方向

```
common ──► model
common ──► control
model  ──► controller
control ──► controller
main    ──► controller
```

模块之间单向依赖，`model` 与 `control` 不反向访问 `WbrControllerV2` 内部状态。

---

## 5. 关键参数汇总

| 类别 | 参数 | 典型值 | 位置 |
|---|---|---|---|
| 几何 | 五杆杆长 AB/BC/CD/AD/AG/GH | 见 `controller_config.h:7-12` | `controller_config.h` |
| 腿 VMC | Kp/Ki/Kd | 300 / 400 / 20 | `controller_config.h:20-22` |
| LQR | Q/R | diag([1,1,500,100,5000,1]) / diag([1,0.25]) | `lqr_schedule.cc`、`controller.m` |
| 横滚 | Kp/Kd | 67 / 30 | `controller_config.h:35-36` |
| 偏航 | Kp/Kd | 4.0 / 0.05 | `controller_config.h:40-41` |
| 力矩限幅 | 轮/腿关节/总腿角 | 5 / 20 / 20 N·m | `controller_config.h:25,31-32,48` |
| 接触 | grace/debounce/recovery | 0.08 / 0.05–0.02 / 0.15 s | `controller_config.h:56-59` |
| 估计 | chassis/odometry 校正 | 0.02 / 0.08 s | `controller_config.h:55,66` |

---

## 6. 未来优化方向

### 6.1 控制算法

1. **非线性/时变 LQR / MPC**
   - 当前 LQR 在小扰动线性化点工作，大推力、单轮离地、高速偏航时线性假设失效。可引入 **TVLQR** 或 **MPC**，在预测时域内直接优化足端力、轮力矩与接触力分配。
2. **MIMO 横滚-偏航-腿分叉联合控制**
   - 目前横滚、偏航、腿分叉为三条独立 SISO 回路，并通过 authority 软切换。可建立 3D 耦合模型，设计 MIMO 控制器统一处理 roll/yaw/split。
3. **自适应/在线参数辨识**
   - 腿质量、摩擦、接触刚度会随地面变化。可在线辨识 `A/B` 或腿部阻抗参数，动态调整 LQR 增益。
4. **优化力分配 QP**
   - 当前 VMC 使用固定雅可比映射，未显式考虑地面摩擦锥。可加入 **摩擦锥约束 QP**，在保证平衡/偏航需求的同时避免打滑。

### 6.2 状态估计

1. **卡尔曼/互补滤波融合**
   - 当前为简单的一阶修正。可用 **EKF/UKF** 同时估计 IMU 零偏、接触滑移、轮半径变化。
2. **力/触觉感知**
   - 目前接触仅依赖 geom 碰撞检测。加入接触力估计或力传感器可更早检测离地/打滑。
3. **视觉/里程计融合**
   - 轮里程置信度低时漂移严重，可融合视觉或激光里程计提高长距离定位精度。

### 6.3 工程实现

1. **消除测试代码中的临时分配**
   - `lqr_identification.cc` 等离线工具每步大量 `std::vector` 分配，可改为预分配或 Eigen。
2. **实时性与确定性**
   - 当前控制器在 `main.cc` 的物理线程中运行，但 `StateEstimator` 内部依赖 MuJoCo 传感器。可考虑硬实时调度与日志/控制分离。
3. **统一坐标与单位**
   - `closed_chain.xml` 使用角度制，`wbr_free.xml` 使用弧度制，`basic.cc` 与 `wbr_controller_v2` 目标不同。建议统一模型约定并显式注释。
4. **可配置化**
   - 当前参数全部硬编码在 `controller_config.h`。可改为 JSON/YAML 运行时配置，便于调参与 A/B 测试。

### 6.4 测试与验证

1. **系统化回归测试**
   - `wbr_v2_closed_loop_test` 已覆盖多种扰动，可加入 **噪声注入测试**、**不同地面摩擦测试**、**质心偏移测试**。
2. **Sim-to-real 对齐**
   - 离线辨识模型与实际机器人存在差距。建议采集真实机器人数据，对比辨识 `A/B`，迭代模型参数。
3. **安全边界量化**
   - 当前 authority 为经验阈值。可通过 **可达集/不变集分析** 量化稳定域，给出可证明的安全包络。

---

## 7. 附录：文件与行号速查

| 功能 | 文件 | 关键行号 |
|---|---|---|
| 主控制循环 | `simulate/wbr_controller_v2.cc` | 15–604 |
| 命令整形 | `simulate/wbr_controller_v2.cc` | 22–46 |
| LQR 计算 | `simulate/wbr_controller_v2.cc` | 376–498 |
| 腿长积分 | `simulate/wbr_controller_v2.cc` | 346–374 |
| 横滚控制 | `simulate/wbr_controller_v2.cc` | 331–340 |
| 偏航反馈 | `simulate/wbr_controller_v2.cc` | 411–457 |
| 运动学/VMC | `simulate/wbr_controller_v2/model/leg_kinematics.cc` | 10–123 |
| 状态估计 | `simulate/wbr_controller_v2/model/state_estimator.cc` | 17–149 |
| 接触安全 | `simulate/wbr_controller_v2/control/contact_safety.cc` | 17–83 |
| 偏航协调 | `simulate/wbr_controller_v2/control/yaw_coordinator.cc` | 17–115 |
| 轮力分配 | `simulate/wbr_controller_v2/control/wheel_allocator.cc` | 10–65 |
| LQR 增益表 | `simulate/wbr_controller_v2/control/lqr_schedule.cc` | 15–102 |
| 公共数学 | `simulate/wbr_controller_v2/common/controller_math.cc` | 7–87 |
| 参数配置 | `simulate/wbr_controller_v2/common/controller_config.h` | 7–122 |
| 物理循环 | `simulate/main.cc` | 324–498 |
| 噪声注入 | `simulate/simulate.cc` | 3093–3129 |
| 离线 LQR 辨识 | `lqr_identification.cc` | 1–1902 |
| 简化模型辨识 | `reduced_identification.cc` | 1–454 |
| 开环解耦测试 | `lqr_open_loop_test.cc` | 1–632 |
| 闭环测试 | `wbr_v2_closed_loop_test.cc` | 1–724 |
| 运动学校验 | `kinematics_checker.cc` | 1–325 |
| 交互 IK | `ik_viewer.cc` | 1–672 |
| 简单位置 demo | `basic.cc` | 72–152 |
| MATLAB 参考 | `controller.m`、`IKS.m` | 全部 |

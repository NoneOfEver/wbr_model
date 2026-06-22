# WBR controller v2 architecture

The controller is organized as a one-way control pipeline:

1. `WbrControllerV2::Apply` reads commands and MuJoCo observations.
2. State estimation converts IMU, wheel and contact data into controller states.
3. Contact and yaw-authority logic limits control authority.
4. LQR, leg VMC and yaw control produce generalized torque requests.
5. The wheel allocator converts generalized balance/yaw requests to actuator
   torques and enforces contact-dependent limits.

Directory layers:

- `common/`: dependency-free controller configuration, shared types and scalar
  utilities.
- `model/`: MuJoCo-facing observation and five-bar mechanism models.
- `control/`: state machines, coordination and actuator allocation.

Modules:

- `common/controller_config.h`: immutable geometry, gains, limits and identified
  decoupling parameters.
- `common/controller_types.h`: public telemetry and contact-state data shared
  with tests/UI.
- `common/controller_math.*`: scalar shaping, orientation conversion, sensors and
  contact queries.
- `model/leg_kinematics.*`: five-bar forward kinematics, Jacobian and leg VMC.
- `model/state_estimator.*`: IMU orientation/rate handling, body-forward
  velocity fusion, wheel odometry confidence and track-width observation.
- `model/controller_binding.cc`: MuJoCo model binding, controller reset and
  initial target synchronization.
- `control/contact_safety.*`: contact grace, debounced support transitions and
  recovery authority.
- `control/lqr_schedule.*`: leg-length-dependent gain schedule and
  interpolation.
- `control/yaw_coordinator.*`: leg-split reference, predictive safety authority
  and coordinated yaw-rate shaping.
- `control/wheel_allocator.*`: wheel torque allocation and actuator writes.
- `../wbr_controller_v2.cc`: one-cycle orchestration and high-level control
  composition.

Dependency direction is intentionally one-way: `common` → `model`/`control` →
controller orchestration. Low-level modules own their state and do not access
`WbrControllerV2` internals.

Behavioral changes must pass `wbr_v2_closed_loop_test`, including the
interaction-equivalent sustained-yaw test, before being used in `simulate`.
Ground Balance is the controller's only runtime mode; diagnostic tests may
disable individual feedback paths without introducing another mode.

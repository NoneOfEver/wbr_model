# Controller migration status

## Migrated and shared

The following sources build into `wbr_control_core` without MuJoCo or Zephyr
headers and are consumed by the simulator today:

- controller parameters and scalar math;
- cubic LQR gain scheduling;
- five-bar forward kinematics, numerical Jacobian and VMC torque calculation;
- contact safety state machine;
- yaw authority coordinator;
- wheel balance/yaw torque allocation;
- the complete Ground Balance cycle, including command shaping, LQR
  composition, leg integrators, yaw torque slew and six-motor torque output;
- fixed-size firmware I/O contracts.
- raw IMU/motor observation building, complementary roll/pitch estimation,
  wheel odometry, contact-confidence mapping and freshness checks;
- a dedicated Zephyr 1 kHz timer/semaphore control thread with fail-safe motor
  disable behavior.

## MuJoCo platform layer

`wbr_mujoco_platform` currently owns:

- sensor, quaternion and contact extraction;
- MuJoCo-specific state estimation;
- joint position/velocity conversion and actuator writes;
- model-name and actuator/joint binding.

`simulate/wbr_controller_v2.cc` is now a thin compatibility facade: it builds a
processed `GroundBalanceInput`, calls the portable controller once and writes
the returned six-motor command.

## Remaining integration

The portable firmware path is complete through six normalized motor torque
commands. Board-specific implementations are still required for IMU capture,
motor CAN feedback/commands, contact confidence, operator commands, monotonic
time and robot parameter loading. The checked-in stub intentionally fails
initialization, so it cannot enable motors accidentally.

## Acceptance gates for the next step

1. Implement and unit-test the board hardware port.
2. Capture real idle/suspended sensor logs and replay them on the host.
3. Replay identical vectors through desktop and target builds.
4. Verify all normalized motor/IMU signs with motors suspended.
5. Run the staged hardware bring-up in `docs/ZEPHYR_PORTING.md`.

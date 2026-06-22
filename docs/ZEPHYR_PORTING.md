# Zephyr integration guide

## Hardware boundary

The firmware depends only on the port contracts under
`firmware/wbr_zephyr/include/wbr/firmware/`. Board, IMU and CAN implementations
belong under `firmware/wbr_zephyr/port/` and are intentionally absent from the
portable controller.

All values crossing the boundary use SI units:

- position: rad
- velocity: rad/s
- acceleration: m/s^2
- torque: N.m
- time: microseconds at the interface, seconds inside the controller

Encoder multi-turn expansion, motor direction, CAN IDs and IMU mounting
rotation are port responsibilities. Gear geometry and controller signs are
robot parameters, not driver constants.

### Coordinate convention

Hardware feedback passed to the controller must use robot-normalized axes:

- body X points forward, Y points left and Z points up;
- IMU angular velocity is `[roll_rate, pitch_rate, yaw_rate]`;
- acceleration is expressed in that body frame and reads approximately
  `[0, 0, +g]` while upright and stationary;
- both wheel velocities are positive when driving the robot forward;
- left and right B/D joint positions use the same five-bar positive direction.

Motor commands returned by the controller use the same normalized convention.
Mirrored installation signs must be applied exactly once in the CAN port. The
MuJoCo adapter demonstrates this conversion with its actuator-sign table.

## Required port functions

Implement the declarations in `hardware_interface.h`:

- initialize sensors and the motor bus;
- load mass, gravity, wheel radius, estimator and freshness parameters;
- capture one coherent IMU and six-motor feedback snapshot;
- submit one six-motor torque command without blocking the control thread;
- disable all motors on a fault;
- provide a monotonic microsecond clock;
- read the latest operator command.

## Real-time execution

Run the controller from a dedicated high-priority 1 kHz thread. A timer or
data-ready ISR should only timestamp data and release a semaphore. Do not run
the controller in an ISR or the system workqueue.

The control thread must not allocate memory, format logs, write flash or wait
for CAN responses. Telemetry is copied into a bounded lock-free/ring buffer and
serialized by a lower-priority thread.

The checked-in control thread rejects a cycle and disables all motors when an
IMU or motor sample is invalid, is timestamped in the future, is older than
5 ms, a control period falls outside 0.2--3 ms, leg kinematics is invalid, the
operator command is disabled/stale, or command submission fails. Wheel contact
confidence is normalized to `[0, 1]`; values at or above `0.5` are treated as
grounded by the default robot parameters.

## Bring-up order

1. Build and run host golden-vector tests.
2. Run the same vectors on the MCU without enabling motors.
3. Verify timestamp, axis, sign and unit conversions.
4. Enable suspended motors with conservative current limits.
5. Verify each joint direction and torque conversion.
6. Land with yaw disabled and reduced wheel torque.
7. Restore low-speed translation and yaw.
8. Increase yaw rate in bounded steps only after watchdog and emergency stop
   tests pass.

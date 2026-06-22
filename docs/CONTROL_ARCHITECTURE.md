# WBR control architecture

## Goal

The same deterministic control algorithm must run in MuJoCo and on a Zephyr
target. Platform code acquires normalized observations and applies motor
commands; it must not contain control laws, gain scheduling, VMC, safety
coordination or actuator allocation.

## Dependency rule

```text
application (simulate / Zephyr)
              |
              v
platform adapter (MuJoCo / hardware port)
              |
              v
control_core (no platform headers)
```

`control_core` must not include `mujoco.h`, Zephyr headers, CAN drivers or
sensor-vendor headers. It uses fixed-size value types, performs no dynamic
allocation and owns all state that affects controller output.

## One control cycle

1. The platform captures one timestamped IMU/motor snapshot.
2. The platform converts signs and units to SI units.
3. State/contact estimation creates a `ControlInput`.
4. The controller computes six requested motor torques.
5. The safety supervisor validates freshness and limits the command.
6. The platform writes the command to MuJoCo actuators or the motor bus.

The platform may reject an invalid command, but it may not silently add a
second controller or a second torque allocator.

## Ownership

- `control_core/`: kinematics, estimation, LQR, leg/yaw control, contact
  safety, allocation and controller state.
- `platform/mujoco/`: model-name binding, sensor/contact extraction and
  `data->ctrl` writes.
- `firmware/wbr_zephyr/`: 1 kHz scheduling, snapshot exchange, watchdog,
  telemetry transport and hardware ports.
- `simulate/`: interactive application and the temporary compatibility facade
  used while the legacy controller is moved into `control_core`.
- `tests/`: host replay and target tests. Simulation tests never enter the
  firmware image.

## Migration invariant

Every extraction step must keep the host closed-loop tests passing. A module is
considered migrated only when its implementation compiles without MuJoCo and
both simulation and firmware targets consume that same source file.

Current progress and the remaining compatibility boundary are tracked in
`docs/MIGRATION_STATUS.md`.

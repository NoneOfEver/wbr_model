# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a MuJoCo-based simulation project for a closed-chain robotic mechanism (RoboMaster balance robot). It models a planar linkage with a parallelogram closed loop using MuJoCo's equality constraints.

## Build System

CMake is used with an out-of-source `build/` directory. The build requires:
- MuJoCo source and build directories (defaults to `~/Documents/mujoco_ws/mujoco` and `~/Documents/mujoco_ws/mujoco/build`)
- GLFW3
- On macOS: Cocoa, IOKit, OpenGL frameworks

### Build Commands

```bash
cd build
cmake ..
make
```

To use a different MuJoCo installation:
```bash
cmake -DMUJOCO_SRC=/path/to/mujoco -DMUJOCO_BUILD=/path/to/mujoco/build ..
```

### Run Commands

Two executables are produced:

```bash
# Minimal viewer with hardcoded actuator setpoints (from basic.cc)
./build/basic <model.xml>

# Full-featured interactive MuJoCo viewer (from simulate/)
./build/simulate <model.xml>
```

Example:
```bash
./build/basic closed_chain.xml
./build/simulate closed_chain.xml
```

## Architecture

### Model Files

- `closed_chain.xml` — Primary closed-chain model. Defines a planar linkage with two driven joints (`q_B`, `q_D`) and uses `equality/connect` constraints to close kinematic loops (C-D and G_loop-G). Also uses an `equality/tendon` constraint (`t_de_abs` vs `t_dc_abs`) to enforce the parallelogram angle relationship: `q_D + q_DE = q_B + q_BC + q_C`.
- `COD-2026RoboMaster-Balance.xml`, `humanoid.xml`, `wbr.xml` — Other robot models in the workspace.

**Important:** The `closed_chain.xml` model specifies `compiler angle="degree"`, so all angle values in XML and in `d->ctrl` are in **degrees**, not radians.

### Source Files

- `basic.cc` — Minimal GLFW-based simulation viewer. Hardcodes control targets (`phi1=70`, `phi2=140`) for `act_q_B` and `act_q_D` position actuators and steps the simulation at 60 Hz.
- `main.cpp` — Another minimal viewer with more error checking. Loads the `init` keyframe if present and runs the same hardcoded control logic.
- `simulate/` — Full-featured interactive MuJoCo viewer application (copied from MuJoCo's upstream sample apps). Contains `main.cc`, `simulate.cc/h`, GLFW adapters, and macOS-specific `.mm` files. This is a self-contained viewer with built-in UI, profiler, and drag-and-drop model loading.
- `matlab_closedchain.m` — MATLAB script that computes the closed-chain kinematics analytically (circle intersection for joint C, parallelogram for E/F, and H-point trajectory). Used to verify the MuJoCo model against analytical geometry.

### Actuators and Control

The `closed_chain.xml` model defines two position actuators:
- `act_q_B` controls joint `q_B`
- `act_q_D` controls joint `q_D`

Both have `kp="0.0"` in the model definition. The custom viewers (`basic.cc`, `main.cpp`) write target angles directly to `d->ctrl[act_id]`.

### Kinematic Chain

The mechanism structure in `closed_chain.xml` is:
- A (fixed base)
  - B branch: A → B (`q_B`) → BC (`q_BC`) → C (`q_C`) → (connect constraint to D)
  - D branch: A → D (`q_D`) → DE (`q_DE`) → E (`q_E`) → F (`q_F`) → G_loop → (connect constraint to G on AD extension)
- Parallelogram loop D-E-F-G is constrained by the tendon equality `t_de_abs == t_dc_abs`.

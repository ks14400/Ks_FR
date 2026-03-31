# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## Project Summary

Automated pick-and-place system using **Fairino FR16/FR20** cobots to load samples
into a **PXRD** (powder X-ray diffraction) instrument. The goal is a **ROS2-based
simulation workflow** where pick-and-place paths are planned with collision avoidance
using real CAD models, tested entirely in simulation, then deployed to the physical
robot by changing one IP address.

**This repo** (`Ks_Fairino`) is a local clone of the **fairino-python-sdk**, matched
to controller firmware **v3.8.6**. It is one component of the larger system.

## Current State

- SDK (`linux/fairino/Robot.py`) and pre-compiled bindings (`linux/libfairino/`) are present
- ~97 example scripts in `linux/example/`
- Robot is accessible via WebApp at its IP address
- The ROS2 workspace, simulation environment, and application code have **not been built yet**

---

## What We're Building

```
CAD files (PXRD instrument, fixtures, robot cell)
    ↓ convert STEP → DAE meshes
URDF scene (robot + instrument + fixtures)
    ↓
MoveIt2 (collision-free path planning in simulation)
    ↓
Gazebo (3D visualization and physics validation)
    ↓
When validated → connect to real robot (same code, different IP)
```

We are **not** scripting motions via the Python SDK or WebApp directly. All motion
planning goes through ROS2/MoveIt2 with collision avoidance against real CAD geometry.

---

## Phase Guide

Development is organized into five sequential phases. **Read the relevant phase doc
before working on that phase.**

| Phase | Focus | Doc |
|---|---|---|
| **1** | ROS2 + MoveIt2 + Gazebo installation | [phase1-ros2-install.md](docs/phases/phase1-ros2-install.md) |
| **2** | Fairino URDF in Gazebo + MoveIt2 planning | [phase2-robot-sim.md](docs/phases/phase2-robot-sim.md) |
| **3** | PXRD cell scene with collision meshes | [phase3-pxrd-cell.md](docs/phases/phase3-pxrd-cell.md) |
| **4** | Pick-and-place application code | [phase4-application.md](docs/phases/phase4-application.md) |
| **5** | Hardware commissioning (real robot) | [phase5-hardware.md](docs/phases/phase5-hardware.md) |

---

## Key Repos (We Don't Edit These)

| Repo | What it gives us |
|---|---|
| `FAIR-INNOVATION/frcobot_ros2` | FR16/FR20 URDF + meshes, MoveIt2 configs, `fairino_hardware` C++ plugin, `fairino_msgs` |
| `Devonics-Inc/ros2_fr_gazebo` | Gazebo Fortress simulation, mirror mode, digital twin mode |
| `FAIR-INNOVATION/fairino-python-sdk` | Python SDK (this repo) — used for direct connection verification only |

**Our package** (`pxrd_cell`) is the only code we write — it adds the PXRD instrument
to the scene, configures collision meshes, and runs the pick-and-place job.

---

## Core Architecture

```
Job YAML → job_runner.py (Python) → MoveIt 2 action client
  → MoveIt 2 planner (with PXRD collision mesh) → ros2_control
  → fairino_hardware (C++ plugin — never edit) → TCP/IP :8080
  → SimMachine / real controller → Robot arm + gripper
```

---

## Non-Negotiable Rules

1. **Never edit** `fairino_hardware/`, `fairino_description/`, or upstream repos
2. **Never hardcode** robot IP, waypoints, or speeds — use config/YAML/env vars
3. **Always load** the PXRD collision mesh before any motion plan
4. **Always validate** in Gazebo simulation before connecting to real robot
5. **Speed limit** near PXRD instrument: ≤ 50 mm/s
6. **Firmware matching**: SDK, `frcobot_ros2`, and `ros2_fr_gazebo` must match v3.8.6
7. **YAML discipline**: never modify a YAML that has run on hardware — duplicate it
8. **Only Python and config files** are authored. C++ hardware interface is pre-built.

---

## Hardware Quick Facts

| | FR16 | FR20 |
|---|---|---|
| Payload | 16 kg | 20 kg |
| Reach | 1034 mm | 1854 mm |
| Max TCP speed | **1 m/s** | **2 m/s** |
| Repeatability | ±0.03 mm | ±0.03 mm |

- SDK ports: 20003 (XML-RPC commands), 20004 (state feedback ~10 Hz)
- ros2_control ports: 8080 (commands), 8083 (status ~10 Hz)
- Default IP: `192.168.58.2`
- Gripper: head I/O DO[0]=open, DO[1]=close
- Firmware: v3.8.6 (no EtherCAT — use Modbus or TCP/IP)

---

## Official Manual Reference (docs/fr.pdf)

The full Fairino manual (v3.9.3, 2707 pages) is at `docs/fr.pdf`. Key chapters:

| Chapter | Content | Relevant Phase |
|---|---|---|
| Ch 2 | SDK Manual (C++, Python, C#, Lua) — full API reference | Phase 5 |
| Ch 7 | frcobot_ros (ROS1 Noetic) — reference only | — |
| Ch 8 | frcobot_ros2 (ROS2 Humble, fairino_hardware plugin) | Phase 1-2 |
| Ch 9 | MoveIt2 (plugin setup, MTC pick-and-place demo) | Phase 2-3 |
| Ch 16 | SimMachine (VMware & Docker) | Phase 5 |

---

## Key References

| Resource | URL |
|---|---|
| Official docs (latest) | https://fairino-doc-en.readthedocs.io/latest/ |
| Python SDK releases | https://github.com/FAIR-INNOVATION/fairino-python-sdk/releases |
| ROS2 package releases | https://github.com/FAIR-INNOVATION/frcobot_ros2/releases |
| Devonics Gazebo repo | https://github.com/Devonics-Inc/ros2_fr_gazebo |
| Devonics support | https://support.devonics.com |
| SimMachine / downloads | https://fairino.support |
| CAD models / 3D STEP | https://www.fairino.com/DOWNLOAD2 |
| URDF files | https://inluxrobotics.eu/pages/tech-support |

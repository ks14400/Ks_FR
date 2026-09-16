# CLAUDE.md

Guidance for Claude Code when working in this repository.

---

## What this repo is

Two ROS2/MoveIt2 robot cells that load 96-well plates into lab instruments
with Fairino cobots, plus the shared plumbing to run the SAME code in
simulation and on real hardware (switch = launch args, not code):

| Cell | Robot | Instrument | Status |
|---|---|---|---|
| **unchained** (`unchained_cell`) | FR16 + DH AG-145 gripper | Unchained Junior (open decks) | **Hardware-proven** — full pick-and-place incl. real gripper |
| **pxrd** (`pxrd_cell`) | FR10 + DH AG-145 gripper | Rigaku SmartLab PXRD (enclosed bay) | **Sim-validated round trip** — hardware bring-up pending |

Everything lives in one colcon workspace: `ros2_ws/`.

## Quick start

```bash
# PXRD cell (domain 43):
./run_pxrd.sh sim          # terminal 1 — sim + rviz
./run_pxrd.sh load         # terminal 2 — plate: table -> PXRD
./run_pxrd.sh retrieve     #              plate: PXRD -> table
./run_pxrd.sh recover      # stranded-arm rescue (after aborts)

# Unchained cell (domain 42):
./run_unchained.sh sim                              # sim + rviz
./run_unchained.sh pick deck_9_10_pos1 table_top    # pick-and-place
# hardware: see docs/unchained-cell.md (bridge + hw-scene + --hardware)
```

The two cells run on isolated ROS domains (42/43) — both sims can run
side by side with zero interference. The runner scripts set the domain;
for manual terminals use `source ros2_ws/env_pxrd.sh` / `env_unchained.sh`
(or the `pxrd` / `unchained` bash functions).

## Repo map

```
run_pxrd.sh / run_unchained.sh   ← START HERE: one-command launchers
ros2_ws/
  src/pxrd_cell/       PXRD cell (Python + config; we own this)
  src/unchained_cell/  Unchained cell (Python + config; we own this)
  src/fr_bridge/       Cell-agnostic hardware bridge (sim topics -> Fairino SDK)
  src/ros2_fr_gazebo/  Vendored upstream (fairino descriptions/moveit cfgs) — DO NOT EDIT
  clean_sim.sh         Kill all cell ROS nodes (run before every launch)
  env_pxrd.sh / env_unchained.sh   Domain isolation
cad/                   Source CAD (originals; runtime meshes live in each pkg)
docs/
  pxrd-cell.md         PXRD cell: architecture, validated params, debugging
  unchained-cell.md    Unchained cell: hardware-proven params + procedures
  phases/              Historical phase docs (original build plan)
linux/                 Fairino Python SDK (v3.8.6) — reference/bridge dependency
scripts/               One-off conversion/test scripts (mostly historical)
```

## Architecture (both cells)

```
pick_place.py (phase orchestrator, budget gates, telemetry)
  -> MoveIt2 (move_group): OMPL joint goals + collision-checked Cartesian
  -> ros2_control:  mock controllers (sim)  |  fr_bridge sdk_executor (hardware)
  -> real robot: Fairino SDK over TCP (bridge bypasses the buggy C++ plugin)
```

- The **plate** is a MoveIt CollisionObject managed by pick_place (attach to
  deck ↔ gripper with touch_links); in the PXRD cell it is the real tabbed
  mesh, rigid-TF attached.
- **Budget gates** reject crazy plans before execution (max joint travel /
  per-joint / per-step).
- **Hardware = same script + `--hardware`** with the bridge running; real
  arm speed is the bridge's `movej_vel_pct`, not `--vel` (planning-only).

## Non-negotiable rules

1. **Never edit** `ros2_ws/src/ros2_fr_gazebo/` or other vendored upstream code
2. **Never hardcode** robot IP / waypoints / speeds in code — launch args & flags
3. **Always validate in sim before hardware** — same command minus `--hardware`
4. **Never weaken collision checking to make a motion pass** — masked
   SRDF/touch_links entries hid real fingertip-vs-instrument collisions once;
   fix geometry or motion, not the checker
5. **Speed near instruments** ≤ 50 mm/s on hardware (bridge `movej_vel_pct` low)
6. **Firmware matching**: SDK & vendored pkgs match controller v3.8.6
7. **Never modify a YAML/param set that has run on hardware — duplicate it**
8. We author **Python + config only**; C++ interfaces stay vendored

## Debugging playbook (earned the hard way)

- Pose mysteriously unplannable? `ros2 run pxrd_cell probe_pose --x .. --y .. --z ..`
  → prints reachability + EXACT colliding link pair. Never guess.
- Cartesian fraction < 1.0: real collision vs solver artifact →
  GetCartesianPath with avoid_collisions True vs False (see scratchpad cart_probe).
- Path found but rejected: move_group's launch log prints
  `Found a contact between X and Y` — read it.
- Joint angles from rviz: `ros2 topic echo /joint_states --once` and pair
  name[] with position[] (order is scrambled; a mis-paired j4 sign cost a day).
- Stale sim weirdness (`-4` errors, zombie nodes): `ros2_ws/clean_sim.sh`,
  and check `ps -o stat` for `Tl`/`Z` (a Ctrl-Z'd launch looks alive to pgrep).
- IK branch nondeterminism: same TCP pose has mirror wrist families with
  totally different collision behavior — pin branches (STAGE_HINTS pattern).

## Cell-specific docs — READ BEFORE WORKING ON A CELL

- **PXRD**: [docs/pxrd-cell.md](docs/pxrd-cell.md) — enclosed-bay geometry,
  side-pinch grasp, insertion choreography, validated constants
- **Unchained**: [docs/unchained-cell.md](docs/unchained-cell.md) —
  hardware-proven parameters, bridge operation, gripper force control

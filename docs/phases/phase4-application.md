# Phase 4 — Pick-and-Place Application Code

Write the Python application that loads job YAML files, sequences MoveIt2 motion
plans, controls the gripper, and triggers the PXRD diffractometer. Everything runs
in simulation first.

---

## Why This Phase

Phases 1-3 gave us a simulated robot in a collision-aware scene. Now we write the
code that actually does the pick-and-place: move to pick position, grab sample,
move to PXRD instrument, place sample, trigger measurement, return home.

---

## Files to Write

All files go in `~/ros2_ws/src/pxrd_cell/pxrd_cell/`:

| File | Purpose |
|---|---|
| `job_runner.py` | Loads YAML, calls MoveIt2 action client, sequences steps |
| `waypoint.py` | Waypoint dataclass, validates against joint limits, unit conversion |
| `gripper_interface.py` | Wraps gripper open/close/check via ROS2 service calls |
| `pxrd_trigger.py` | Sends trigger signal to PXRD diffractometer |

**Everything is Python. No C++.**

---

## Data Flow

```
Job YAML
    ↓
job_runner.py (loads YAML, iterates sequence)
    ↓
MoveIt2 Python action client (moveit_commander or moveit_py)
    ↓
MoveIt2 move_group (plans collision-free paths using PXRD cell scene)
    ↓
ros2_control → fairino_hardware plugin → robot
```

---

## Job YAML Format

One YAML per pick-and-place task. All positions in mm and degrees.
`job_runner.py` converts to meters/radians for MoveIt2.

```yaml
job_name: pxrd_sample_01
robot_model: fr16
tool_frame: gripper_tcp
speed_linear_mms: 50                 # mm/s — max 50 near PXRD
speed_joint_pct: 20                  # % of max for transit
gripper_open_width_mm: 85
gripper_close_width_mm: 30
gripper_force_n: 20

waypoints:
  home:
    type: joint
    joints_deg: [0, -45, 90, -45, -90, 0]
  approach_pick:
    type: cartesian
    xyz_mm: [320.5, -145.2, 420.0]
    rpy_deg: [180.0, 0.0, 90.0]
  pick:
    type: cartesian
    xyz_mm: [320.5, -145.2, 385.0]
    rpy_deg: [180.0, 0.0, 90.0]
  retract_pick:
    type: cartesian
    xyz_mm: [320.5, -145.2, 420.0]
    rpy_deg: [180.0, 0.0, 90.0]
  approach_place:
    type: cartesian
    xyz_mm: [-50.0, 410.3, 380.0]
    rpy_deg: [180.0, 0.0, 0.0]
  place:
    type: cartesian
    xyz_mm: [-50.0, 410.3, 352.0]
    rpy_deg: [180.0, 0.0, 0.0]
  retract_place:
    type: cartesian
    xyz_mm: [-50.0, 410.3, 380.0]
    rpy_deg: [180.0, 0.0, 0.0]

sequence:
  - home
  - gripper_open
  - approach_pick
  - pick
  - gripper_close
  - retract_pick
  - approach_place
  - place
  - gripper_open
  - retract_place
  - pxrd_trigger
  - home
```

### YAML Rules

- Never modify a YAML that has run on hardware — duplicate and rename
- `sequence` is the single source of truth for execution order
- `robot_model` determines speed caps: FR16=1000 mm/s, FR20=2000 mm/s
- Never hardcode positions in Python — always from YAML

---

## job_runner.py — Design Notes

- Loads job YAML
- Ensures PXRD collision mesh is in the planning scene
- Iterates through `sequence`, dispatching each step:
  - Waypoint name → MoveIt2 motion plan + execute
  - `gripper_open` / `gripper_close` → `gripper_interface.py`
  - `pxrd_trigger` → `pxrd_trigger.py`
- Enforces `speed_linear_mms` cap per `robot_model`
- Robot IP from env var or config, never hardcoded

```bash
# Run a job
ros2 run pxrd_cell job_runner --ros-args -p job_file:=~/jobs/pxrd_sample_01.yaml
```

---

## waypoint.py — Design Notes

- Dataclass for joint-space and Cartesian waypoints
- Converts mm → meters, degrees → radians for MoveIt2
- Validates joint values against FR16/FR20 limits from URDF

---

## gripper_interface.py — Design Notes

- `open()`, `close()`, `is_gripping()` methods
- In simulation: publishes to a simulated gripper topic
- On real robot: calls head I/O (DO[0]=open, DO[1]=close) via
  the `/fairino_remote_command_service` ROS2 service:
  ```
  SetToolDO(0,1)   # open
  SetToolDO(1,1)   # close
  ```

---

## pxrd_trigger.py — Design Notes

- Sends trigger signal to PXRD diffractometer after sample placement
- Protocol TBD (digital I/O, Modbus, or network command)
- In simulation: logs the trigger event

---

## Reference: MTC Demo

The official `fairino_mtc_demo` package (from Phase 2) is a working pick-and-place
using MoveIt Task Constructor. Study its code structure:

```bash
# See how it sequences motions
ros2 launch fairino_mtc_demo mtc_demo_app.launch.py
```

---

## Phase 4 Completion Checklist

- [ ] `waypoint.py` — dataclass with joint limit validation and unit conversion
- [ ] `gripper_interface.py` — open/close/check (simulated + real modes)
- [ ] `pxrd_trigger.py` — trigger signal (simulated + real modes)
- [ ] `job_runner.py` — full sequence execution from YAML
- [ ] Sample job YAML created
- [ ] Full pick-place-trigger sequence runs in Gazebo simulation
- [ ] MoveIt2 plans avoid PXRD instrument collision mesh
- [ ] Speed caps enforced per robot model

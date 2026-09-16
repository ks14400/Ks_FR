# PXRD cell (`pxrd_cell`) — FR10 loading a Rigaku SmartLab

**Status (2026-09): full round trip validated in simulation** with strict
collision checking and deterministic staging. Hardware bring-up pending.

## LATEST WORKING TERMINAL COMMANDS (validated 2026-09-15, verbatim)

```bash
# Sim launch (terminal 1):
ros2 launch pxrd_cell pxrd_full.launch.py tall_pedestal:=true \
  pedestal_off_z:=0.55 pedestal_off_x:=0 pedestal_off_y:=-0.2 \
  table_x:=-0.5 table_y:=-0.9 table_h:=0.5

# Load — plate table -> PXRD (terminal 2):
ros2 run pxrd_cell pick_place --source table_top --target pxrd_sample \
  --planner RRTConnect --vel 0.1 --place-lift 0.005 --transit-time 20

# Retrieve — plate PXRD -> table:
ros2 run pxrd_cell pick_place --source pxrd_sample --target table_top \
  --planner RRTConnect --vel 0.1 --place-lift 0.005 --transit-time 20
```

These exact values are baked into `./run_pxrd.sh`. Position meanings:
- `pedestal_off_x:=0 pedestal_off_y:=-0.2 pedestal_off_z:=0.55` — pedestal
  origin offset from the PXRD origin, CAD frame (Y = height: −0.2 puts the
  945 mm pedestal's top plate 200 mm below the PXRD origin; Z = depth away
  from the instrument). SolidWorks-measured baseline was
  (0.12763, −0.6246 for the 585 pedestal, 0.41012).
- `table_x:=-0.5 table_y:=-0.9 table_h:=0.5` — staging table in WORLD frame
  (meters), table top at 0.5 m.
- Tabbed-plate rest pose (baked constant `TABBED_PLATE_POSE`): xyz
  (0.005, 0.0085, 0.0) rpy (180°, 90°, 0°) in the `pxrd_smartlab` frame —
  dialed visually, wells land on the sample origin.
- Resulting deck positions in `base_link` at this layout:
  table_top ≈ (−0.387, +0.042, −0.510), pxrd_sample ≈ (+0.113, +0.942, +0.200).

## The scene

- **Instrument**: Rigaku SmartLab PXRD. CAD origin = sample position
  (`pxrd_sample` deck marker at (0,0,0) of `pxrd_smartlab`, CAD Y-up).
  The sample sits inside an **enclosed bay**: an open slot below ~Y=330 mm,
  tall front wall at Z≈250–300 mm, roof over the pocket. This geometry
  dictates the entire motion strategy.
- **Robot**: FR10 on the tall Vention pedestal (945 mm, `tall_pedestal:=true`).
- **Plate**: custom 96-well plate with a **raised grab paddle**
  (`cad/pxrd/tabbed_plate_v2.STL`): paddle ~110.6 mm from the well center,
  20 mm thick, pinch faces centered 61.5 mm above the plate bottom.
- **Canonical layout** (baked into `run_pxrd.sh`):
  `tall_pedestal:=true pedestal_off_z:=0.55 pedestal_off_x:=0
   pedestal_off_y:=-0.2 table_x:=-0.5 table_y:=-0.9 table_h:=0.5`

## Why side-pinch (the key design fact)

A top-down grasp is **geometrically impossible** into this bay: gripper-down
means a ~350 mm vertical wrist stack above the TCP, and the forearm/wrist1
must cross the front-wall plane exactly where the wall is (probe-confirmed
contacts on every IK branch). The validated strategy mimics a human hand:
**gripper horizontal through the front slot**, jaws vertical, pinching the
plate's raised paddle — the arm never rises above the slot.

## Motion choreography (grasp-mode `side`, the default)

**Load (table → pxrd_sample):**
approach behind paddle (horizontal, pinch height) → advance 60 mm → pinch →
rigid TF attach → vertical lift to insert height → staged swing to the bay
(4-hop ladder, branch-pinned) → 200 mm horizontal insert through the slot →
set down → partial-open release (40 mm) → horizontal retract → clearance
pull → home.

**Retrieve (pxrd_sample → table):**
staged swing to the slot (empty jaws at 30 mm) → 200 mm dive, entering
**4 mm high** → pinch there (offset pinch; no seat-down — the open-jaw
envelope collides below +2 mm) → micro-lift → retract out carrying →
j1-first swing to the table → descend (+4 mm compensated → plate lands
flush) → release → back off → home.

## Hard-won constants (in `pxrd_cell/pick_place.py`)

| Constant | Value | Why |
|---|---|---|
| `TAB_CENTER_X` | −0.1106 | paddle center offset from wells (mesh frame) |
| `TAB_PINCH_Y` | 0.0615 | paddle pinch height above deck |
| `SIDE_JAW_GAP_MM` | 40 (30 for enclosed acquire) | lower-jaw tip clearance over bay floor |
| `grab_dz` | 0.004 | enclosed acquire pinches 4 mm high; deposit compensates |
| `PAD_OFFSET_FROM_TCP` | 0.069 | pads ride this far ahead of the TCP |
| `STAGE_HINTS` | pxrd + table configs | pins the IK branch — mirror wrist families collide |
| Budget gates | 13 rad total / 6 per joint / 0.1 step | reject crazy plans pre-execution |

**Staging ladder** (`plan_side_stage`): posture(j2/j3) → j1 swing → wrist →
settle, each a short deterministic joint goal. Order flips to **j1-first**
when leaving the bay (`j1_first=True`) — re-posturing next to the instrument
reaches into it; swinging away in carry posture is safe.

## Commands

```bash
./run_pxrd.sh sim         # clean + launch (terminal 1, keeps rviz)
./run_pxrd.sh load        # table -> PXRD
./run_pxrd.sh retrieve    # PXRD -> table
./run_pxrd.sh home
./run_pxrd.sh recover     # ALWAYS after a mid-cycle abort inside the bay
```

Manual (equivalent): `ros2 run pxrd_cell pick_place --source table_top
--target pxrd_sample --planner RRTConnect --place-lift 0.005 --transit-time 20`

### Diagnostic tools
| Tool | Use |
|---|---|
| `probe_pose --x --y --z [--side=vx,vy]` | why is a pose failing? reachability + exact contact pairs |
| `goto_joints --joints=j1..j6` | reproduce a pose taught in rviz |
| `spawn_plate` | live plate placement in the planning scene (no relaunch) |
| `recover` | extract stranded arm from the bay + home |
| `purge_scene` | clear plate state between runs |

## Plate design spec (for future CAD revisions)

Validated with v2; for the next spin **raise the riser +10 mm** (pinch
center ~72 mm) — converts the offset-pinch into a plain centered pinch with
comfortable margins. Keep: flat parallel pinch faces 10–20 mm thick,
≥40 mm long along the reach axis, paddle 105–120 mm from the well center.
Driver: the AG-145 fingertip extends far below the pads.

## Hardware bring-up TODO

1. Bridge works unchanged (`fr_bridge`, cell-agnostic — proven on unchained).
2. Add a `bridge.launch.py` + `control_mode:=hardware` path mirroring
   `unchained_cell` (copy the pattern; FR10 IP + pendant at low speed).
3. First runs: `--hardware`, bridge `movej_vel_pct` ≤ 5, pendant override 25%.
4. Physical plate must match `tabbed_plate_v2.STL`; verify the paddle pinch
   with `go_home --gripper-mm` calibration before any insertion.

# PXRD cell (`pxrd_cell`) — FR10 loading a Rigaku SmartLab

**Status (2026-10-20): full round trip validated in simulation for BOTH plate
revisions** with strict collision checking and deterministic staging — v2
riser plate (2026-09, mirrored-gripper era) and v3 overhang plate
(2026-10-20, repeated round trips under the FIXED gripper model, 0 failed
phases, all Cartesian fractions 1.0). Hardware bring-up pending.

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
- **Plate**: two custom 96-well plate revisions, selected with
  `pick_place --plate {v2,v3}` (also on `spawn_plate` / `recover`; default v2):
  - **v2 riser** (`cad/pxrd/tabbed_plate_v2.STL`): paddle ~110.6 mm from the
    well center, 20 mm thick, pinch faces centered 61.5 mm above the plate
    bottom (raised on a stem).
  - **v3 overhang** (`cad/pxrd/check.STL`, 2026-10-20): LOW tab — paddle
    155.7 mm out, 15 mm thick, 60 mm wide, pinch center only 26.5 mm above
    the plate bottom (deck-local 32.1 mm). Works because the paddle
    **overhangs the support edge at both decks** (table edge = 150 mm from
    the marker; SmartLab stage ends at Z≈110–115 mm), so the AG-145's long
    lower fingertip hangs in free air below deck level instead of needing a
    tall riser. See "Plate revisions" below for the jaw-gap constraint.
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

## Plate revisions (`--plate`, PLATES dict in pick_place.py)

| | v2 riser | v3 overhang |
|---|---|---|
| source CAD | `tabbed_plate_v2.STL` | `check.STL` (2026-10-20) |
| mesh | `the_plate_collision.stl` | `the_plate_v3_collision.stl` |
| `TAB_CENTER_X` | −0.1106 | −0.1557 |
| `TAB_THICKNESS` / `TAB_WIDTH` | 0.020 / 0.077 | 0.015 / 0.060 |
| `TAB_PINCH_Y` (deck-local) | 0.0615 | 0.0321 |
| jaw gap open-deck / enclosed | 40 / 30 mm | **30 / 30 mm** |
| validated | 2026-09 (mirrored-gripper era) | 2026-10-20, fixed gripper model |

**v3 jaw-gap constraint (do not raise to 40):** gripper-vs-table is
whitelisted in the SRDF ("Pick/place clearance"), so the planner CANNOT see
a fingertip-vs-table hit. Measured analytically from the finger FK: at a
40 mm gap at v3 pinch height the fingertip's forward tip passes **≈3 mm
below the table top** over the table corner (deck Z 137–150 mm); at 30 mm it
clears by +2.5 mm. Everything else dips below table-top level only beyond
the table edge, in free air. On hardware, verify the real aperture
calibration before trusting that 2.5 mm.

**v3 mesh-gen** (`scripts/make_plate_v3_meshes.py`): unlike v2's raw export,
`check.STL` is exported wells-DOWN (raw −Y = physically up), which is already
the processed-mesh convention — NO preflip-X180, just a +0.9 mm Y shift
(plate bottom → mesh +2.9 mm, same as v2) and mm→m. Well grid is already
origin-centered in X/Z. If a future export looks "upside down", check the
well-opening direction first (face-normal pattern), not the paddle side.

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

Two validated answers to "the AG-145 fingertip extends far below the pads":

- **v2 riser**: raise the paddle on a stem (pinch 61.5 mm; +10 mm more would
  center the enclosed pinch comfortably).
- **v3 overhang** (current, 2026-10): keep the paddle LOW (pinch 26.5 mm
  above plate bottom) but push it OUT past the support edge (155.7 mm from
  the well center) so the fingertip dangles in free air. Constraint chain:
  paddle must start past the SmartLab stage edge (~Z 115 mm) and mostly past
  the table edge (150 mm from the marker with the 0.30 m table); jaw gap
  capped at 30 mm (see Plate revisions). Flat parallel pinch faces ≥35 mm
  long; 15 mm thickness is comfortable at a 30 mm gap (7.5 mm per side).

## Hardware bring-up TODO

1. Bridge works unchanged (`fr_bridge`, cell-agnostic — proven on unchained).
2. Add a `bridge.launch.py` + `control_mode:=hardware` path mirroring
   `unchained_cell` (copy the pattern; FR10 IP + pendant at low speed).
3. First runs: `--hardware`, bridge `movej_vel_pct` ≤ 5, pendant override 25%.
4. Physical plate must match the chosen revision (`tabbed_plate_v2.STL` or
   `check.STL` v3 — pass the matching `--plate`); verify the paddle pinch
   with `go_home --gripper-mm` calibration before any insertion. For v3 the
   fingertip-vs-table margin is only +2.5 mm at a commanded 30 mm gap —
   re-derive it with the REAL aperture calibration (hardware open = 148.7 mm
   on the unchained AG-145, not the nominal 145) before the first table pick.

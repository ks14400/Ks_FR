# Unchained refit — session handoff (2026-09-24 → 2026-09-28)

Complete state capture so any new session/account can pick up without loss.
Read together with [unchained-cell.md](unchained-cell.md) and the top-level
CLAUDE.md rules.

## Validated state (all under FULL strict collision checking, zero retries)

| Test | Result |
|---|---|
| 8ml round trip stand_pos2 ↔ deck_vortex_pos1 (7 neighbor racks live) | **PASS both legs, repeated after fresh relaunch** |
| 20ml round trip stand_pos2 ↔ deck_vortex_pos1 | **PASS both legs** |
| 20ml LOAD from ALL 8 holder pockets → vortex pos1 | **PASS 8/8** (at final table position, see below) |
| 8ml ROUND TRIP all 8 pockets ↔ vortex pos1 (2026-10-20) | **PASS 16/16** — load + retrieve every pocket, neighbors live |
| 20ml ROUND TRIP all 8 pockets ↔ vortex pos1 (2026-10-20) | **PASS 16/16** — same loop |
| 96-well plate (type `96`, flat box 46.2mm, no vials, 2026-10-27) | **PASS 8/8** loads from all pockets |
| MOUNT −135° re-validation (2026-10-28) | 20ml 16/16 (twice: scratch + live-watched), 8ml 16/16, 96 spots 4/4 |
| PEDESTAL as-built correction (+5mm away, −2° CW; comp 137, home j1 137°) | 20ml 16/16 + spots 8/8 (pos3 load was a seed fluke: 3/3 on retry) |
| sd rack | **EXCLUDED**: knuckle-vs-vial at BOTH grips (vials 54mm above pads, sim-verified) — needs ~20-25mm finger extensions |
| Retrieve legs from vortex for pos7/pos8 sources | covered by the 16/16 matrices |
| deck_9_10 with final geometry | not re-run since refit fixes |

**Stock AG-145 fingers work. NO finger extensions needed** (earlier
55–60mm-extension conclusion was an artifact of two since-fixed bugs:
mirrored gripper model + wrong grip width).

## The five big fixes this session (all in the committed code)

1. **URDF gripper model was MIRRORED** — rendered joint 0.0 as pinched-shut
   while hardware (bridge, empirically verified) treats 0.0 = SDK 100% = OPEN.
   Fixed in `dh_ag145_macro.xacro` (BOTH cells): every gripper joint's axis
   flipped + constant `0.65 × multiplier` rotation baked into its origin, so
   value `v` now renders what the old model rendered at `(-0.65 − v)`.
   Joint VALUES and limits unchanged → bridge / pick_place / go_home /
   hardware conventions untouched. Symptom that found it: user saw jaws
   "open" when telemetry said closed (and vice versa) — trust that pattern.
2. **Jaw aperture is NONLINEAR in the joint value** (4-bar linkage).
   `_APERTURE_POLY` cubic in `unchained_cell/pick_place.py` (FK-calibrated,
   <0.05mm residual): full open = **148.7mm** true pad gap.
   `grip_mm_to_joint()` bisects the cubic → `--grip-mm` = TRUE pad gap.
   Bridge keeps its linear rad↔pct map (real gripper force-stops on the
   plate, so the conventions meet safely at the workpiece).
3. **AUTO grip width per deck** (in pick_place main flow): projects the pad
   axis (gripper X of DOWN_Q) onto the source marker's axes →
   long axis ⇒ **128.5mm** (stand pockets), short axis ⇒ **86.1mm**
   (vortex nest). Explicit `--grip-mm` / non-default `--grip-value` overrides.
   LESSON: over-closing past the plate is INVISIBLE for the pads
   (pad↔plate whitelisted for grasping) and shows up only as phantom
   inner-knuckle↔vial contacts — wrong width masquerades as a
   knuckle-geometry problem.
4. **Per-deck rest float** `rest_float_for(deck)`: stand pockets + table seat
   FLUSH (0.5mm, visually touching); instrument nests keep the measured 5mm
   stand-off (nest wall-base blocks the held plate below bottom+4mm).
   User-flagged: racks used to hover 5mm on the holder's corner fins.
5. **Per-rack grip heights** in `PLATE_TYPES[...]["grip_center"]`
   (pad center above rack bottom, must stay on the FLAT body side face):
   8ml **30mm** (body 0–44.5, vials to 66) · 20ml **15mm** (body 0–25.1,
   vials to 65) · sd **19mm** (body 0–29, vials to 77).

## Cell geometry (validated layout — treat as validated params, rule 7)

- **Table** (AMR stand-in box): world **(−0.4, −1.7)**, size
  **400 × 550 × 750mm** (launch args `table_x/y/z, table_w/d/h`).
  History: original (−0.3,−1.7) 600×600 failed pos7/pos8 (carry-home
  planning); the −100mm X shift fixed the whole 8-pocket matrix.
- **Wellplate holder** `meshes/wellplate_holder.stl` (source CAD
  `cad/unchained/wellplate_holderasm.STL`): 350 × 500 × 12.5mm, **8 pockets**
  (2 cols × 4 rows), pocket 128.0 × 86.0mm, corner fins 5mm tall, plate
  SEATS ON THE BASE SURFACE at y=7.5mm between the fins.
  Markers `stand_pos1..8` at pocket floors, yaw π/2:
  pos1–4 col X=−0.0875 (rows Z +0.1875, +0.0625, −0.0625, −0.1875),
  pos5–8 col X=+0.0875 (same row order).
- **Racks** (accurate CAD 2026-09-24, runtime meshes rotated 90°: short side
  X, long side Z, mm units): footprint 127.9 × 85.5mm all types.
  `PLATE_FOOT_X=0.1279, PLATE_FOOT_Z=0.0855`.
- **mimic_repeater** node (auto-started by unchained_full.launch.py):
  publishes the 7 mimic gripper joints at 10Hz so rviz renders the true
  gripper (controllers only publish finger1; rsp does NOT resolve mimics —
  without this the display LIES while collision checking stays correct).

## How to run (the validated commands)

```bash
# per terminal: source ros2_ws/env_unchained.sh   (domain 42)
NEIGH=stand_pos1,stand_pos3,...   # the other 7 pockets
ros2 run unchained_cell pick_place --source stand_pos2 \
    --target deck_vortex_pos1 --plate-type 20ml --also-spawn-at $NEIGH
# grip width, grasp height, place height: ALL AUTO. No --grip-mm needed.
```

Restart hygiene: `ros2_ws/clean_sim.sh` — NOTE it does not always kill the
`mimic_repeater`; `pkill -9 -f "lib/unchained_cell/mimic_repeater"` too.

After a mid-run failure the arm can strand in a config from which the home
plan exceeds the 6-rad budget gate; recover with
`ros2 run unchained_cell go_home` (and if that's budget-blocked, purge the
plate first / use the escape scripts pattern from docs).

## Real table + physical setup numbers (2026-10-02..06)

Box stand-in replaced by the real table assembly
(`cad/unchained/table_asm.STL` → `meshes/table_visual/collision.stl`):
550×400 frame on casters, long side along world Y, same world position
(−0.4, −1.7). **Tabletop plane = 756.0mm** above floor (as-built 2026-10-25:
table_z=−20.2mm — casters sink below the floor plane, table part origin ON
the floor; was 776.2 wheels-on-floor, 750 box before that);
table_top frame moved, holder + markers follow. Perimeter bolt heads
(+3.6mm, ~22mm in from edges) are in collision but clear the centered
holder footprint. **20ml re-validated from ALL 8 pockets → vortex pos1
against this table: 8/8 PASS.**

For the physical build (SolidWorks assembly, Y-up, mm, axes parallel —
derived from the sim joints + mesh conversion offsets, cross-checked in
rviz with origin markers):

- Unchained scan (`Top Level Assy AIMATX Q126_strip`) origin → pedestal
  part (`vention_pedestal_585mm`) origin: **(ΔX +465.5, ΔY +28.3,
  ΔZ +157.3)**, straight-line 492.2. Pedestal part origin = its TOP plane.
- Robot flange center on pedestal top: (−113.0, 0, +392.0) from pedestal
  origin, arm yawed **−135°** about vertical (re-measured 2026-10-28: the
  as-built arm sits 90° clockwise-from-above vs the original −45° model;
  robot_mount_yaw_deg, GRASP_YAW_COMP_DEG=135 and HOME j1=+135° all updated
  together — they must always move as a trio). Full matrix re-validated in
  sim at −135°: 20ml all-8-pocket round trips + 8ml/96 spot checks, 24/24.
- Heights above floor: scan origin **916.7**, pedestal top / robot base
  plane **945.0**, tabletop **756.0**, scan's lowest point **588.0**
  (bottom pad at scan coords (410.7, −328.6, −80.3)).
- **User confirmed the real machine's lowest point = 588mm** →
  `device_height = 0.629675` is correct as-built; height chain validated.
- Sim pedestal column runs 945 floor-to-top (585 source + 360 extension):
  the bare 585mm Vention column needs a 360mm base/riser, or adjust
  `device_height` + re-run the matrix if the as-built differs.

- Table part origin → pedestal part origin mate (scan coords, mm):
  **(ΔX +1010.0, ΔY +945.0, ΔZ −276.7)**, straight-line 1410.6. Table part
  origin sits exactly ON the floor plane; pedestal origin = its top plane.
  NOTE: matrices were validated at tabletop 776.2/750 — 756.0 sits between
  the two proven heights; re-run one 20ml round-trip loop to confirm.

## Teach/calibration system (2026-10-30 session)

UI (unchained_ui, port 8420) now carries the full manual-calibration loop:
Go-to-hover (real pick pipeline, --hover-only) -> drag and/or XYZ+RPY jog
(collision-checked steps) -> FREE jaws (direct-to-bridge, force-limited,
no model check — teach only) -> Capture XY / Z / ANGLES (guards refuse
wrong-deck >50mm and wrong-moment >15deg) -> ledger unchained_ui/
deck_calib.yaml: dx/dy + drpy per DECK, dz per PLATE TYPE. Hardware runs
apply taught values to targets, grasp/place orientation, AND the virtual
plate rest pose; the in-nest window covers the held plate.
Drag-teach service lives in unchained_bridge (/sdk_executor/drag_teach).
Scene singleton guard is in the LAUNCH FILE (4 duplicate-scene incidents).
place_held = recovery/second-half runs; never moves before the virtual
attach. Taught so far: deck_vortex_pos1 dx=-13.0 dy=-16.6 (dz: 20ml +15.2,
8ml +0.1; drpy captured feature exists, not yet taught).
KEY OPEN FINDING: the vortex dx/dy (~21mm) likely means the INSTRUMENT
model position is off — teach a second instrument deck; if it agrees,
bake the offset into the model and shrink the ledger to ~0.
Last hardware state: 8ml grasp at the vortex SUCCEEDED (close+attach);
lift was the last fixed bug; a clean full pick has not yet completed —
first action next session: relaunch via app, full pick vortex->stand_pos4.

## Open items (in priority order)

1. **sd rack** round trip (grip_center 19mm set but never run with new grip).
2. Retrieve spot-checks: vortex → stand_pos7 / stand_pos8 (reverse of the
   configs that were hardest).
3. deck_9_10 re-validation with final geometry (expected to still be
   height-limited by the new obstruction element — characterize honestly).
4. Playbook (`docs/robot-cell-playbook.md`) update with this session's
   lessons (mirrored-model discovery pattern, aperture calibration,
   auto-width, per-deck float, wrong-width-masquerades-as-knuckle-clash).
5. PXRD cell: gripper macro fix is copied over, but pxrd grip phases were
   validated under the OLD mirrored model — re-validate before trusting.
6. Hardware bring-up for the refit cell: sim→hardware same script +
   `--hardware` + bridge; bridge gripper mapping intentionally unchanged.

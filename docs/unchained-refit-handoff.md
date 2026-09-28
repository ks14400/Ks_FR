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
| sd rack | **NOT yet run** with the new grip/holder — next task |
| Retrieve legs from vortex for pos7/pos8 sources | not spot-checked yet |
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

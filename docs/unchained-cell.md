# Unchained cell (`unchained_cell`) — FR16 serving an Unchained Junior

**Status: HARDWARE-PROVEN (2026-06).** Full pick-and-place on the real FR16
including real AG-145 gripper actuation, force-grip, the −45° pedestal
mount, and a fixed obstacle. This package was previously named
`fr_test_cell` (renamed 2026-09).

## LATEST WORKING TERMINAL COMMANDS (hardware-proven 2026-06, verbatim)

```bash
# T1 — bridge:
ros2 launch unchained_cell bridge.launch.py robot_ip:=192.168.58.2 \
  movej_vel_pct:=20 movej_acc_pct:=30 gripper_force_pct:=40

# T2 — scene + MoveIt (hardware):
ros2 launch unchained_cell unchained_full.launch.py control_mode:=hardware \
  table_x:=-0.3 table_y:=-1.7 table_h:=1

# T3 — the job:
ros2 run unchained_cell pick_place --source deck_9_10_pos1 --target table_top \
  --yaw-tol 0.15 --vel 0.10 --hover 0.25 --pick-lift 0.015 --hardware
```

These exact values are baked into `./run_unchained.sh`. Position meanings:
- `table_x:=-0.3 table_y:=-1.7 table_h:=1` — REAL measured staging-table
  position in WORLD frame (meters), top at 1.0 m. (Old sim-only default was
  `table_x:=-0.8 table_y:=-0.9 table_h:=1`.)
- Pedestal origin offset from the Unchained origin (URDF `pedestal_joint`):
  xyz = (0.400, 0.31533, 1.13827), rpy 0 — from SolidWorks origin-to-origin.
- Robot mount: `robot_mount_yaw_deg:=-45` (default; in the PITCH slot).

## The scene

- **Instrument**: Unchained Junior with 7 open deck positions:
  `deck_9_10_pos1..3`, `deck_vortex_pos1..3`, `deck_vacuum_filtration`,
  plus the external `table_top` staging table.
- **Robot**: FR16 + DH AG-145, bolted to the 585 mm Vention pedestal at
  **−45°** (connector right, parallel to the instrument).
- **Plate**: standard SBS 96-well (85.5 × 127.8 × 25 mm box model).

## Hardware-calibrated values (all baked as defaults — do not re-derive)

| Thing | Value | Note |
|---|---|---|
| `robot_mount_yaw_deg` | −45 | goes in the **pitch slot** of the mount rpy (pedestal frame is Y-up; yaw would tip the arm) |
| `HOME_JOINTS.j1` | +0.785 | compensates the mount so home faces the workspace (rule: j1 = −mount_yaw) |
| `GRASP_YAW_COMP_DEG` | +45 | pre-rotates DOWN_Q so jaws meet the plate square |
| `grasp_offset` | 0.088 | TCP above deck at grasp; pads on plate's top 12 mm |
| `--pick-lift` | 0.015 | deck pick stops 15 mm higher so fingertips don't bottom out |
| `--hover` | 0.25 | clears the fixed obstacle near deck_9_10_pos1 |
| Real table position | `table_x:=-0.3 table_y:=-1.7 table_h:=1` | measured |
| Gripper | force-grip: `--grip-value -0.64` + bridge `gripper_force_pct` (live-tunable) | jaws drive nearly shut; force limit stops them on the plate |

## Running

### Simulation
```bash
./run_unchained.sh sim
./run_unchained.sh pick deck_9_10_pos1 table_top
./run_unchained.sh home
```

### Hardware (three terminals)
```bash
# T0 (once): ros2_ws/clean_sim.sh
# T1 — bridge (generic SDK transport; arm+gripper trajectories, /joint_states, /stop_motion):
./run_unchained.sh bridge          # robot_ip:=192.168.58.2 movej_vel_pct:=20 gripper_force_pct:=40
# T2 — scene + MoveIt for hardware:
./run_unchained.sh hw-scene
# T3 — the job:
./run_unchained.sh pick deck_9_10_pos1 table_top --hardware
```

**Speed truth**: real arm speed = bridge `movej_vel_pct` (3 = crawl, 20–30 =
normal). `--vel` only paces the planned trajectory (keep ≤ 0.10 under the
jerk budget). Pendant velocity override multiplies on top.

**Gripper force live-tune** (no relaunch):
`ros2 param set /sdk_executor gripper_force_pct 50`

**Emergency**: `./run_unchained.sh stop` (cancels MoveIt goal + SDK halt).
Prefer it over Ctrl-C. Then `./run_unchained.sh home`.

### Gripper calibration
`ros2 run unchained_cell go_home --hardware --gripper-mm 70` then measure
the physical gap; adjust `GRIPPER_STROKE_MM` (nominal 145, unverified) if
it differs. Feedback quirk: `GetGripperCurPosition` reads 0 transiently
after a command; the bridge uses debounced position + stall detection
(stall = object gripped).

## Known behaviors

- `-6` action timeouts in logs while the motion visibly completes: bridge
  completion-report latency, harmless.
- A pick that plans but scores > budget gets auto-retried with a new seed;
  persistent rejections mean the layout changed — re-check deck TFs.
- Never modify the YAML/param sets that ran on hardware — copy them.

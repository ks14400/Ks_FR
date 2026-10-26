#!/usr/bin/env bash
# ============================================================
# PXRD cell launcher — one command to a working sim.
#
#   ./run_pxrd.sh sim             # clean + launch sim with canonical layout
#   ./run_pxrd.sh load            # table -> PXRD  (orchestrator pick_place)
#   ./run_pxrd.sh retrieve        # PXRD -> table  (orchestrator pick_place)
#   ./run_pxrd.sh replay          # table -> PXRD via the TAUGHT joint path (fast)
#   ./run_pxrd.sh replay-reverse  # PXRD -> table via the taught path (retrieve)
#   ./run_pxrd.sh replay-home     # park at the TAUGHT 585mm start-HOME (do FIRST)
#   ./run_pxrd.sh home            # arm to home (OLD 945mm HOME_JOINTS — not 585mm)
#   ./run_pxrd.sh recover         # stranded-arm rescue (extract from bay + home)
#   ./run_pxrd.sh preflight       # FR10 -> MANUAL mode + enabled (ONCE, no bridge)
#   ./run_pxrd.sh hw-scene        # scene + MoveIt for the REAL robot
#   ./run_pxrd.sh bridge          # hardware bridge to the FR10 (own terminal)
#
# Run `sim` in one terminal (keeps rviz open); load/retrieve/home/recover in
# a second terminal. Domain isolation (43) is handled automatically.
# Extra args after the verb are passed through to the underlying command.
#
# Hardware flow (pendant override 25%, e-stop in hand):
#   preflight: ./run_pxrd.sh preflight        # ONCE, bridge stopped — sets
#              the FR10 to MANUAL mode (SDK MoveJ only runs in manual) + enable
#   T1: ./run_pxrd.sh bridge                  # verify robot_ip first!
#   T2: ./run_pxrd.sh hw-scene
#   T3: ./run_pxrd.sh load --plate v3 --hardware
#
# OPEN-SPACE MOTION REHEARSAL of the taught paths on the REAL FR10 (no gripper,
# nothing within reach). The bridge executes the same budget-chunked trajectories;
# real speed = BRIDGE_ARGS movej_vel_pct. The --hardware flag prints a checklist,
# asks for confirmation, and moves to the side-HOME first before the path:
#   preflight (ONCE), then  T1: bridge   T2: hw-scene
#   T3: ./run_pxrd.sh replay-home --hardware      # home to the start pose FIRST
#       ./run_pxrd.sh replay --hardware           # table -> PXRD  (traced in air)
#       ./run_pxrd.sh replay-reverse --hardware   # PXRD -> table
# Before the first REAL insertion (with a gripper + the instrument present):
# verify the paddle pinch (ros2 run pxrd_cell go_home --gripper-mm 30, measure the
# real gap) and RE-TEACH the carry on real geometry (PXRD_RECORD_CARRY, see docs).
# ============================================================
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=43

# ===== 585mm PEDESTAL LAYOUT (2026-10-23) — matches the real hardware =====
# The FR10 is now on the 585mm Vention pedestal (tall_pedestal:=false), bolted
# 45 deg rotated on the plate (robot_mount_yaw_deg:=45). Pedestal at off_z 0.8,
# off_x 0.2 (off_y auto = -0.6246). These were calibrated in sim against the
# REAL arm's jogged-into-bay joint pose (cad/pxrd/fr10_585_jogged_joints.json,
# mirrored into config/initial_positions.yaml) until the sim gripper matched
# the real one in the bay. Table at (-0.5,-0.9,0.5): the position the full
# table->home->bay path was taught against (scripts/full_path_probe.py ->
# cad/pxrd/taught_path_585_full.json; replay via scripts/pxrd_replay_path.py).
# (2.0,2.0 "out of the way" was only for the earlier home-only testing.)
SIM_ARGS="tall_pedestal:=false pedestal_off_z:=0.8 pedestal_off_x:=0.2 pedestal_off_y:=auto robot_mount_yaw_deg:=45 table_x:=-0.5 table_y:=-0.9 table_h:=0.5"
# PREVIOUS 945mm flat-carry layout (2026-10-21, v3-validated; superseded by the
# 585mm hardware swap — kept for reference / rollback):
#   SIM_ARGS="tall_pedestal:=true pedestal_off_z:=0.70 pedestal_off_x:=0.15 pedestal_off_y:=-0.2 table_x:=-0.5 table_y:=-0.9 table_h:=0.5"
# Canonical validated motion flags. place-lift 0.002 (2026-10-22): the plate
# seats 3 mm lower than the v2-era 0.005 — firmer set-down on the sample
# stage (same change the unchained cell needed on hardware). The deep-reach
# singularity that motivated 0.005 is gone at the stepped-back pedestal.
RUN_FLAGS="--planner RRTConnect --place-lift 0.002 --transit-time 20"

# Hardware: same world as sim (geometry comes from the launch defaults), the
# bridge executes trajectories on the real FR10. VERIFY robot_ip on the
# pendant before first use. movej_vel_pct 3 for bring-up (repo rule: <=5
# near the instrument).
HW_SCENE_ARGS="control_mode:=hardware $SIM_ARGS"
# gripper_enable:=false — no gripper on the FR10 yet (avoids the phantom
# AG-145 activation error). The bridge code is the UNMODIFIED shared
# fr_bridge; manual-mode is handled by the `preflight` verb below, never in
# the bridge (which the unchained cell also runs).
BRIDGE_ARGS="robot_ip:=192.168.58.2 robot_model:=fr10 movej_vel_pct:=20 movej_acc_pct:=30 gripper_enable:=false"

cmd="${1:-sim}"; shift || true
case "$cmd" in
  sim)
    ./ros2_ws/clean_sim.sh || true
    exec ros2 launch pxrd_cell pxrd_full.launch.py $SIM_ARGS "$@"
    ;;
  preflight)
    # Put the FR10 in MANUAL mode + enabled (SDK MoveJ needs manual mode).
    # Run ONCE with the bridge stopped, before `bridge`. No arm motion.
    exec python3 "$(dirname "$0")/scripts/pxrd_hw_preflight.py" "$@"
    ;;
  hw-scene)
    exec ros2 launch pxrd_cell pxrd_full.launch.py $HW_SCENE_ARGS "$@"
    ;;
  bridge)
    exec ros2 launch pxrd_cell bridge.launch.py $BRIDGE_ARGS "$@"
    ;;
  load)
    exec ros2 run pxrd_cell pick_place --source table_top --target pxrd_sample $RUN_FLAGS "$@"
    ;;
  retrieve)
    exec ros2 run pxrd_cell pick_place --source pxrd_sample --target table_top $RUN_FLAGS "$@"
    ;;
  replay)
    # TAUGHT-PATH load (table -> PXRD) via the recorded joint path — fast, no
    # IK/OMPL search. Works in sim as-is; on the REAL FR10 add --hardware after
    # `preflight`+`bridge`+`hw-scene` are up (open space, no gripper).
    exec python3 "$(dirname "$0")/scripts/pxrd_replay_path.py" "$@"
    ;;
  replay-reverse)
    # TAUGHT-PATH retrieve (PXRD -> table). Same flags; add --hardware for real.
    exec python3 "$(dirname "$0")/scripts/pxrd_replay_path.py" --reverse "$@"
    ;;
  replay-home)
    # Park the arm at the TAUGHT side-HOME (the 585mm starting position the
    # replay paths expect) and stop. Run this BEFORE replay/replay-reverse.
    # NB: this is NOT `home` (go_home uses the stale 945mm HOME_JOINTS). Add
    # --hardware to home the real FR10 (confirmation + bridge speed).
    exec python3 "$(dirname "$0")/scripts/pxrd_replay_path.py" --home-only "$@"
    ;;
  home)
    exec ros2 run pxrd_cell go_home "$@"
    ;;
  recover)
    exec ros2 run pxrd_cell recover "$@"
    ;;
  *)
    echo "usage: $0 {sim|load|retrieve|replay|replay-reverse|replay-home|home|recover|preflight|hw-scene|bridge} [extra args]" >&2
    exit 2
    ;;
esac

#!/usr/bin/env bash
# ============================================================
# Unchained cell launcher (FR16 + AG-145, HARDWARE-PROVEN).
#
#   ./run_unchained.sh sim                    # sim (mock controllers) + rviz
#   ./run_unchained.sh hw-scene               # scene+MoveIt for REAL robot
#   ./run_unchained.sh bridge                 # hardware bridge (own terminal)
#   ./run_unchained.sh pick <src> <tgt>       # pick-and-place
#   ./run_unchained.sh home                   # arm to home
#   ./run_unchained.sh stop                   # halt motion (use over Ctrl-C)
#
# Decks: deck_9_10_pos1..3, deck_vortex_pos1..3, deck_vacuum_filtration,
#        stand_pos1..8 (wellplate holder pockets), table_top
# Domain isolation (42) is handled automatically. Extra args pass through.
# Hardware flow (three terminals):
#   T1: ./run_unchained.sh bridge
#   T2: ./run_unchained.sh hw-scene
#   T3: ./run_unchained.sh pick deck_9_10_pos1 table_top --hardware
# ============================================================
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

# REFIT layout (2026-10, sim-validated: 8ml/20ml round trips + all-8-pocket
# matrix): the world is defined ENTIRELY by the launch defaults — real table
# mesh at (-0.4,-1.7) top 776.2mm, 8-pocket holder, stand_pos1..8. Hardware
# must use the SAME world, so hw-scene passes NO geometry overrides.
# Grip width / grasp height / seating are AUTO in pick_place — no flags.
HW_SCENE_ARGS="control_mode:=hardware"
PICK_FLAGS=""
# Bridge: proven comms values; movej_vel_pct 5 for refit bring-up (raise to
# the proven 20 only after the refit workflow is hardware-verified).
BRIDGE_ARGS="robot_ip:=192.168.58.2 movej_vel_pct:=5 movej_acc_pct:=10 gripper_force_pct:=40"
# LEGACY (pre-refit, hardware-proven 2026-06 with the old box table — kept
# per rule 7; do NOT use with the refit world):
#   HW_SCENE_ARGS="control_mode:=hardware table_x:=-0.3 table_y:=-1.7 table_h:=1"
#   BRIDGE_ARGS="robot_ip:=192.168.58.2 movej_vel_pct:=20 movej_acc_pct:=30 gripper_force_pct:=40"
#   PICK_FLAGS="--yaw-tol 0.15 --vel 0.10 --hover 0.25 --pick-lift 0.015"

cmd="${1:-sim}"; shift || true
case "$cmd" in
  sim)
    ./ros2_ws/clean_sim.sh || true
    exec ros2 launch unchained_cell unchained_full.launch.py "$@"
    ;;
  hw-scene)
    exec ros2 launch unchained_cell unchained_full.launch.py $HW_SCENE_ARGS "$@"
    ;;
  bridge)
    exec ros2 launch unchained_cell bridge.launch.py $BRIDGE_ARGS "$@"
    ;;
  pick)
    src="$1"; tgt="$2"; shift 2 || { echo "usage: $0 pick <source> <target> [flags]" >&2; exit 2; }
    exec ros2 run unchained_cell pick_place --source "$src" --target "$tgt" $PICK_FLAGS "$@"
    ;;
  home)
    exec ros2 run unchained_cell go_home "$@"
    ;;
  stop)
    exec ros2 run unchained_cell stop "$@"
    ;;
  *)
    echo "usage: $0 {sim|hw-scene|bridge|pick|home|stop} [extra args]" >&2
    exit 2
    ;;
esac

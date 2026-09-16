#!/usr/bin/env bash
# ============================================================
# PXRD cell launcher — one command to a working sim.
#
#   ./run_pxrd.sh sim         # clean + launch sim with canonical layout
#   ./run_pxrd.sh load        # table -> PXRD  (plate into the instrument)
#   ./run_pxrd.sh retrieve    # PXRD -> table  (plate out of the instrument)
#   ./run_pxrd.sh home        # arm to home
#   ./run_pxrd.sh recover     # stranded-arm rescue (extract from bay + home)
#
# Run `sim` in one terminal (keeps rviz open); load/retrieve/home/recover in
# a second terminal. Domain isolation (43) is handled automatically.
# Extra args after the verb are passed through to the underlying command.
# ============================================================
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=43

# Canonical validated layout (2026-09): tall pedestal, table to the side.
SIM_ARGS="tall_pedestal:=true pedestal_off_z:=0.55 pedestal_off_x:=0 pedestal_off_y:=-0.2 table_x:=-0.5 table_y:=-0.9 table_h:=0.5"
# Canonical validated motion flags.
RUN_FLAGS="--planner RRTConnect --place-lift 0.005 --transit-time 20"

cmd="${1:-sim}"; shift || true
case "$cmd" in
  sim)
    ./ros2_ws/clean_sim.sh || true
    exec ros2 launch pxrd_cell pxrd_full.launch.py $SIM_ARGS "$@"
    ;;
  load)
    exec ros2 run pxrd_cell pick_place --source table_top --target pxrd_sample $RUN_FLAGS "$@"
    ;;
  retrieve)
    exec ros2 run pxrd_cell pick_place --source pxrd_sample --target table_top $RUN_FLAGS "$@"
    ;;
  home)
    exec ros2 run pxrd_cell go_home "$@"
    ;;
  recover)
    exec ros2 run pxrd_cell recover "$@"
    ;;
  *)
    echo "usage: $0 {sim|load|retrieve|home|recover} [extra args]" >&2
    exit 2
    ;;
esac

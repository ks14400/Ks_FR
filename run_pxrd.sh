#!/usr/bin/env bash
# ============================================================
# PXRD cell launcher — one command to a working sim.
#
#   ./run_pxrd.sh sim         # clean + launch sim with canonical layout
#   ./run_pxrd.sh load        # table -> PXRD  (plate into the instrument)
#   ./run_pxrd.sh retrieve    # PXRD -> table  (plate out of the instrument)
#   ./run_pxrd.sh home        # arm to home
#   ./run_pxrd.sh recover     # stranded-arm rescue (extract from bay + home)
#   ./run_pxrd.sh preflight   # FR10 -> MANUAL mode + enabled (run ONCE, no bridge)
#   ./run_pxrd.sh hw-scene    # scene + MoveIt for the REAL robot
#   ./run_pxrd.sh bridge      # hardware bridge to the FR10 (own terminal)
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
# Before the first insertion: verify the paddle pinch with
#   ros2 run pxrd_cell go_home --gripper-mm 30   (and measure the real gap)
# and RE-TEACH the carry on real geometry (PXRD_RECORD_CARRY, see docs).
# ============================================================
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=43

# Canonical FLAT-CARRY layout (2026-10-21, validated with the v3 plate):
# pedestal stepped BACK to 0.70 and shifted +0.15 east vs the 2026-09 layout
# (was off_z 0.55, off_x 0). Why: the v3 plate's long tab closes every carry
# clearance at the old spot — the swing arc grazes the SmartLab cabinet's
# west corner by 1 mm (structural: the arm cannot fold below the stage
# radius) and the bay-mouth wrist yaw shaves the face. At 0.70/+0.15 the
# corner clears by ~10 cm and the bay-mouth yaw circle clears by 4-14 cm.
# The carry itself replays a TAUGHT flat path (config/taught_carry_*.json),
# collision+tilt-checked live each run; the tilt gate (--max-carry-tilt 10)
# guarantees the plate top face never tips, measured 9.3 deg max.
SIM_ARGS="tall_pedestal:=true pedestal_off_z:=0.70 pedestal_off_x:=0.15 pedestal_off_y:=-0.2 table_x:=-0.5 table_y:=-0.9 table_h:=0.5"
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
  home)
    exec ros2 run pxrd_cell go_home "$@"
    ;;
  recover)
    exec ros2 run pxrd_cell recover "$@"
    ;;
  *)
    echo "usage: $0 {sim|load|retrieve|home|recover|preflight|hw-scene|bridge} [extra args]" >&2
    exit 2
    ;;
esac

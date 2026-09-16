"""
Plan + execute a single arm move to an explicit set of joint angles.

Diagnostic / calibration tool. Unlike pick_place there is NO pick, NO plate,
NO via routing — just "from wherever the arm is now, go to these 6 joint
angles." Use it to reproduce a pose you found by jogging the arm in rviz:

    ros2 run pxrd_cell go_home
    ros2 run pxrd_cell goto_joints --joints=-1.5882,-1.9548,2.7053,2.3213,-1.5708,0.1047

Watch the arm SHAPE at the end — it should match exactly what you posed in
rviz. If it does, the angles are correct and any "weird" look in pick_place
is the PATH (planner), not the target. If the shape differs, the angle
mapping/order is wrong.

Values are j1..j6 in radians. Because they often start with '-', use the
'=' form: --joints=-1.6,... (a leading '-' otherwise confuses argparse).

The Layer-1 budget gate still applies so a "crazy" plan can't execute.
"""
import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from moveit_msgs.action import MoveGroup

from pxrd_cell.pick_place import (
    HOME_JOINTS,
    DOWN_Q,
    plan_arm_to_joints,
    _print_config,
)
from pxrd_cell.go_home import cancel_all_motion


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--joints", type=str, required=True,
                   help="Comma-separated j1..j6 in radians. Use the '=' form "
                        "when the first value is negative: --joints=-1.6,...")
    p.add_argument("--vel", type=float, default=0.10,
                   help="Velocity scaling 0..1 (default 0.10).")
    p.add_argument("--hardware", action="store_true",
                   help="Hardware-safe: default vel 0.05, clamp to 0.50.")
    p.add_argument("--planning-time", type=float, default=10.0,
                   help="OMPL planning time (seconds, default 10).")
    p.add_argument("--planner", type=str, default="RRTConnect",
                   help="OMPL planner id (default RRTConnect — clean paths).")
    p.add_argument("--no-cancel", action="store_true",
                   help="Skip the cancel-active-motion step.")
    p.add_argument("--keep-plate-flat", action="store_true",
                   help="Hold the gripper level (plate flat) for the whole "
                        "move. Use when carrying a plate during teaching.")
    p.add_argument("--path-tilt-tol", type=float, default=0.3,
                   help="Roll/pitch tolerance (rad) for --keep-plate-flat "
                        "(default 0.3 ≈ 17°).")
    args = p.parse_args()

    names = [f"j{i+1}" for i in range(6)]
    try:
        vals = [float(x) for x in args.joints.split(",")]
    except ValueError:
        print("ERROR: --joints must be comma-separated numbers", file=sys.stderr)
        sys.exit(2)
    if len(vals) != 6:
        print(f"ERROR: need exactly 6 joint values, got {len(vals)}",
              file=sys.stderr)
        sys.exit(2)
    target = {names[i]: vals[i] for i in range(6)}

    HARDWARE_VEL_CEIL = 0.50
    if args.hardware:
        if args.vel == 0.10:  # left at default
            args.vel = 0.05
        if args.vel > HARDWARE_VEL_CEIL:
            print(f"[hardware] clamping vel {args.vel} -> {HARDWARE_VEL_CEIL}",
                  file=sys.stderr)
            args.vel = HARDWARE_VEL_CEIL

    rclpy.init()
    node = Node("goto_joints")
    _print_config(node, args)

    cb_group = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    mg = ActionClient(node, MoveGroup, "/move_action", callback_group=cb_group)
    node.get_logger().info("Waiting for /move_action ...")
    if not mg.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("move_group not available")
        rclpy.shutdown()
        sys.exit(1)

    if not args.no_cancel:
        node.get_logger().info("Step 1/2: cancel any active motion")
        cancel_all_motion(node, mg)

    node.get_logger().info("Step 2/2: planning + executing to target joints")
    node.get_logger().info(
        "  target: " + ", ".join(f"{k}={v:+.4f}" for k, v in target.items()))
    node.get_logger().info(f"  planner={args.planner}  vel={args.vel:.3f}")

    t0 = time.time()
    ok, info = plan_arm_to_joints(
        node, mg, target,
        vel=args.vel,
        planning_time=args.planning_time,
        planning_attempts=15,
        planner_id=args.planner,
        keep_orientation_along_path=args.keep_plate_flat,
        orient_q=DOWN_Q,
        path_tilt_tol=args.path_tilt_tol,
    )
    dur = time.time() - t0

    if not ok:
        node.get_logger().error(f"  [FAIL] {dur:.2f}s — {info}")
        rclpy.shutdown()
        sys.exit(1)
    node.get_logger().info(f"  [OK] {dur:.2f}s — {info}")
    node.get_logger().info("DONE — compare the arm SHAPE to what you posed in rviz")
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()

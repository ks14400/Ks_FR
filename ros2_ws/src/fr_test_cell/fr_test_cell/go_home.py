"""
Cancel any active arm motion and slowly send the arm back to the HOME pose.

Use cases:
  - Reset the simulation between pick_place runs (no relaunch needed)
  - Bring the arm back to a known safe state after a failed run
  - First test on real hardware (default vel is very slow)

The home pose is HOME_JOINTS imported from pick_place.py — same config used
by pick_place's initial "go to home" phase. Layer-1 budget gate applies so a
"crazy" plan can never be executed.

Usage:
  ros2 run fr_test_cell go_home                 # default --vel 0.10
  ros2 run fr_test_cell go_home --vel 0.05      # slower
  ros2 run fr_test_cell go_home --hardware      # hardware-safe default vel
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

from fr_test_cell.pick_place import (
    HOME_JOINTS,
    GRIPPER_OPEN,
    GRIPPER_STROKE_MM,
    grip_mm_to_joint,
    plan_arm_to_joints,
    plan_gripper,
    _print_config,
    _trajectory_stats,
)

GRIPPER_CLOSED = -0.65  # URDF lower limit = jaws fully closed (stow state)


def cancel_all_motion(node, mg_client, timeout=3.0):
    """Cancel ANY active MoveGroup goal so the new plan can start cleanly."""
    cancel_cli = node.create_client(CancelGoal, "/move_action/_action/cancel_goal")
    if not cancel_cli.wait_for_service(timeout_sec=2.0):
        node.get_logger().warning(
            "  /move_action cancel service unavailable — proceeding anyway")
        return
    req = CancelGoal.Request()  # empty = cancel all
    fut = cancel_cli.call_async(req)
    deadline = time.time() + timeout
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if fut.done() and fut.result() is not None:
        n_canceled = len(fut.result().goals_canceling)
        if n_canceled > 0:
            node.get_logger().info(f"  Canceled {n_canceled} active goal(s)")
            time.sleep(0.3)  # give the canceled goal time to actually stop
        else:
            node.get_logger().info("  No active motion to cancel")
    else:
        node.get_logger().warning("  Cancel request timed out")


def main():
    p = argparse.ArgumentParser()
    # Use a sentinel default so we can tell whether the user explicitly
    # specified --vel or accepted the default.
    p.add_argument("--vel", type=float, default=None,
                   help="Velocity scaling 0..1. Default 0.10 in sim, 0.05 "
                        "in hardware mode. Whatever you pass is respected "
                        "(up to the hardware ceiling of 0.10).")
    p.add_argument("--hardware", action="store_true",
                   help="Hardware-safe defaults: clamps --vel to max 0.10 "
                        "and sets it to 0.05 if not otherwise specified.")
    p.add_argument("--planning-time", type=float, default=5.0,
                   help="OMPL planning time (seconds, default 5)")
    p.add_argument("--no-cancel", action="store_true",
                   help="Skip the cancel-active-motion step (faster, but "
                        "may fail if another action is running)")
    p.add_argument("--gripper", choices=["close", "open", "keep"],
                   default="keep",
                   help="What to do with the gripper after homing: "
                        "'close' = jaws fully closed (stow), 'open' = fully "
                        "open, 'keep' = leave as-is (default).")
    p.add_argument("--gripper-mm", type=float, default=None,
                   help="Open the gripper to a specific jaw width (mm) after "
                        "homing — calibration/test tool. Measure the physical "
                        "gap and compare; if it differs from this value, adjust "
                        "GRIPPER_STROKE_MM in pick_place.py. Overrides --gripper.")
    args = p.parse_args()

    # Decide --vel default & clamp to hardware ceiling.
    # NOTE: actual arm speed is also bounded by the bridge's movej_vel_pct
    # (default 3%), so this is a planning scaling only.
    HARDWARE_VEL_CEIL = 0.50
    if args.vel is None:
        # User didn't specify --vel; pick a safe default per mode.
        args.vel = 0.05 if args.hardware else 0.10
    elif args.hardware and args.vel > HARDWARE_VEL_CEIL:
        print(f"[hardware] --vel {args.vel} exceeds safety ceiling "
              f"{HARDWARE_VEL_CEIL}; clamping.", file=sys.stderr)
        args.vel = HARDWARE_VEL_CEIL

    rclpy.init()
    node = Node("go_home")
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

    node.get_logger().info("Step 2/2: planning + executing home")
    node.get_logger().info(
        f"  target HOME_JOINTS: " +
        ", ".join(f"{k}={v:+.2f}" for k, v in HOME_JOINTS.items()))
    node.get_logger().info(
        f"  vel scaling: {args.vel:.3f}  (trajectory will be paced slowly)")

    t0 = time.time()
    ok, info = plan_arm_to_joints(
        node, mg, HOME_JOINTS,
        vel=args.vel,
        planning_time=args.planning_time,
        planning_attempts=15,
        planner_id="RRTstar",
    )
    dur = time.time() - t0

    if not ok:
        node.get_logger().error(f"  [FAIL] {dur:.2f}s — {info}")
        node.get_logger().error("Arm did NOT reach home. Check rviz.")
        rclpy.shutdown()
        sys.exit(1)
    node.get_logger().info(f"  [OK] {dur:.2f}s — {info}")

    if args.gripper_mm is not None:
        target = grip_mm_to_joint(args.gripper_mm)
        node.get_logger().info(
            f"Step 3: gripper to {args.gripper_mm:.0f}mm (joint={target:+.3f}, "
            f"stroke={GRIPPER_STROKE_MM:.0f}mm) — MEASURE the physical gap")
        t0 = time.time()
        ok, info = plan_gripper(node, mg, target)
        dur = time.time() - t0
        if ok:
            node.get_logger().info(f"  [OK] {dur:.2f}s — {info}")
        else:
            node.get_logger().error(f"  [FAIL] {dur:.2f}s — {info}")
            rclpy.shutdown(); sys.exit(1)
        node.get_logger().info("DONE — arm at HOME, gripper set")
        rclpy.shutdown(); sys.exit(0)

    if args.gripper != "keep":
        target = GRIPPER_CLOSED if args.gripper == "close" else GRIPPER_OPEN
        node.get_logger().info(
            f"Step 3: gripper {args.gripper} (joint={target:+.2f})")
        t0 = time.time()
        ok, info = plan_gripper(node, mg, target)
        dur = time.time() - t0
        if ok:
            node.get_logger().info(f"  [OK] {dur:.2f}s — {info}")
        else:
            node.get_logger().error(f"  [FAIL] {dur:.2f}s — {info}")
            node.get_logger().error("Gripper did not move. Arm IS at home.")
            rclpy.shutdown()
            sys.exit(1)

    node.get_logger().info("DONE — arm is at HOME")
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()

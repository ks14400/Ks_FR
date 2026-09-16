"""
Emergency stop — cancel any active arm motion immediately. Arm holds where
it is. Used in place of Ctrl-C so we get a clean, predictable halt.

Works in both sim and hardware:
  - Sim: cancels MoveIt action goals; mock controller stops sending commands.
  - Hardware: same cancels PLUS calls sdk_executor's /stop_motion service
    which invokes the FR16 SDK's StopMotion() for guaranteed halt independent
    of MoveIt.

After stop you should run 'go_home' to recover to a known safe pose before
issuing further motion commands.

Usage:
  ros2 run fr_test_cell stop
"""
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.srv import CancelGoal
from std_srvs.srv import Trigger


def cancel_action(node, action_ns, label):
    """Send a cancel-all request to a ROS 2 action server's cancel service."""
    cli = node.create_client(CancelGoal, f"{action_ns}/_action/cancel_goal")
    if not cli.wait_for_service(timeout_sec=1.5):
        node.get_logger().info(f"  {label}: cancel service unavailable (not running?)")
        return 0
    req = CancelGoal.Request()  # empty = cancel all
    fut = cli.call_async(req)
    deadline = time.time() + 3.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done():
        node.get_logger().warning(f"  {label}: cancel timed out")
        return 0
    result = fut.result()
    if result is None:
        node.get_logger().warning(f"  {label}: cancel returned None")
        return 0
    n = len(result.goals_canceling)
    if n > 0:
        node.get_logger().info(f"  {label}: canceled {n} goal(s)")
    else:
        node.get_logger().info(f"  {label}: no active goals")
    return n


def try_stop_motion_service(node):
    """Try calling sdk_executor's /stop_motion service (only present in
    hardware mode). Silently no-ops if not available."""
    cli = node.create_client(Trigger, "/stop_motion")
    if not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info(
            "  /stop_motion: not available (sim mode, this is normal)")
        return False
    fut = cli.call_async(Trigger.Request())
    deadline = time.time() + 3.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if fut.done() and fut.result() is not None:
        res = fut.result()
        node.get_logger().info(
            f"  /stop_motion: {'OK' if res.success else 'FAILED'} — {res.message}")
        return res.success
    node.get_logger().warning("  /stop_motion: call timed out")
    return False


def main():
    rclpy.init()
    node = Node("stop")

    cb_group = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    node.get_logger().info("STOP — cancelling any active arm motion")

    # 1. Cancel MoveIt's MoveGroup action (the high-level planner)
    n_mg = cancel_action(node, "/move_action", "MoveGroup")

    # 2. Cancel ExecuteTrajectory (in case we're mid-execution)
    n_et = cancel_action(node, "/execute_trajectory", "ExecuteTrajectory")

    # 3. (Hardware mode only) call SDK StopMotion as a last-line safety
    try_stop_motion_service(node)

    total = n_mg + n_et
    node.get_logger().info(f"DONE — {total} goal(s) canceled. "
                           f"Run 'go_home' to return to a known state.")

    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()

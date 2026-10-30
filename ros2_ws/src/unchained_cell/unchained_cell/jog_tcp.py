#!/usr/bin/env python3
"""
Jog the gripper TCP by a small WORLD-frame step — the UI teach panel's
fine-adjust alternative to drag mode.

    ros2 run unchained_cell jog_tcp --dz -2            # 2mm straight down
    ros2 run unchained_cell jog_tcp --dyaw 1.5         # 1.5deg about world Z

Translations in mm (world axes), rotations in deg (applied about the world
axes, pre-multiplied). Every jog is a COLLISION-CHECKED Cartesian move from
the current pose: a step that would collide is refused, not executed.
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from unchained_cell.pick_place import cartesian_move, lookup_tf, _quat_mul

MAX_STEP_MM = 25.0     # refuse absurd jogs
MAX_STEP_DEG = 10.0


def _rpy_quat(roll_deg, pitch_deg, yaw_deg):
    r, p, y = (math.radians(v) / 2.0 for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return Quaternion(
        w=cr * cp * cy + sr * sp * sy,
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dx", type=float, default=0.0, help="mm, world X")
    p.add_argument("--dy", type=float, default=0.0, help="mm, world Y")
    p.add_argument("--dz", type=float, default=0.0, help="mm, world Z (up+)")
    p.add_argument("--droll", type=float, default=0.0, help="deg about world X")
    p.add_argument("--dpitch", type=float, default=0.0, help="deg about world Y")
    p.add_argument("--dyaw", type=float, default=0.0, help="deg about world Z")
    p.add_argument("--vel", type=float, default=0.05)
    p.add_argument("--ee", default="gripper_grasp_link")
    args = p.parse_args()

    if max(abs(args.dx), abs(args.dy), abs(args.dz)) > MAX_STEP_MM:
        print(f"REFUSED: translation step > {MAX_STEP_MM}mm", file=sys.stderr)
        sys.exit(2)
    if max(abs(args.droll), abs(args.dpitch), abs(args.dyaw)) > MAX_STEP_DEG:
        print(f"REFUSED: rotation step > {MAX_STEP_DEG}deg", file=sys.stderr)
        sys.exit(2)

    rclpy.init()
    node = Node("jog_tcp")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    buf = Buffer()
    TransformListener(buf, node)
    cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    execc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    if not (cart.wait_for_service(timeout_sec=10)
            and execc.wait_for_server(timeout_sec=10)):
        print("MoveIt not available", file=sys.stderr)
        sys.exit(1)
    time.sleep(1.5)

    cur = lookup_tf(node, buf, "world", args.ee).transform
    x = cur.translation.x + args.dx / 1000.0
    y = cur.translation.y + args.dy / 1000.0
    z = cur.translation.z + args.dz / 1000.0
    q = _quat_mul(_rpy_quat(args.droll, args.dpitch, args.dyaw), cur.rotation)

    node.get_logger().info(
        f"jog: d=({args.dx:+.1f},{args.dy:+.1f},{args.dz:+.1f})mm "
        f"rpy=({args.droll:+.1f},{args.dpitch:+.1f},{args.dyaw:+.1f})deg "
        f"-> world ({x:+.4f},{y:+.4f},{z:+.4f})")
    ok, *info = cartesian_move(node, cart, execc, "world", args.ee,
                               x, y, z, q, vel=args.vel, max_step=0.002)
    if ok:
        node.get_logger().info(f"JOG OK {info}")
    else:
        node.get_logger().error(
            f"JOG REFUSED/FAILED {info} — step would collide or is unreachable")
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

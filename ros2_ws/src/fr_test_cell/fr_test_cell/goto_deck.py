"""
Plan + execute arm motion to position the gripper TCP above a named deck marker.

Usage:
  ros2 run fr_test_cell goto_deck deck_9_10_pos1
  ros2 run fr_test_cell goto_deck deck_vortex_pos2 --hover 0.20

The TF tree already contains every deck as a fixed link under unchained_junior
(via unchained_full.urdf.xacro). This script looks up TF base_link -> <deck>,
then plans the arm so gripper_grasp_link sits "hover" meters above the deck
origin, with the gripper pointing straight down (world -Z).

For yaw constraints leave wide tolerance — IK has more flexibility that way.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from shape_msgs.msg import SolidPrimitive


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("deck", help="Deck marker link, e.g. deck_9_10_pos1")
    parser.add_argument("--hover", type=float, default=0.15,
                        help="Z offset above deck origin in meters (default 0.15)")
    parser.add_argument("--ee", default="gripper_grasp_link",
                        help="End-effector link (default gripper_grasp_link)")
    parser.add_argument("--group", default="fairino16_v6_group",
                        help="Planning group (default fairino16_v6_group)")
    parser.add_argument("--base", default="base_link",
                        help="Planning base frame (default base_link)")
    parser.add_argument("--vel", type=float, default=0.2,
                        help="Velocity scaling [0..1] (default 0.2)")
    parser.add_argument("--plan-only", action="store_true",
                        help="Plan but do not execute")
    args = parser.parse_args()

    rclpy.init()
    node = Node("goto_deck")

    buf = Buffer()
    TransformListener(buf, node)

    node.get_logger().info(f"Looking up TF {args.base} -> {args.deck}")
    deadline = time.time() + 5.0
    deck_tf = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            deck_tf = buf.lookup_transform(args.base, args.deck, rclpy.time.Time())
            break
        except Exception:
            continue
    if deck_tf is None:
        node.get_logger().error(f"TF {args.base} -> {args.deck} unavailable")
        rclpy.shutdown(); sys.exit(1)

    dx = deck_tf.transform.translation.x
    dy = deck_tf.transform.translation.y
    dz = deck_tf.transform.translation.z
    node.get_logger().info(
        f"{args.deck} in {args.base}: ({dx:.3f}, {dy:.3f}, {dz:.3f})")

    # Build pose: hover above deck, gripper pointing down (180° flip about X)
    target = Pose()
    target.position.x = dx
    target.position.y = dy
    target.position.z = dz + args.hover
    target.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)

    # MoveGroup goal
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = args.group
    req.num_planning_attempts = 10
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = args.vel
    req.max_acceleration_scaling_factor = args.vel

    # Position constraint (small box of tolerance around the target point)
    pc = PositionConstraint()
    pc.header.frame_id = args.base
    pc.link_name = args.ee
    box = SolidPrimitive()
    box.type = SolidPrimitive.BOX
    box.dimensions = [0.01, 0.01, 0.01]
    pc.constraint_region.primitives.append(box)
    pose_at_target = Pose()
    pose_at_target.position = target.position
    pose_at_target.orientation = Quaternion(w=1.0)
    pc.constraint_region.primitive_poses.append(pose_at_target)
    pc.weight = 1.0

    # Orientation constraint (gripper points down; yaw free)
    oc = OrientationConstraint()
    oc.header.frame_id = args.base
    oc.link_name = args.ee
    oc.orientation = target.orientation
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = 3.1416  # free yaw — let IK pick
    oc.weight = 1.0

    constraints = Constraints()
    constraints.position_constraints.append(pc)
    constraints.orientation_constraints.append(oc)
    req.goal_constraints.append(constraints)

    goal.planning_options.plan_only = args.plan_only

    client = ActionClient(node, MoveGroup, "/move_action")
    node.get_logger().info("Waiting for /move_action ...")
    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error("/move_action not available — is move_group running?")
        rclpy.shutdown(); sys.exit(1)

    node.get_logger().info(
        f"Planning {args.ee} -> {args.hover*1000:.0f} mm above {args.deck} "
        f"(plan_only={args.plan_only})")
    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    handle = fut.result()
    if not handle.accepted:
        node.get_logger().error("Goal rejected by move_group")
        rclpy.shutdown(); sys.exit(1)

    rfut = handle.get_result_async()
    rclpy.spin_until_future_complete(node, rfut)
    res = rfut.result().result
    code = res.error_code.val
    if code == 1:
        node.get_logger().info("SUCCESS")
    else:
        node.get_logger().error(f"Failed: MoveItErrorCode {code}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()

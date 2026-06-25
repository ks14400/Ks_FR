"""
Remove all dynamic plates / obstacles from the MoveIt planning scene.

After a pick_place run (or a failed run) the planning scene can be left with:
  - the main plate `the_plate` attached to some link
  - obstacle plates spawned via --also-spawn-at

This script clears all of them so the next run starts from a clean scene
without restarting the launch.

Usage:
  ros2 run fr_test_cell purge_scene
"""
import sys
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fr_test_cell.pick_place import (
    purge_plate,
    purge_obstacle_plates,
)


# Decks where obstacle plates could have been spawned
KNOWN_DECKS = [
    "deck_9_10_pos1", "deck_9_10_pos2", "deck_9_10_pos3",
    "deck_vortex_pos1", "deck_vortex_pos2", "deck_vortex_pos3",
    "deck_vacuum_filtration",
    "table_top",
]


def main():
    rclpy.init()
    node = Node("purge_scene")

    cb_group = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    from moveit_msgs.srv import ApplyPlanningScene
    scene_client = node.create_client(
        ApplyPlanningScene, "/apply_planning_scene", callback_group=cb_group)

    node.get_logger().info("Waiting for /apply_planning_scene service ...")
    if not scene_client.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("apply_planning_scene not available")
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info("Purging the main plate ('the_plate')")
    purge_plate(node, scene_client)

    node.get_logger().info(
        f"Purging obstacle plates from {len(KNOWN_DECKS)} possible decks")
    purge_obstacle_plates(node, scene_client, KNOWN_DECKS)

    node.get_logger().info("DONE — planning scene cleared")
    rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()

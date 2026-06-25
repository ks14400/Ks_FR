"""
Check whether the robot's CURRENT state is valid (no collisions, no joint-limit
violations). Also dumps current joint values and what's in the planning scene.

Use this whenever a plan fails fast (~< 1s) — it tells you definitively
whether the current state is the blocker, no guessing.

Usage:
  ros2 run fr_test_cell check_state
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.srv import GetPlanningScene, GetStateValidity
from moveit_msgs.msg import PlanningSceneComponents


def main():
    rclpy.init()
    node = Node("check_state")
    cb = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    scene_cli = node.create_client(GetPlanningScene, "/get_planning_scene", callback_group=cb)
    check_cli = node.create_client(GetStateValidity, "/check_state_validity", callback_group=cb)

    print("Waiting for services...")
    if not scene_cli.wait_for_service(timeout_sec=15.0):
        print("FAIL: get_planning_scene not available"); sys.exit(1)
    if not check_cli.wait_for_service(timeout_sec=15.0):
        print("FAIL: check_state_validity not available"); sys.exit(1)

    # Fetch the live scene: robot state + ACM + world objects
    req = GetPlanningScene.Request()
    req.components.components = (
        PlanningSceneComponents.ROBOT_STATE
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        | PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )
    fut = scene_cli.call_async(req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done():
        print("FAIL: get_planning_scene timed out"); sys.exit(1)
    scene = fut.result().scene

    print("\n--- Current robot state ---")
    for n, p in zip(scene.robot_state.joint_state.name,
                    scene.robot_state.joint_state.position):
        print(f"  {n:30s} {p:+.4f}")

    print("\n--- Planning scene world ---")
    print(f"  Collision objects: {[co.id for co in scene.world.collision_objects]}")
    aco = scene.robot_state.attached_collision_objects
    print(f"  Attached objects:  "
          f"{[(a.link_name, a.object.id) for a in aco]}")

    def run_validity(label, state, group):
        sv_req = GetStateValidity.Request()
        sv_req.group_name = group
        sv_req.robot_state = state
        fut = check_cli.call_async(sv_req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        res = fut.result()
        print(f"\n--- {label} (group: {group}) ---")
        print(f"  Valid: {res.valid}")
        if res.contacts:
            print(f"  Contacts ({len(res.contacts)}):")
            for c in res.contacts:
                print(f"    {c.contact_body_1!r} <-> {c.contact_body_2!r}  "
                      f"(depth={c.depth:.4f}, types: {c.body_type_1}/{c.body_type_2})")
        else:
            print(f"  No contacts.")

    # 1. current state, arm group
    run_validity("Current state validity (arm group)",
                 scene.robot_state, "fairino16_v6_group")
    # 2. current state, gripper group (failure was in the gripper group)
    run_validity("Current state validity (GRIPPER group)",
                 scene.robot_state, "gripper")
    # 3. hypothetical: gripper FULLY OPEN at current arm pose — this is the goal
    #    state that "open gripper (release)" was trying to reach when it failed.
    print("\n--- Building hypothetical 'gripper fully open' state and checking it ---")
    open_state = scene.robot_state
    joint_state = open_state.joint_state
    # Replace the gripper joints with the fully-open values (master + mimics at 0)
    new_names = list(joint_state.name)
    new_pos = list(joint_state.position)
    for jn in [
        "gripper_finger1_joint", "gripper_finger2_joint",
        "gripper_finger1_finger_joint", "gripper_finger2_finger_joint",
        "gripper_finger1_inner_knuckle_joint",
        "gripper_finger2_inner_knuckle_joint",
        "gripper_finger1_finger_tip_joint",
        "gripper_finger2_finger_tip_joint",
    ]:
        if jn in new_names:
            new_pos[new_names.index(jn)] = 0.0
    open_state.joint_state.name = new_names
    open_state.joint_state.position = new_pos
    open_state.is_diff = False
    run_validity("Gripper-open hypothetical (gripper group)", open_state, "gripper")
    run_validity("Gripper-open hypothetical (arm group)", open_state, "fairino16_v6_group")

    rclpy.shutdown()


if __name__ == "__main__":
    main()

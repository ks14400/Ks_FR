"""
Diagnose why pick_place's descent goal fails.

Calls /check_state_validity for the SAME goal pose pick_place sends, after
spawning the plate exactly the same way. Reports every constraint violation
and every contact pair — actual evidence, not guesses.

Usage:
  ros2 run fr_test_cell diagnose_descent --deck deck_9_10_pos1
"""
import argparse
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint,
    AttachedCollisionObject, CollisionObject, PlanningScene, RobotState,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetPositionIK, GetStateValidity, GetPlanningScene,
)
from moveit_msgs.msg import PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive

# Match pick_place.py
PLATE_X = 0.0855
PLATE_Y = 0.025
PLATE_Z = 0.1278
PLATE_DECK_Y_OFFSET = PLATE_Y / 2.0
PLATE_ID = "the_plate"
DOWN_Q = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--deck", required=True)
    p.add_argument("--grasp-offset", type=float, default=0.06)
    p.add_argument("--ee", default="gripper_grasp_link")
    p.add_argument("--group", default="fairino16_v6_group")
    p.add_argument("--with-plate", action="store_true",
                   help="Also spawn the plate as AttachedCollisionObject before checking")
    p.add_argument("--source-deck", default=None,
                   help="Where to attach plate (default: same as --deck)")
    p.add_argument("--proper-touch", action="store_true",
                   help="When attaching plate, use the same touch_links pick_place "
                        "uses (all gripper + arm + chassis links). Proves whether "
                        "pick_place's whitelist is correctly sized.")
    args = p.parse_args()

    rclpy.init()
    node = Node("diagnose_descent")
    buf = Buffer(); TransformListener(buf, node)

    cb = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    apply_scene_cli = node.create_client(
        ApplyPlanningScene, "/apply_planning_scene", callback_group=cb)
    get_scene_cli = node.create_client(
        GetPlanningScene, "/get_planning_scene", callback_group=cb)
    ik_cli = node.create_client(
        GetPositionIK, "/compute_ik", callback_group=cb)
    validity_cli = node.create_client(
        GetStateValidity, "/check_state_validity", callback_group=cb)

    print("Waiting for services ...")
    for cli, name in [(apply_scene_cli, "apply_planning_scene"),
                       (get_scene_cli, "get_planning_scene"),
                       (ik_cli, "compute_ik"),
                       (validity_cli, "check_state_validity")]:
        if not cli.wait_for_service(timeout_sec=15.0):
            print(f"FAIL: {name} not available"); sys.exit(1)
    print("All services ready.\n")

    # 1. (Optional) attach the plate so the scene matches what pick_place sees
    if args.with_plate:
        deck = args.source_deck or args.deck
        aco = AttachedCollisionObject()
        aco.link_name = deck
        aco.object.id = PLATE_ID
        aco.object.header.frame_id = deck
        box = SolidPrimitive(); box.type = SolidPrimitive.BOX
        box.dimensions = [PLATE_X, PLATE_Y, PLATE_Z]
        aco.object.primitives.append(box)
        pose = Pose(); pose.position.y = PLATE_DECK_Y_OFFSET
        pose.orientation = Quaternion(w=1.0)
        aco.object.primitive_poses.append(pose)
        aco.object.operation = CollisionObject.ADD
        if args.proper_touch:
            # Mirror pick_place's DEFAULT_TOUCH_LINKS + the attachment link
            aco.touch_links = [
                "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link",
                "gripper_finger1_finger_link",     "gripper_finger2_finger_link",
                "gripper_finger1_inner_knuckle_link", "gripper_finger2_inner_knuckle_link",
                "gripper_finger1_knuckle_link",    "gripper_finger2_knuckle_link",
                "gripper_base_link",
                "base_link", "shoulder_link", "upperarm_link",
                "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link",
                "unchained_junior", "pedestal",
                "deck_9_10_pos1", "deck_9_10_pos2", "deck_9_10_pos3",
                "deck_vortex_pos1", "deck_vortex_pos2", "deck_vortex_pos3",
                "deck_vacuum_filtration",
                deck,
            ]
        else:
            aco.touch_links = []  # intentionally empty so we see what touches
        ps = PlanningScene(); ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects.append(aco)
        req = ApplyPlanningScene.Request(); req.scene = ps
        fut = apply_scene_cli.call_async(req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        print(f"Plate attached to {deck} -> success={fut.result().success if fut.done() else 'TIMEOUT'}\n")

    # 2. Dump the planning scene to show what MoveIt actually has
    req = GetPlanningScene.Request()
    req.components.components = (
        PlanningSceneComponents.WORLD_OBJECT_NAMES
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    )
    fut = get_scene_cli.call_async(req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    scene = fut.result().scene
    print("--- Current planning scene ---")
    print(f"  World collision objects: {[co.id for co in scene.world.collision_objects]}")
    print(f"  Attached collision objects: "
          f"{[(a.link_name, a.object.id, a.touch_links) for a in scene.robot_state.attached_collision_objects]}")
    print(f"  ACM entry_names ({len(scene.allowed_collision_matrix.entry_names)}): "
          f"{scene.allowed_collision_matrix.entry_names}")
    print(f"  ACM default_entry_names: {scene.allowed_collision_matrix.default_entry_names}")
    print(f"  ACM default_entry_values: {scene.allowed_collision_matrix.default_entry_values}\n")

    # 3. Look up deck pose
    print(f"Looking up TF base_link -> {args.deck} ...")
    deadline = time.time() + 5.0
    deck_tf = None
    while time.time() < deadline:
        try:
            deck_tf = buf.lookup_transform("base_link", args.deck, rclpy.time.Time())
            break
        except Exception:
            time.sleep(0.05)
    if not deck_tf:
        print("FAIL: TF unavailable"); sys.exit(1)
    dx = deck_tf.transform.translation.x
    dy = deck_tf.transform.translation.y
    dz = deck_tf.transform.translation.z
    print(f"  deck in base_link: ({dx:.3f}, {dy:.3f}, {dz:.3f})\n")

    # 4. Try IK for the descent goal — what joint solution does MoveIt find?
    print("--- IK check for descent goal ---")
    ik_req = GetPositionIK.Request()
    ik_req.ik_request.group_name = args.group
    ik_req.ik_request.ik_link_name = args.ee
    ik_req.ik_request.avoid_collisions = True
    ik_req.ik_request.timeout.sec = 2
    ik_req.ik_request.pose_stamped.header.frame_id = "base_link"
    ik_req.ik_request.pose_stamped.pose.position.x = dx
    ik_req.ik_request.pose_stamped.pose.position.y = dy
    ik_req.ik_request.pose_stamped.pose.position.z = dz + args.grasp_offset
    ik_req.ik_request.pose_stamped.pose.orientation = DOWN_Q
    fut = ik_cli.call_async(ik_req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    res = fut.result()
    print(f"  IK error_code: {res.error_code.val} "
          f"({'SUCCESS' if res.error_code.val == 1 else 'FAILED'})")
    if res.error_code.val == 1:
        names = res.solution.joint_state.name
        pos = res.solution.joint_state.position
        print(f"  IK solution joints:")
        for n, p in zip(names, pos):
            print(f"    {n}: {p:+.3f}")
    print()

    # 5. If IK succeeded, ask /check_state_validity whether that exact state
    # is in collision and which contacts it has.
    if res.error_code.val == 1:
        print("--- State validity check (gives us the contacts) ---")
        sv_req = GetStateValidity.Request()
        sv_req.group_name = args.group
        sv_req.robot_state = res.solution
        fut = validity_cli.call_async(sv_req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.02)
        sv = fut.result()
        print(f"  Valid: {sv.valid}")
        if not sv.valid:
            print(f"  Contacts ({len(sv.contacts)}):")
            for c in sv.contacts:
                print(f"    {c.contact_body_1} <-> {c.contact_body_2} "
                      f"(depth={c.depth:.4f})")
            if sv.constraint_result:
                print(f"  Constraint results: {sv.constraint_result}")
    print()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
Diagnostic: WHY is a gripper pose unreachable/colliding?

For a given TCP pose, runs IK *without* collision avoidance (so a solution is
found if the pose is merely reachable), then checks that state's validity and
prints the EXACT colliding link pairs. Turns "IK failed (no collision-free
solution)" into "wrist2_link hits pxrd_smartlab", with no guessing.

    ros2 run pxrd_cell probe_pose --x -0.127 --y 0.950 --z 0.393
    ros2 run pxrd_cell probe_pose --x ... --grasp-yaw-deg 0 --seeds 5
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.srv import GetPositionIK, GetStateValidity, GetPlanningScene
from moveit_msgs.msg import PlanningSceneComponents

from pxrd_cell.pick_place import DOWN_Q, _quat_mul, _rz_quat, side_grasp_q

GROUP = "fairino10_v6_group"


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--x", type=float, required=True)
    p.add_argument("--y", type=float, required=True)
    p.add_argument("--z", type=float, required=True)
    p.add_argument("--grasp-yaw-deg", type=float, default=0.0)
    p.add_argument("--side", type=str, default="",
                   help="Use SIDE (horizontal) grasp orientation along this "
                        "vx,vy insert axis, e.g. --side=-0.054,0.999")
    p.add_argument("--seeds", type=int, default=3,
                   help="IK retries with random seeds (default 3)")
    args = p.parse_args()

    if args.side.strip():
        vx, vy = (float(t) for t in args.side.split(","))
        q = side_grasp_q((vx, vy))
    else:
        q = _quat_mul(_rz_quat(args.grasp_yaw_deg), DOWN_Q)

    rclpy.init()
    node = Node("probe_pose")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    ik_cli = node.create_client(GetPositionIK, "/compute_ik")
    valid_cli = node.create_client(GetStateValidity, "/check_state_validity")
    scene_cli = node.create_client(GetPlanningScene, "/get_planning_scene")
    for cli, name in ((ik_cli, "compute_ik"), (valid_cli, "check_state_validity"),
                      (scene_cli, "get_planning_scene")):
        if not cli.wait_for_service(timeout_sec=10.0):
            node.get_logger().error(f"{name} unavailable"); sys.exit(1)

    # Current state (incl. attached objects) as IK seed / validity template
    req = GetPlanningScene.Request()
    req.components.components = (PlanningSceneComponents.ROBOT_STATE
                                 | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS)
    fut = scene_cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 5.0:
        time.sleep(0.01)
    cur_state = fut.result().scene.robot_state
    n_att = len(cur_state.attached_collision_objects)
    node.get_logger().info(f"Attached objects on robot: {n_att} "
                           f"({[a.object.id for a in cur_state.attached_collision_objects]})")

    ps = PoseStamped()
    ps.header.frame_id = "base_link"
    ps.pose.position.x = args.x
    ps.pose.position.y = args.y
    ps.pose.position.z = args.z
    ps.pose.orientation = q

    import random
    random.seed(7)
    found_any = False
    for attempt in range(args.seeds):
        ik = GetPositionIK.Request()
        ik.ik_request.group_name = GROUP
        ik.ik_request.pose_stamped = ps
        ik.ik_request.ik_link_name = "gripper_grasp_link"
        ik.ik_request.avoid_collisions = False        # reachability only
        ik.ik_request.timeout.sec = 2
        seed = cur_state
        if attempt > 0:
            # jitter the seed joints to explore other IK branches
            import copy
            seed = copy.deepcopy(cur_state)
            pos = list(seed.joint_state.position)
            names = list(seed.joint_state.name)
            for i, nm in enumerate(names):
                if nm.startswith("j"):
                    pos[i] += random.uniform(-1.5, 1.5)
            seed.joint_state.position = pos
        ik.ik_request.robot_state = seed
        fut = ik_cli.call_async(ik)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.01)
        res = fut.result()
        if res is None or res.error_code.val != 1:
            node.get_logger().info(
                f"seed {attempt}: IK says UNREACHABLE (code "
                f"{None if res is None else res.error_code.val})")
            continue
        found_any = True
        sol = res.solution
        # keep attached objects for the validity check
        sol.attached_collision_objects = cur_state.attached_collision_objects
        sol.is_diff = False
        jmap = dict(zip(sol.joint_state.name, sol.joint_state.position))
        node.get_logger().info(
            "seed %d: reachable, config: %s" % (attempt, ", ".join(
                f"{j}={jmap.get(j, 0):+.2f}" for j in
                ["j1", "j2", "j3", "j4", "j5", "j6"])))
        vreq = GetStateValidity.Request()
        vreq.robot_state = sol
        vreq.group_name = GROUP
        fut = valid_cli.call_async(vreq)
        t0 = time.time()
        while not fut.done() and time.time() - t0 < 5.0:
            time.sleep(0.01)
        vres = fut.result()
        if vres.valid:
            node.get_logger().info(f"  -> VALID (collision-free)!")
        else:
            pairs = {(c.contact_body_1, c.contact_body_2)
                     for c in vres.contacts}
            for a, b in sorted(pairs):
                node.get_logger().info(f"  -> CONTACT: {a}  <->  {b}")
    if not found_any:
        node.get_logger().info("Pose is UNREACHABLE for every seed (not a "
                               "collision problem — out of workspace/orientation)")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

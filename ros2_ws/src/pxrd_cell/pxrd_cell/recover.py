"""
Stranded-arm recovery: get the arm safely OUT of the PXRD bay and home.

When a pick_place run dies mid-cycle (planner failure, Ctrl-C, e-stop), the
arm stops wherever it was. From deep inside the enclosure, a direct plan to
home reliably FAILS — paths can't thread back out of the slot. This tool:

  1. Locates the gripper relative to the PXRD bay.
  2. If inside the danger envelope: Cartesian-slides straight BACK OUT along
     the insert corridor (the known-safe axis), in steps, accepting partial
     fractions and continuing — until clear of the front wall.
  3. Never touches the jaws inside the bay; a plate attached to the gripper
     is carried out (kept in the scene for honest collision checking).
  4. Goes home (multi-attempt, generous planning time).

Usage:
    ros2 run pxrd_cell recover              # extract if needed + home
    ros2 run pxrd_cell recover --no-home    # extract only
    ros2 run pxrd_cell recover --purge      # also clear plate state after
                                            # homing (sim reset only — never
                                            # purge a plate the real gripper
                                            # is physically holding)
"""
import argparse
import math
import sys
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener

from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import (
    GetCartesianPath, GetPlanningScene, ApplyPlanningScene,
)
from moveit_msgs.msg import PlanningSceneComponents

from pxrd_cell.pick_place import (
    HOME_JOINTS,
    cartesian_move,
    lookup_tf,
    plan_arm_to_joints,
    purge_plate,
    side_grasp_q,
    tab_center_xy_in_base,
    _print_config,
)

# Danger envelope around the pxrd_sample bay, in the pxrd_sample (CAD) frame:
# X = across the corridor, Y = up, Z = out toward the robot. The front wall
# sits at Z ~0.25-0.30; anything with the TCP closer than CLEAR_Z may have
# fingers/wrist inside or under the hood.
ENCLOSED_DECK = "pxrd_sample"
DANGER_X = 0.35      # |x| within corridor width
DANGER_Y = (-0.15, 0.60)
DANGER_Z = 0.48      # TCP deck-Z closer than this = potentially inside
CLEAR_Z = 0.55       # extract until TCP deck-Z beyond this
STEP = 0.08          # extraction step (m)
MAX_PULL = 0.70      # give-up bound (m)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--no-home", action="store_true",
                   help="Extract from the bay only; skip going home.")
    p.add_argument("--purge", action="store_true",
                   help="After homing, purge plate scene state (sim reset). "
                        "NEVER use if the real gripper physically holds a plate.")
    p.add_argument("--vel", type=float, default=0.1)
    args = p.parse_args()

    rclpy.init()
    node = Node("recover")
    _print_config(node, args)
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    buf = Buffer()
    TransformListener(buf, node)
    cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    scene_cli = node.create_client(GetPlanningScene, "/get_planning_scene")
    execc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory")
    mg = ActionClient(node, MoveGroup, "/move_action")
    for ok, name in ((cart.wait_for_service(timeout_sec=10), "cartesian"),
                     (scene_cli.wait_for_service(timeout_sec=10), "scene"),
                     (execc.wait_for_server(timeout_sec=10), "execute"),
                     (mg.wait_for_server(timeout_sec=10), "move_group")):
        if not ok:
            node.get_logger().error(f"{name} unavailable — is the sim/bridge up?")
            rclpy.shutdown(); sys.exit(1)
    time.sleep(1.5)  # let TF fill

    # What is the robot holding? (informational + guards --purge misuse)
    req = GetPlanningScene.Request()
    req.components.components = PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    fut = scene_cli.call_async(req)
    t0 = time.time()
    while not fut.done() and time.time() - t0 < 5:
        time.sleep(0.01)
    attached = []
    if fut.done() and fut.result() is not None:
        attached = [(a.object.id, a.link_name) for a in
                    fut.result().scene.robot_state.attached_collision_objects]
    for oid, link in attached:
        node.get_logger().info(f"attached object: '{oid}' on link '{link}'")

    # Gripper TCP in the bay (deck) frame
    tf_deck = lookup_tf(node, buf, ENCLOSED_DECK, "gripper_grasp_link")
    dx = tf_deck.transform.translation.x
    dy = tf_deck.transform.translation.y
    dz = tf_deck.transform.translation.z
    node.get_logger().info(
        f"TCP in {ENCLOSED_DECK} frame: ({dx:+.3f}, {dy:+.3f}, {dz:+.3f})")

    inside = (abs(dx) < DANGER_X
              and DANGER_Y[0] < dy < DANGER_Y[1]
              and dz < DANGER_Z)

    if inside:
        node.get_logger().warn(
            f"TCP inside the bay envelope (deck-Z {dz:.3f} < {DANGER_Z}) — "
            "extracting along the insert corridor")
        # Outward direction in base XY = -(tab->wells axis) at this deck
        deck_tf = lookup_tf(node, buf, "base_link", ENCLOSED_DECK)
        wx, wy = (deck_tf.transform.translation.x,
                  deck_tf.transform.translation.y)
        tabx, taby = tab_center_xy_in_base(node, buf, ENCLOSED_DECK)
        vx, vy = wx - tabx, wy - taby
        n = math.hypot(vx, vy) or 1.0
        vx, vy = vx / n, vy / n            # insert axis (inward)
        q = side_grasp_q((vx, vy))         # keep the horizontal carry attitude
        # NOTE: cartesian_move does NOT execute partial plans — a step either
        # moves the full STEP (frac=1.0) or not at all. Track true progress.
        pulled = 0.0
        while pulled < MAX_PULL:
            tf_now = lookup_tf(node, buf, "base_link", "gripper_grasp_link")
            gx, gy, gz = (tf_now.transform.translation.x,
                          tf_now.transform.translation.y,
                          tf_now.transform.translation.z)
            tgt = (gx - vx * STEP, gy - vy * STEP, gz)
            ok, frac, npts, info = cartesian_move(
                node, cart, execc, "base_link", "gripper_grasp_link",
                tgt[0], tgt[1], tgt[2], q, vel=args.vel, max_step=0.005)
            tf_deck = lookup_tf(node, buf, ENCLOSED_DECK, "gripper_grasp_link")
            dz_now = tf_deck.transform.translation.z
            if ok:
                pulled += STEP
                node.get_logger().info(
                    f"  pulled {STEP*1000:.0f}mm (total {pulled*1000:.0f}mm, "
                    f"deck-Z {dz_now:.3f})")
            else:
                node.get_logger().info(f"  step blocked: {info}")
            if dz_now >= CLEAR_Z:
                node.get_logger().info(
                    f"  CLEAR of the bay (deck-Z {dz_now:.3f} >= {CLEAR_Z})")
                break
            if not ok:
                # Can't pull straight back any further (often: the arm runs
                # out of reach toward its own base). If we're at least out
                # of the danger envelope, that's good enough — home planning
                # takes over from here.
                if dz_now >= DANGER_Z:
                    node.get_logger().warn(
                        f"  cannot retract further, but deck-Z {dz_now:.3f} "
                        f">= {DANGER_Z} (outside danger envelope) — proceeding "
                        "to home")
                    break
                node.get_logger().error(
                    f"  extraction stalled INSIDE the bay (deck-Z {dz_now:.3f}"
                    f" < {DANGER_Z}) — arm may be wedged; inspect in rviz")
                rclpy.shutdown(); sys.exit(2)
        else:
            node.get_logger().error("  pulled MAX with no clearance — abort")
            rclpy.shutdown(); sys.exit(2)
    else:
        node.get_logger().info("TCP outside the bay envelope — no extraction needed")

    if args.no_home:
        node.get_logger().info("DONE (extract only, --no-home)")
        rclpy.shutdown(); sys.exit(0)

    node.get_logger().info("Going home (multi-attempt)...")
    ok = False
    for attempt, ptime in enumerate((10.0, 20.0, 30.0), 1):
        ok, info = plan_arm_to_joints(
            node, mg, HOME_JOINTS, vel=args.vel,
            planning_time=ptime, planning_attempts=30,
            planner_id="RRTConnect", exec_client=execc)
        node.get_logger().info(f"  home attempt {attempt}: "
                               f"{'OK — ' + info if ok else 'failed: ' + str(info)}")
        if ok:
            break
    if not ok:
        node.get_logger().error(
            "Could not reach home even after extraction. Arm is at least "
            "clear of the bay — inspect in rviz.")
        rclpy.shutdown(); sys.exit(1)

    if args.purge:
        held = [oid for oid, link in attached if "gripper" in link]
        if held:
            node.get_logger().warn(
                f"--purge with object(s) {held} attached to the GRIPPER: "
                "only do this if the physical gripper is NOT holding a plate!")
        node.get_logger().info("Purging plate scene state...")
        scene_apply = node.create_client(ApplyPlanningScene,
                                         "/apply_planning_scene")
        scene_apply.wait_for_service(timeout_sec=5)
        purge_plate(node, scene_apply)

    node.get_logger().info("RECOVERED — arm is home")
    rclpy.shutdown(); sys.exit(0)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""585mm PXRD insert DIAGNOSTIC — why does the carried plate stop short of the
sample, and at what height/depth does it clear?

Replays the taught pick+carry to the bay stage (fast), then probes the INSERT:
  * Cartesian toward the real seat at a sweep of heights (seat_z +/- 20mm),
    reporting the reachable fraction and the stop-point TCP depth.
  * At the best stop, names the exact colliding link pair (check_state_validity).
  * Tries an OMPL joint goal to the captured real seat config (CAP) with the
    plate attached — if a collision-free plan exists, the carried plate CAN
    reach the seat and we just need to route to it (not a hard collision).

Read-only w.r.t. files; moves the arm in sim only. Prints a report.
"""
import json
import math
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import (GetPositionIK, GetPlanningScene, ApplyPlanningScene,
                             GetCartesianPath, GetPositionFK, GetStateValidity)
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener

from pxrd_cell.pick_place import (
    apply_plate, lerp_path, plan_arm_to_joints, attach_plate_to_gripper,
    attach_plate_to_deck, purge_plate, get_current_robot_state, side_grasp_q,
    lookup_tf, _tf_to_T, _plate_rest_T, _quat_to_R, TAB_CENTER_X, compute_ik)

TAUGHT = "/home/aimatx_nuc3/Ks_FR/cad/pxrd/taught_path_585_full.json"
SEAT = (0.613, 0.750, 0.631)
CAP = {"j1": -2.416711, "j2": -1.372981, "j3": 1.568905,
       "j4": -0.164911, "j5": 1.586216, "j6": 1.571256}


def tilt(buf):
    tf = buf.lookup_transform("base_link", "gripper_grasp_link", rclpy.time.Time())
    R = _quat_to_R(tf.transform.rotation)
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2][0]))))


def main():
    with open(TAUGHT) as f:
        data = json.load(f)
    wps = data["waypoints"]

    rclpy.init()
    node = Node("insert_diag")
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    buf = Buffer(); TransformListener(buf, node)
    apply_plate("v3")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    gs = node.create_client(GetPlanningScene, "/get_planning_scene")
    scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    fk = node.create_client(GetPositionFK, "/compute_fk")
    sv = node.create_client(GetStateValidity, "/check_state_validity")
    for c in (ik, gs, scene, cart, fk, sv):
        c.wait_for_service(timeout_sec=15)
    mg = ActionClient(node, MoveGroup, "/move_action"); mg.wait_for_server(timeout_sec=15)
    exc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory"); exc.wait_for_server(timeout_sec=15)
    time.sleep(1.0)

    def L(m): print(m, flush=True)

    # insert axis (tab->wells) in base XY, from the sample deck geometry
    st = lookup_tf(node, buf, "base_link", "pxrd_sample")
    sx, sy = st.transform.translation.x, st.transform.translation.y
    T = _tf_to_T(st)
    tabx, taby, _ = (T @ _plate_rest_T() @ np.array([TAB_CENTER_X, 0, 0, 1]))[:3]
    vx, vy = sx - tabx, sy - taby
    n = math.hypot(vx, vy) or 1.0
    vx, vy = vx / n, vy / n
    q = side_grasp_q((vx, vy))

    # ---- replay pick + carry to the bay stage (fast) ----
    src = data.get("source", "table_top")
    tags = [w["tag"] for w in wps]
    ai = next((i for i, t in enumerate(tags) if "advance" in t), 1)
    purge_plate(node, scene); attach_plate_to_deck(node, scene, src)
    for wp in wps[:ai + 1]:
        plan_arm_to_joints(node, mg, wp["joints"], vel=0.4, exec_client=exc)
    attach_plate_to_gripper(node, scene, src, buf=buf, source_deck=src, seat_tab=True)
    carry = [w["joints"] for w in wps[ai + 1:-1]]   # home..bay stage
    lerp_path(node, gs, exc, carry, vel=0.4)
    L(f"\n=== at bay stage, plate held, tilt={tilt(buf):.1f} deg ===")
    stage_cfg = get_current_robot_state(node, gs)

    def cart_to(x, y, z):
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"; req.group_name = "fairino10_v6_group"
        req.link_name = "gripper_grasp_link"; req.max_step = 0.005
        req.jump_threshold = 5.0; req.avoid_collisions = True
        p = Pose(); p.position.x = x; p.position.y = y; p.position.z = z
        p.orientation = q; req.waypoints = [p]
        fu = cart.call_async(req); dl = time.time() + 10
        while not fu.done() and time.time() < dl: time.sleep(0.02)
        r = fu.result()
        return r.fraction

    # ---- height sweep: Cartesian to the seat XY at several heights ----
    L("\n--- insert height sweep (Cartesian to seat XY, avoid_collisions) ---")
    best = None
    for dz in (-0.02, -0.01, 0.0, 0.01, 0.02, 0.03):
        frac = cart_to(SEAT[0], SEAT[1], SEAT[2] + dz)
        short_mm = (1 - frac) * 0.20 * 1000  # 200mm stage->seat span approx
        L(f"  z={SEAT[2]+dz:.3f} (seat{dz*1000:+.0f}mm): frac={frac:.2f}  "
          f"(~{short_mm:.0f}mm short)")
        if best is None or frac > best[1]:
            best = (dz, frac)
    L(f"  best height: seat{best[0]*1000:+.0f}mm @ frac={best[1]:.2f}")

    # ---- name the colliding pair at the seat (plate attached) ----
    import copy as _copy
    L("\n--- colliding pair at the seat (plate attached) ---")
    jd = compute_ik(node, ik, "base_link", "gripper_grasp_link",
                    SEAT[0], SEAT[1], SEAT[2] + best[0], q, stage_cfg,
                    avoid_collisions=False)
    if jd is None:
        L("  no IK at the seat pose (seed=stage) — reach/branch issue")
    else:
        rs = _copy.deepcopy(stage_cfg)
        pos = list(rs.joint_state.position)
        for k, nm in enumerate(rs.joint_state.name):
            if nm in jd:
                pos[k] = jd[nm]
        rs.joint_state.position = pos
        req = GetStateValidity.Request()
        req.robot_state = rs
        req.group_name = "fairino10_v6_group"
        fu = sv.call_async(req); dl = time.time() + 5
        while not fu.done() and time.time() < dl: time.sleep(0.02)
        r = fu.result()
        if r and r.valid:
            L("  seat pose is COLLISION-FREE (carried) — can route straight in!")
        else:
            pairs = {f"{c.contact_body_1} <-> {c.contact_body_2}"
                     for c in (r.contacts if r else [])}
            L(f"  seat pose BLOCKED. contacts: {sorted(pairs) or '(none reported)'}")

    # ---- can OMPL reach the captured real seat config with the plate on? ----
    L("\n--- OMPL to captured real seat config CAP (plate attached) ---")
    ok, info = plan_arm_to_joints(node, mg, CAP, vel=0.2, exec_client=None,
                                  planning_time=8.0)
    L(f"  plan to CAP: {'OK (collision-free path exists)' if ok else 'FAIL'} "
      f"({str(info)[:70]})")
    L(f"  (CAP tilt would be ~1.8 deg)\n")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

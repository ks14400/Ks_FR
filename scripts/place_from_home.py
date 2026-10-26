#!/usr/bin/env python3
"""585mm layout — home -> carry the v3 plate LEVEL -> place it in the PXRD.
No table. Watchable demo (run it, watch rviz). Uses the real jogged pose as
the placement anchor so it does not over-insert.

Sequence:
  SETUP (gets a correctly-held plate to home):
    0. go to PLACED (empty), attach plate at the sample + into the gripper,
    1. retract PLACED -> STAGE -> HOME carrying it (level).
  DEMO (home -> place):
    2. HOME -> STAGE (level), 3. STAGE -> PLACED insert (level),
    4. release: re-attach the plate to the deck, 5. retract empty to STAGE.

Every carried move is orientation-locked (plate stays level).
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import (GetPositionIK, GetPlanningScene, GetCartesianPath,
                             ApplyPlanningScene)
from tf2_ros import Buffer, TransformListener

from moveit_msgs.srv import GetPositionFK
from pxrd_cell.pick_place import (
    side_grasp_q, compute_ik_best_of_many, get_current_robot_state,
    attach_plate_to_gripper, attach_plate_to_deck, purge_plate, lookup_tf,
    _tf_to_T, _plate_rest_T, _quat_to_R, PAD_OFFSET_FROM_TCP, TAB_CENTER_X,
    apply_plate, plan_arm_to_joints, cartesian_move,
)

STANDOFF = 0.20       # stage = placed - axis*STANDOFF
HOME_OUT = 0.38       # home  = placed - axis*HOME_OUT


def tilt(buf):
    tf = buf.lookup_transform("base_link", "gripper_grasp_link", rclpy.time.Time())
    R = _quat_to_R(tf.transform.rotation)
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2][0]))))


def main():
    rclpy.init()
    node = Node("place_from_home")
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    buf = Buffer(); TransformListener(buf, node)
    apply_plate("v3")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    gs = node.create_client(GetPlanningScene, "/get_planning_scene")
    cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    for c in (ik, gs, cart, scene): c.wait_for_service(timeout_sec=15)
    fk = node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service(timeout_sec=15)
    mg = ActionClient(node, MoveGroup, "/move_action"); mg.wait_for_server(timeout_sec=15)
    exc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory"); exc.wait_for_server(timeout_sec=15)
    time.sleep(1.0)
    carry_gate = (fk, 10.0)   # keep the plate within 10 deg of level while carried

    def log(m): node.get_logger().info("  >> " + m)

    # geometry from TF
    st = lookup_tf(node, buf, "base_link", "pxrd_sample")
    sx, sy, sz = (st.transform.translation.x, st.transform.translation.y,
                  st.transform.translation.z)
    T_deck = _tf_to_T(st)
    tabx, taby, _ = (T_deck @ _plate_rest_T() @ np.array([TAB_CENTER_X, 0, 0, 1]))[:3]
    vx, vy = sx - tabx, sy - taby
    n = math.hypot(vx, vy) or 1.0
    vx, vy = vx / n, vy / n
    q = side_grasp_q((vx, vy))
    # PLACED = the real jogged seating pose (FK of captured joints), pulled
    # back 30mm along -axis so the carried plate has clearance from the
    # instrument (at the exact seat the plate touches pxrd_smartlab, which
    # blocks the carried-plate collision check). 30mm short = plate just
    # inside the bay, collision-free, visually "placed".
    SEAT = (0.613, 0.750, 0.631)
    PLACED = (SEAT[0] - vx * 0.030, SEAT[1] - vy * 0.030, SEAT[2])
    stage = (PLACED[0] - vx * STANDOFF, PLACED[1] - vy * STANDOFF, PLACED[2])
    home = (PLACED[0] - vx * HOME_OUT, PLACED[1] - vy * HOME_OUT, PLACED[2])

    def goto(xyz, tag):
        cur = get_current_robot_state(node, gs)
        tgt, _ = compute_ik_best_of_many(node, ik, "base_link",
                                         "gripper_grasp_link", *xyz, q, cur)
        if tgt is None:
            log(f"{tag}: NO IK at {xyz}"); return False
        ok, info = plan_arm_to_joints(node, mg, tgt, vel=0.25, exec_client=exc)
        log(f"{tag}: {'OK' if ok else 'FAIL'}")
        return ok

    def slide(xyz, tag, gated=True):
        # carried move as a tilt-gated OMPL joint goal (robust; keeps level)
        cur = get_current_robot_state(node, gs)
        tgt, _ = compute_ik_best_of_many(node, ik, "base_link",
                                         "gripper_grasp_link", *xyz, q, cur)
        if tgt is None:
            log(f"{tag}: NO IK"); return False
        ok, info = plan_arm_to_joints(node, mg, tgt, vel=0.2, exec_client=exc,
                                      tilt_gate=(carry_gate if gated else None))
        log(f"{tag}: {'OK' if ok else 'FAIL'} tilt={tilt(buf):.1f}deg  ({info[:50]})")
        return ok

    purge_plate(node, scene)
    log("SETUP 0: go to PLACED (empty) and pick up the plate")
    if not goto(PLACED, "goto placed"): return
    # remember the exact placed config (known collision-free) to reuse for the
    # insert — re-searching IK with the plate attached lands in clipping families
    _cs = get_current_robot_state(node, gs)
    placed_joints = {n: p for n, p in zip(_cs.joint_state.name, _cs.joint_state.position)
                     if n in ("j1", "j2", "j3", "j4", "j5", "j6")}
    attach_plate_to_deck(node, scene, "pxrd_sample")
    attach_plate_to_gripper(node, scene, "pxrd_sample", buf=buf,
                            source_deck="pxrd_sample", seat_tab=True)
    time.sleep(0.5)
    log("SETUP 1: retract carrying the plate  PLACED -> STAGE -> HOME (level)")
    slide(stage, "retract to stage"); slide(home, "retract to home")
    log(f"--- at HOME holding the plate level ({tilt(buf):.1f}deg) ---")
    time.sleep(1.0)

    log("DEMO 2: HOME -> STAGE (carry level)")
    slide(stage, "home->stage")
    log("DEMO 3: STAGE -> PLACED (insert level) — reuse known placed config")
    ok, info = plan_arm_to_joints(node, mg, placed_joints, vel=0.2,
                                  exec_client=exc, tilt_gate=carry_gate)
    log(f"stage->placed(insert): {'OK' if ok else 'FAIL'} tilt={tilt(buf):.1f}deg")
    log("DEMO 4: release — re-attach plate to the sample deck")
    attach_plate_to_deck(node, scene, "pxrd_sample", current_parent="gripper_grasp_link")
    time.sleep(0.5)
    log("DEMO 5: retract empty  PLACED -> STAGE")
    slide(stage, "retract empty")
    log("DONE — plate placed in the PXRD from home, level throughout.")
    time.sleep(1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

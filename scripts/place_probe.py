#!/usr/bin/env python3
"""Probe a PLACE sequence (no table): attach the v3 plate to the gripper held
LEVEL, stage outside the PXRD bay, and insert onto the sample keeping the
plate level. Geometry is read from TF, so it respects the 585mm/rotated mount.

Prints one JSON line: stage_ok, stage_tilt_deg, insert_fraction,
insert_tilt_deg, verdict.

Usage:
  place_probe.py [--stage-standoff M] [--pinch-dz M] [--insert-depth M]
"""
import argparse
import json
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

from pxrd_cell.pick_place import (
    side_grasp_q, compute_ik_best_of_many, get_current_robot_state,
    attach_plate_to_gripper, attach_plate_to_deck, lookup_tf, _tf_to_T,
    _plate_rest_T, _quat_to_R, PAD_OFFSET_FROM_TCP, TAB_CENTER_X, TAB_PINCH_Y,
    apply_plate, plan_arm_to_joints, cartesian_move,
)


def tilt(node, buf):
    tf = buf.lookup_transform("base_link", "gripper_grasp_link", rclpy.time.Time())
    R = _quat_to_R(tf.transform.rotation)
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2][0]))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--stage-standoff", type=float, default=0.20)
    ap.add_argument("--pinch-dz", type=float, default=0.0)
    ap.add_argument("--insert-depth", type=float, default=0.20)
    a = ap.parse_args()
    out = {"stage_standoff": a.stage_standoff, "pinch_dz": a.pinch_dz,
           "insert_depth": a.insert_depth, "stage_ok": False,
           "stage_tilt_deg": None, "insert_fraction": 0.0,
           "insert_tilt_deg": None, "verdict": "fail", "detail": ""}

    rclpy.init()
    node = Node("place_probe")
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    buf = Buffer(); TransformListener(buf, node)
    apply_plate("v3")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    gs = node.create_client(GetPlanningScene, "/get_planning_scene")
    cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    for c in (ik, gs, cart, scene):
        if not c.wait_for_service(timeout_sec=15):
            out["detail"] = "services unavailable"; print(json.dumps(out)); return
    mg = ActionClient(node, MoveGroup, "/move_action"); mg.wait_for_server(timeout_sec=15)
    exc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory"); exc.wait_for_server(timeout_sec=15)
    time.sleep(1.0)

    # geometry from TF (respects the mount rotation)
    st = lookup_tf(node, buf, "base_link", "pxrd_sample")
    sx, sy, sz = (st.transform.translation.x, st.transform.translation.y,
                  st.transform.translation.z)
    T_deck = _tf_to_T(st)
    tabx, taby, _ = (T_deck @ _plate_rest_T() @ np.array([TAB_CENTER_X, 0, 0, 1]))[:3]
    vx, vy = sx - tabx, sy - taby
    n = math.hypot(vx, vy) or 1.0
    vx, vy = vx / n, vy / n                     # insert axis (tab->wells) base XY
    q = side_grasp_q((vx, vy))
    pinch_z = sz + TAB_PINCH_Y + a.pinch_dz
    in_x = tabx - vx * PAD_OFFSET_FROM_TCP      # TCP at insert END (gripper outside)
    in_y = taby - vy * PAD_OFFSET_FROM_TCP
    stage_x = in_x - vx * a.stage_standoff
    stage_y = in_y - vy * a.stage_standoff
    out["detail"] = (f"sample=({sx:.3f},{sy:.3f},{sz:.3f}) axis=({vx:+.2f},{vy:+.2f}) "
                     f"stage=({stage_x:.3f},{stage_y:.3f},{pinch_z:.3f})")

    cur = get_current_robot_state(node, gs)
    tgt, _ = compute_ik_best_of_many(node, ik, "base_link", "gripper_grasp_link",
                                     stage_x, stage_y, pinch_z, q, cur)
    if tgt is None:
        out["verdict"] = "stage-no-ik"; print(json.dumps(out)); return
    ok, _ = plan_arm_to_joints(node, mg, tgt, vel=0.3, exec_client=exc)
    if not ok:
        out["verdict"] = "stage-plan-fail"; print(json.dumps(out)); return
    time.sleep(0.4)
    out["stage_ok"] = True
    out["stage_tilt_deg"] = round(tilt(node, buf), 2)

    # attach plate to the gripper as held
    attach_plate_to_deck(node, scene, "pxrd_sample")
    attach_plate_to_gripper(node, scene, "pxrd_sample", buf=buf,
                            source_deck="pxrd_sample", seat_tab=True)
    time.sleep(0.4)

    ok, frac, npts, cinfo = cartesian_move(node, cart, exc, "base_link",
                                           "gripper_grasp_link", in_x, in_y,
                                           pinch_z, q, vel=0.2, max_step=0.005)
    out["insert_fraction"] = round(frac, 3)
    out["insert_tilt_deg"] = round(tilt(node, buf), 2)
    if ok and frac > 0.99:
        out["verdict"] = "PLACED"
    elif frac > 0.5:
        out["verdict"] = f"partial-{int(frac*100)}pct"
    else:
        out["verdict"] = f"insert-blocked-{int(frac*100)}pct"
    print(json.dumps(out))
    rclpy.shutdown()


if __name__ == "__main__":
    main()

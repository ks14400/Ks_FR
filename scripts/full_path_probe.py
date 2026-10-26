#!/usr/bin/env python3
"""585mm full pick->home->place probe (no hard-coded waypoints).

Flow: pick the v3 plate off the table -> lift to a HOME above the table
(outside the PXRD, to the side) -> carry level to the bay stage -> insert
and seat on the sample. Every carried move is tilt-gated (plate stays level).
Placement is planner-found (IK at the seat), NOT anchored to any jog pose.

Table position comes from the launched sim (read via TF). Prints one JSON line.

Usage: full_path_probe.py [--pick-standoff M] [--bay-standoff M] [--home-lift M]
"""
import argparse, json, math, sys, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import (GetPositionIK, GetPlanningScene, GetCartesianPath,
                             ApplyPlanningScene, GetPositionFK)
from tf2_ros import Buffer, TransformListener
from pxrd_cell.pick_place import (
    side_grasp_q, compute_ik_best_of_many, get_current_robot_state,
    attach_plate_to_gripper, attach_plate_to_deck, purge_plate, lookup_tf,
    _tf_to_T, _plate_rest_T, _quat_to_R, PAD_OFFSET_FROM_TCP, TAB_CENTER_X,
    TAB_PINCH_Y, apply_plate, plan_arm_to_joints, plan_side_stage)


def tilt(buf):
    tf = buf.lookup_transform("base_link", "gripper_grasp_link", rclpy.time.Time())
    R = _quat_to_R(tf.transform.rotation)
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2][0]))))


def deck_tab_axis(node, buf, deck):
    """Return (wells_xy, tab_xy, axis_xy, deck_z) for a deck's resting plate."""
    st = lookup_tf(node, buf, "base_link", deck)
    sx, sy, sz = (st.transform.translation.x, st.transform.translation.y,
                  st.transform.translation.z)
    T = _tf_to_T(st)
    tabx, taby, _ = (T @ _plate_rest_T() @ np.array([TAB_CENTER_X, 0, 0, 1]))[:3]
    vx, vy = sx - tabx, sy - taby
    n = math.hypot(vx, vy) or 1.0
    return (sx, sy), (tabx, taby), (vx / n, vy / n), sz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pick-standoff", type=float, default=0.06)
    ap.add_argument("--bay-standoff", type=float, default=0.20)
    ap.add_argument("--home-lift", type=float, default=0.18)
    a = ap.parse_args()
    out = {"pick_ok": False, "carry_home_ok": False, "carry_stage_ok": False,
           "place_ok": False, "max_tilt_deg": 0.0, "verdict": "fail", "detail": ""}

    rclpy.init(); node = Node("full_path_probe")
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    buf = Buffer(); TransformListener(buf, node); apply_plate("v3")
    ik = node.create_client(GetPositionIK, "/compute_ik")
    gs = node.create_client(GetPlanningScene, "/get_planning_scene")
    scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    for c in (ik, gs, scene):
        if not c.wait_for_service(timeout_sec=15):
            out["detail"] = "services down"; print(json.dumps(out)); return
    mg = ActionClient(node, MoveGroup, "/move_action"); mg.wait_for_server(timeout_sec=15)
    exc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory"); exc.wait_for_server(timeout_sec=15)
    fk = node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service(timeout_sec=15)
    gate = (fk, 10.0)
    time.sleep(1.0)
    tilts = []

    def goto(xyz, q, tag, gated):
        cur = get_current_robot_state(node, gs)
        tgt, _ = compute_ik_best_of_many(node, ik, "base_link",
                                         "gripper_grasp_link", *xyz, q, cur)
        if tgt is None:
            node.get_logger().info(f"  {tag}: NO IK"); return False
        ok, info = plan_arm_to_joints(node, mg, tgt, vel=0.3, exec_client=exc,
                                      tilt_gate=(gate if gated else None))
        t = tilt(buf); tilts.append(t)
        node.get_logger().info(f"  {tag}: {'OK' if ok else 'FAIL'} tilt={t:.1f}")
        return ok

    # ---- TABLE geometry
    (tsx, tsy), (ttabx, ttaby), (tvx, tvy), tsz = deck_tab_axis(node, buf, "table_top")
    tq = side_grasp_q((tvx, tvy))
    t_pinch = tsz + TAB_PINCH_Y
    t_adv = (ttabx - tvx * PAD_OFFSET_FROM_TCP, ttaby - tvy * PAD_OFFSET_FROM_TCP)
    t_app = (t_adv[0] - tvx * a.pick_standoff, t_adv[1] - tvy * a.pick_standoff, t_pinch)
    # ---- BAY geometry
    (bsx, bsy), (btabx, btaby), (bvx, bvy), bsz = deck_tab_axis(node, buf, "pxrd_sample")
    bq = side_grasp_q((bvx, bvy))
    b_pinch = bsz + TAB_PINCH_Y
    b_place = (btabx - bvx * PAD_OFFSET_FROM_TCP - bvx * 0.03,   # 30mm clearance
               btaby - bvy * PAD_OFFSET_FROM_TCP - bvy * 0.03, b_pinch)
    b_stage = (b_place[0] - bvx * a.bay_standoff, b_place[1] - bvy * a.bay_standoff, b_pinch)
    out["detail"] = (f"table=({tsx:.2f},{tsy:.2f},{tsz:.2f}) "
                     f"bay_place=({b_place[0]:.2f},{b_place[1]:.2f},{b_place[2]:.2f})")

    purge_plate(node, scene)
    attach_plate_to_deck(node, scene, "table_top")
    # PICK: approach behind the table tab, advance, grab, lift to HOME
    if not goto(t_app, tq, "pick approach", gated=False):
        out["verdict"] = "pick-approach-no-ik"; print(json.dumps(out)); return
    if not goto((t_adv[0], t_adv[1], t_pinch), tq, "pick advance", gated=False):
        out["verdict"] = "pick-advance-fail"; print(json.dumps(out)); return
    attach_plate_to_gripper(node, scene, "table_top", buf=buf,
                            source_deck="table_top", seat_tab=True)
    time.sleep(0.4)
    out["pick_ok"] = True
    # HOME = lifted straight up above the table (outside the PXRD, on the side)
    home = (t_adv[0], t_adv[1], t_pinch + a.home_lift)
    out["carry_home_ok"] = goto(home, tq, "lift to HOME (side)", gated=True)
    if not out["carry_home_ok"]:
        out["verdict"] = "carry-home-fail"; print(json.dumps(out)); return
    # CARRY to the bay stage via the staged/gated ladder (flat carry through
    # the big swing — a single gated goto can't hold level across it).
    ok, info = plan_side_stage(node, mg, ik, gs, *b_stage, bq,
                               vel=0.3, planning_time=8.0, planning_attempts=20,
                               planner_id="RRTConnect", exec_client=exc,
                               hint=None, j1_first=False, tilt_gate=gate)
    tcur = tilt(buf); tilts.append(tcur)
    node.get_logger().info(f"  carry HOME->bay stage (ladder): "
                           f"{'OK' if ok else 'FAIL'} tilt={tcur:.1f}  ({str(info)[:40]})")
    out["carry_stage_ok"] = ok
    if not out["carry_stage_ok"]:
        out["verdict"] = "carry-stage-fail"; print(json.dumps(out)); return
    out["place_ok"] = goto(b_place, bq, "insert+place in bay", gated=True)
    out["max_tilt_deg"] = round(max(tilts) if tilts else 0.0, 2)
    out["verdict"] = "FULL-PATH" if out["place_ok"] else "place-fail"
    print(json.dumps(out))
    rclpy.shutdown()


if __name__ == "__main__":
    main()

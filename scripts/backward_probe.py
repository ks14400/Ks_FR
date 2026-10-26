#!/usr/bin/env python3
"""585mm BACKWARD probe: anchor on the fixed PXRD placement, work out to the
table. The bay is the constant (placement proven by real motion). We:
  1. place the plate in the bay (IK the seat, attach),
  2. retract out to the bay STAGE keeping level,
  3. carry STAGE -> HOME keeping level (HOME is between bay and table),
  4. check the TABLE pick is reachable from HOME.
Home IK is SEEDED from the bay-carry family so the carry stays in-family
(a single big cross-family swing is what tips the plate). Prints one JSON line.

Usage: backward_probe.py [--home-frac F] [--bay-standoff M]
"""
import argparse, json, math, sys, threading, time
import numpy as np, rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import (GetPositionIK, GetPlanningScene, ApplyPlanningScene,
                             GetPositionFK)
from tf2_ros import Buffer, TransformListener
from pxrd_cell.pick_place import (
    side_grasp_q, compute_ik_best_of_many, compute_ik, get_current_robot_state,
    attach_plate_to_gripper, attach_plate_to_deck, purge_plate, lookup_tf,
    _tf_to_T, _plate_rest_T, _quat_to_R, PAD_OFFSET_FROM_TCP, TAB_CENTER_X,
    TAB_PINCH_Y, apply_plate, plan_arm_to_joints)


def tilt(buf):
    tf = buf.lookup_transform("base_link", "gripper_grasp_link", rclpy.time.Time())
    R = _quat_to_R(tf.transform.rotation)
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2][0]))))


def deck(node, buf, name):
    st = lookup_tf(node, buf, "base_link", name)
    sx, sy, sz = (st.transform.translation.x, st.transform.translation.y,
                  st.transform.translation.z)
    T = _tf_to_T(st)
    tabx, taby, _ = (T @ _plate_rest_T() @ np.array([TAB_CENTER_X, 0, 0, 1]))[:3]
    vx, vy = sx - tabx, sy - taby
    n = math.hypot(vx, vy) or 1.0
    return (sx, sy, sz), (tabx, taby), (vx / n, vy / n)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--home-frac", type=float, default=0.5)
    ap.add_argument("--bay-standoff", type=float, default=0.20)
    a = ap.parse_args()
    out = {"home_frac": a.home_frac, "place_ok": False, "retract_ok": False,
           "carry_home_ok": False, "table_pick_reachable": False,
           "max_tilt_deg": 0.0, "verdict": "fail", "detail": ""}
    rclpy.init(); node = Node("backward_probe")
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
    gate = (fk, 10.0); tilts = []
    time.sleep(1.0)

    (bsx, bsy, bsz), (btabx, btaby), (bvx, bvy) = deck(node, buf, "pxrd_sample")
    (tsx, tsy, tsz), (ttabx, ttaby), (tvx, tvy) = deck(node, buf, "table_top")
    bq = side_grasp_q((bvx, bvy)); tq = side_grasp_q((tvx, tvy))
    # Anchor the bay placement on the REAL achievable seat (FK of the captured
    # jogged pose at this confirmed 585mm geometry), pulled back 30mm for
    # carried-plate clearance. This is the proven-reachable placement.
    SEAT = (0.613, 0.750, 0.631)
    place = (SEAT[0] - bvx * 0.03, SEAT[1] - bvy * 0.03, SEAT[2])
    bpz = place[2]
    stage = (place[0] - bvx * a.bay_standoff, place[1] - bvy * a.bay_standoff, bpz)
    t_adv = (ttabx - tvx * PAD_OFFSET_FROM_TCP, ttaby - tvy * PAD_OFFSET_FROM_TCP, tsz + TAB_PINCH_Y)
    # HOME = blend from the bay STAGE toward the table pick, at bay height.
    home = (stage[0] + a.home_frac * (t_adv[0] - stage[0]),
            stage[1] + a.home_frac * (t_adv[1] - stage[1]), bpz)
    out["detail"] = (f"bay_place=({place[0]:.2f},{place[1]:.2f},{place[2]:.2f}) "
                     f"home=({home[0]:.2f},{home[1]:.2f},{home[2]:.2f}) "
                     f"table_adv=({t_adv[0]:.2f},{t_adv[1]:.2f},{t_adv[2]:.2f})")

    def goto(xyz, q, tag, seed_state=None, gated=True):
        cur = seed_state or get_current_robot_state(node, gs)
        tgt, _ = compute_ik_best_of_many(node, ik, "base_link",
                                         "gripper_grasp_link", *xyz, q, cur)
        if tgt is None:
            node.get_logger().info(f"  {tag}: NO IK"); return False, None
        ok, info = plan_arm_to_joints(node, mg, tgt, vel=0.3, exec_client=exc,
                                      tilt_gate=(gate if gated else None))
        t = tilt(buf); tilts.append(t)
        node.get_logger().info(f"  {tag}: {'OK' if ok else 'FAIL'} tilt={t:.1f}")
        return ok, tgt

    purge_plate(node, scene)
    attach_plate_to_deck(node, scene, "pxrd_sample")
    # 0. establish the arm AT the bay (captured reachable config, joint goal —
    # OMPL joint-space handles the big move from the neutral start).
    CAP = {"j1": -2.416711, "j2": -1.372981, "j3": 1.568905,
           "j4": -0.164911, "j5": 1.586216, "j6": 1.571256}
    ok, info = plan_arm_to_joints(node, mg, CAP, vel=0.3, exec_client=exc,
                                  planning_time=25.0, planning_attempts=30,
                                  planner_id="RRTConnect")
    node.get_logger().info(f"  seed to bay config: {'OK' if ok else 'FAIL'} ({str(info)[:50]})")
    if not ok:
        out["verdict"] = "bay-seed-fail"; print(json.dumps(out)); return
    time.sleep(0.3)
    # 1. refine to the exact (clearance) placement pose
    ok, placed_cfg = goto(place, bq, "place in bay", gated=False)
    if not ok:
        out["verdict"] = "place-no-ik"; print(json.dumps(out)); return
    out["place_ok"] = True
    attach_plate_to_gripper(node, scene, "pxrd_sample", buf=buf,
                            source_deck="pxrd_sample", seat_tab=True)
    time.sleep(0.4)
    # 2. retract out to the bay stage (level)
    out["retract_ok"], _ = goto(stage, bq, "retract to bay stage")
    if not out["retract_ok"]:
        out["verdict"] = "retract-fail"; print(json.dumps(out)); return
    # 3. carry stage -> home (level); seed home IK from the CURRENT (bay) family
    cur = get_current_robot_state(node, gs)
    out["carry_home_ok"], home_cfg = goto(home, tq, "carry stage->HOME", seed_state=cur)
    if not out["carry_home_ok"]:
        out["verdict"] = "carry-home-fail"
        out["max_tilt_deg"] = round(max(tilts), 2); print(json.dumps(out)); return
    # 4. from home, is the table pick reachable (collision-free)?
    cur = get_current_robot_state(node, gs)
    tpk = compute_ik(node, ik, "base_link", "gripper_grasp_link", *t_adv, tq, cur)
    out["table_pick_reachable"] = tpk is not None
    out["max_tilt_deg"] = round(max(tilts) if tilts else 0.0, 2)
    out["verdict"] = ("BACKWARD-OK" if out["table_pick_reachable"]
                      else "home-ok-table-unreachable")
    print(json.dumps(out))
    rclpy.shutdown()


if __name__ == "__main__":
    main()

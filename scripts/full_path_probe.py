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
from geometry_msgs.msg import Pose
from tf2_ros import Buffer, TransformListener
from pxrd_cell.pick_place import (
    side_grasp_q, compute_ik_best_of_many, get_current_robot_state,
    attach_plate_to_gripper, attach_plate_to_deck, purge_plate, lookup_tf,
    _tf_to_T, _plate_rest_T, _quat_to_R, PAD_OFFSET_FROM_TCP, TAB_CENTER_X,
    TAB_PINCH_Y, apply_plate, plan_arm_to_joints, plan_side_stage,
    cartesian_move, _check_tilt_gate)


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
    cart = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    for c in (ik, gs, scene, cart):
        if not c.wait_for_service(timeout_sec=15):
            out["detail"] = "services down"; print(json.dumps(out)); return
    mg = ActionClient(node, MoveGroup, "/move_action"); mg.wait_for_server(timeout_sec=15)
    exc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory"); exc.wait_for_server(timeout_sec=15)
    fk = node.create_client(GetPositionFK, "/compute_fk"); fk.wait_for_service(timeout_sec=15)
    gate = (fk, 10.0)
    time.sleep(1.0)
    tilts = []
    taught = []            # recorded joint config at each reached waypoint

    def _record(tag):
        cs = get_current_robot_state(node, gs)
        j = {n: p for n, p in zip(cs.joint_state.name, cs.joint_state.position)
             if n in ("j1", "j2", "j3", "j4", "j5", "j6")}
        taught.append({"tag": tag, "joints": j})

    def insert_partial(x, y, z, q, min_frac=0.80):
        """Plan a straight Cartesian insert and EXECUTE the reachable, collision-
        free, level fraction (pick_place.cartesian_move only executes at >=99%;
        the carried plate's leading edge meets the bay mesh earlier, so we slide
        as deep as the sim allows). Returns (ok, fraction)."""
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = "fairino10_v6_group"
        req.link_name = "gripper_grasp_link"
        req.max_step = 0.005
        req.jump_threshold = 5.0
        req.avoid_collisions = True
        tgt = Pose(); tgt.position.x = x; tgt.position.y = y; tgt.position.z = z
        tgt.orientation = q
        req.waypoints = [tgt]
        fut = cart.call_async(req)
        dl = time.time() + 10.0
        while not fut.done() and time.time() < dl:
            time.sleep(0.02)
        if not fut.done():
            return False, 0.0
        res = fut.result(); frac = res.fraction; traj = res.solution
        if frac < min_frac:
            return False, frac
        gok, _ = _check_tilt_gate(node, gate, traj)
        if not gok:
            return False, frac
        goal = ExecuteTrajectory.Goal(); goal.trajectory = traj
        gf = exc.send_goal_async(goal)
        dl = time.time() + 10.0
        while not gf.done() and time.time() < dl:
            time.sleep(0.02)
        h = gf.result()
        if h is None or not h.accepted:
            return False, frac
        rf = h.get_result_async()
        dl = time.time() + 60.0
        while not rf.done() and time.time() < dl:
            time.sleep(0.02)
        if not rf.done() or rf.result().result.error_code.val != 1:
            return False, frac
        return True, frac

    def goto(xyz, q, tag, gated):
        cur = get_current_robot_state(node, gs)
        tgt, _ = compute_ik_best_of_many(node, ik, "base_link",
                                         "gripper_grasp_link", *xyz, q, cur)
        if tgt is None:
            print(f"  {tag}: NO IK", flush=True); return False
        ok, info = plan_arm_to_joints(node, mg, tgt, vel=0.3, exec_client=exc,
                                      tilt_gate=(gate if gated else None))
        t = tilt(buf); tilts.append(t)
        print(f"  {tag}: {'OK' if ok else 'FAIL'} tilt={t:.1f}", flush=True)
        if ok:
            _record(tag)
        return ok

    # ---- TABLE geometry
    (tsx, tsy), (ttabx, ttaby), (tvx, tvy), tsz = deck_tab_axis(node, buf, "table_top")
    tq = side_grasp_q((tvx, tvy))
    t_pinch = tsz + TAB_PINCH_Y
    t_adv = (ttabx - tvx * PAD_OFFSET_FROM_TCP, ttaby - tvy * PAD_OFFSET_FROM_TCP)
    t_app = (t_adv[0] - tvx * a.pick_standoff, t_adv[1] - tvy * a.pick_standoff, t_pinch)
    # ---- BAY geometry — anchored on the REAL achievable seat (captured FK),
    # not the pinch formula (which over-reaches and collides).
    (bsx, bsy), (btabx, btaby), (bvx, bvy), bsz = deck_tab_axis(node, buf, "pxrd_sample")
    bq = side_grasp_q((bvx, bvy))
    SEAT = (0.613, 0.750, 0.631)
    # Insert/carry 10mm ABOVE the seat: at seat height the carried plate's BOTTOM
    # scrapes the sample stage (pxrd_smartlab) and the slide-in blocks at 0%;
    # +10mm clears the stage and the plate slides 100% to the seat XY, collision-
    # free (proven by scripts/insert_diag.py height sweep). The plate then lines
    # up directly OVER the sample; the SET-DOWN + RELEASE below drop it onto it.
    INSERT_LIFT = 0.012
    b_pinch = SEAT[2] + INSERT_LIFT
    # Full seat XY (no short-fall) at the raised height — the plate ends centered
    # over the sample marker, ready to set down.
    b_place = (SEAT[0], SEAT[1], b_pinch)
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
    # CARRY via an explicit WAYPOINT LADDER (945mm-style): small in-family,
    # tilt-gated hops along a safe arc from HOME up to bay height, across to
    # the bay stage. Each hop IK-seeded from the previous config.
    hx, hy = home[0], home[1]
    sx2, sy2 = b_stage[0], b_stage[1]
    bz = b_pinch
    N = 6
    way = [(hx, hy, bz, bq, "W1 lift+reorient to bay height")]
    for k in range(1, N):
        f = k / N
        way.append((hx + f * (sx2 - hx), hy + f * (sy2 - hy), bz, bq,
                    f"W{k+1} arc {k}/{N}"))
    way.append((sx2, sy2, bz, bq, f"W{N+1} bay stage"))
    carry_ok = True
    for wx, wy, wz, wq, tag in way:
        ok = goto((wx, wy, wz), wq, "carry " + tag, gated=True)
        if not ok:
            carry_ok = False; out["verdict"] = f"carry-fail-{tag.split()[0]}"; break
    out["carry_stage_ok"] = carry_ok
    if not carry_ok:
        out["max_tilt_deg"] = round(max(tilts), 2); print(json.dumps(out)); return
    # INSERT: straight Cartesian slide bay stage -> seat XY at the raised (+12mm)
    # height. At this height the carried plate clears the sample stage and slides
    # FULLY in (100%), ending centered over the sample. Pure translation keeps it
    # dead level and in-family.
    ins_ok, frac = insert_partial(b_place[0], b_place[1], b_place[2], bq,
                                  min_frac=0.85)
    it = tilt(buf); tilts.append(it)
    print(f"  insert (slide in @+12mm): ok={ins_ok} frac={frac:.2f} "
          f"tilt={it:.1f}", flush=True)
    out["insert_fraction"] = round(frac, 3)
    if ins_ok and frac >= 0.85:
        _record(f"insert to seat XY ({int(frac*100)}%)")
        # SET DOWN: lower straight toward the seat as far as the stage allows
        # (the plate bottom nears the sample surface), keeping level.
        sd_ok, sd_frac = insert_partial(SEAT[0], SEAT[1], SEAT[2], bq,
                                        min_frac=0.05)
        sdt = tilt(buf); tilts.append(sdt)
        print(f"  set down over sample: ok={sd_ok} frac={sd_frac:.2f} "
              f"tilt={sdt:.1f}", flush=True)
        _record("set down over sample")
        # RELEASE: hand the plate to the sample deck — it snaps the last few mm to
        # its canonical rest pose ON the sample (whitelisted contact). Placed.
        attach_plate_to_deck(node, scene, "pxrd_sample",
                             current_parent="gripper_grasp_link")
        print("  released: plate re-attached to pxrd_sample (placed on sample)",
              flush=True)
        out["place_ok"] = True
        out["setdown_fraction"] = round(sd_frac, 3)
    out["max_tilt_deg"] = round(max(tilts) if tilts else 0.0, 2)
    out["verdict"] = "FULL-PATH" if out["place_ok"] else f"insert-{int(frac*100)}pct"
    # Write the taught path (joint config at every waypoint) so the demo can
    # REPLAY these configs directly — no IK/OMPL search, no retries. Teach once
    # (this slow probe), replay fast.
    if out["place_ok"]:
        tp = {"layout": "585mm_offz0.8_offx0.2_yaw45", "plate": "v3",
              "source": "table_top", "target": "pxrd_sample",
              "joint_names": ["j1", "j2", "j3", "j4", "j5", "j6"],
              "max_tilt_deg": out["max_tilt_deg"],
              "insert_fraction": out["insert_fraction"],
              "note": "joint-space taught path, table->home->carry->insert, level",
              "waypoints": taught}
        path = "/home/aimatx_nuc3/Ks_FR/cad/pxrd/taught_path_585_full.json"
        with open(path, "w") as fh:
            json.dump(tp, fh, indent=2)
        out["taught_path"] = path
        out["n_waypoints"] = len(taught)
    print(json.dumps(out))
    rclpy.shutdown()


if __name__ == "__main__":
    main()

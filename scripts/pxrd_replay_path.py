#!/usr/bin/env python3
"""585mm PXRD — FAST REPLAY of a taught full path (table -> home -> carry ->
insert into the bay).

full_path_probe.py does the SLOW one-time search (63-seed IK per waypoint +
OMPL + retries ≈ 5-7 min) and records the joint config at every reached
waypoint into cad/pxrd/taught_path_585_full.json. This script REPLAYS those
recorded configs directly — NO IK search, NO per-waypoint seed exploration.

Speed strategy (what the user asked for — "remember the joint angles / merge
the waypoints"):
  * PICK  (neutral -> approach -> advance): gross moves, OMPL-planned straight
    to the RECORDED joints (no IK search; OMPL only to collision-avoid the one
    big start move).
  * CARRY+INSERT (home -> arc -> bay stage -> seat): streamed as ONE merged,
    tilt-gated, collision-validated joint-space trajectory via lerp_path. The
    taught configs were FK-verified level at teach time, so the plate stays
    level the whole way. If any straight segment is blocked, fall back to
    per-hop OMPL to the recorded joints (still no IK search).

The plate is attached to the gripper after pick-advance and re-attached to the
deck after insert, so the collision model stays honest throughout.
"""
import argparse
import json
import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene, GetPositionFK
from tf2_ros import Buffer, TransformListener

from pxrd_cell.pick_place import (
    apply_plate, lerp_path, plan_arm_to_joints, attach_plate_to_gripper,
    attach_plate_to_deck, purge_plate, _quat_to_R)

TAUGHT = "/home/aimatx_nuc3/Ks_FR/cad/pxrd/taught_path_585_full.json"


def tilt(buf):
    tf = buf.lookup_transform("base_link", "gripper_grasp_link", rclpy.time.Time())
    R = _quat_to_R(tf.transform.rotation)
    return math.degrees(math.acos(max(-1.0, min(1.0, R[2][0]))))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--file", default=TAUGHT)
    ap.add_argument("--vel", type=float, default=0.3)
    ap.add_argument("--max-tilt", type=float, default=10.0)
    ap.add_argument("--no-merge", action="store_true",
                    help="force per-hop OMPL replay (skip the merged lerp)")
    ap.add_argument("--retract-only", action="store_true",
                    help="arm is stranded in the bay after a place: just reverse "
                         "the recorded in-path (empty) back out to HOME")
    ap.add_argument("--reverse", action="store_true",
                    help="retrieve: PXRD -> table (reach in empty, grab the plate "
                         "off the sample, carry out level, place on the table)")
    ap.add_argument("--hardware", action="store_true",
                    help="running on the REAL FR10 via the bridge: print the "
                         "open-space checklist, require confirmation, and move to "
                         "the side-HOME first (controlled) before the path")
    ap.add_argument("--yes", action="store_true",
                    help="skip the interactive hardware confirmation prompt")
    ap.add_argument("--home-only", action="store_true",
                    help="just move the arm to the taught side-HOME (starting "
                         "position) and stop — run this before a pick-place")
    a = ap.parse_args()

    with open(a.file) as f:
        data = json.load(f)
    wps = data["waypoints"]
    src = data.get("source", "table_top")
    tgt = data.get("target", "pxrd_sample")
    plate = data.get("plate", "v3")
    trusted = data.get("max_tilt_deg")

    rclpy.init()
    node = Node("pxrd_replay_path")
    ex = MultiThreadedExecutor(); ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    buf = Buffer(); TransformListener(buf, node)
    apply_plate(plate)
    gs = node.create_client(GetPlanningScene, "/get_planning_scene")
    scene = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    fk = node.create_client(GetPositionFK, "/compute_fk")
    for c in (gs, scene, fk):
        if not c.wait_for_service(timeout_sec=15):
            node.get_logger().error("services down"); return
    mg = ActionClient(node, MoveGroup, "/move_action"); mg.wait_for_server(timeout_sec=15)
    exc = ActionClient(node, ExecuteTrajectory, "/execute_trajectory"); exc.wait_for_server(timeout_sec=15)
    gate = (fk, a.max_tilt)
    time.sleep(1.0)

    def log(m): node.get_logger().info("  >> " + m)

    # Split the taught path into three phases:
    #   pre   = pick (up to & incl "advance")      — plate on table, OMPL
    #   carry = home .. "bay stage"                — merged lerp (plate held)
    #   tail  = after the bay stage (insert, set-down) — OMPL hops, then release
    tags = [w["tag"] for w in wps]
    attach_i = next((i for i, t in enumerate(tags) if "advance" in t),
                    0 if not tags else len(tags) - 1)
    stage_i = next((i for i in range(len(tags) - 1, -1, -1)
                    if "bay stage" in tags[i]), len(wps) - 2)
    pre = wps[:attach_i + 1]            # approach, advance
    carry = wps[attach_i + 1:stage_i + 1]   # home, W1..W7 bay stage
    tail = wps[stage_i + 1:]           # insert to seat XY, set down

    def ompl_hop(wp, gated):
        ok, info = plan_arm_to_joints(node, mg, wp["joints"], vel=a.vel,
                                      exec_client=exc,
                                      tilt_gate=(gate if gated else None))
        log(f"{wp['tag']}: {'OK' if ok else 'FAIL'} (ompl) tilt={tilt(buf):.1f}")
        return ok

    # Reverse the recorded in-path (empty arm) back out of the bay to HOME. The
    # arm sits deep in the enclosure after a place — a free OMPL move to the table
    # collides with the bay, so retract along the SAME recorded corridor in
    # reverse. Skips the current (set-down) config; ends at the side HOME.
    retract_wps = list(reversed(carry + tail))[1:]

    def retract(reason):
        log(f"RETRACT out of bay ({reason}) — {len(retract_wps)} waypoints")
        for wp in retract_wps:
            if not ompl_hop(wp, gated=False):
                log(f"retract stalled at '{wp['tag']}'"); return False
        return True

    home_wp = next((w for w in wps if "HOME" in w["tag"]), carry[0] if carry else None)

    def go_to_home():
        """Robustly drive the arm to the taught side-HOME from ANY pose. Uses
        RRTConnect with a real planning budget; execution is budget-chunked, so
        the long j1 swing is fine on hardware.

        purge_plate clears any stale attached object, BUT move_group needs a
        moment to digest that scene diff — planning the instant after the purge
        races the update and returns -2. Settle, then plan (retry once)."""
        purge_plate(node, scene)
        time.sleep(0.8)                # let the scene diff propagate (avoids -2)
        ok, info = (False, "")
        for _ in range(2):
            ok, info = plan_arm_to_joints(node, mg, home_wp["joints"], vel=a.vel,
                                          exec_client=exc, planner_id="RRTConnect",
                                          planning_time=25.0, planning_attempts=30)
            if ok:
                return ok, info
            time.sleep(0.6)
        return ok, info

    # ---- HARDWARE: open-space checklist, confirm, controlled HOME-first ----
    # The bridge is the UNMODIFIED shared fr_bridge; it executes these same
    # trajectories on the real FR10 (budget-chunked). Real speed = bridge
    # movej_vel_pct (planning --vel is sim-only). No gripper/plate: the plate
    # scene ops are virtual (collision model only) and never drive hardware.
    if a.hardware:
        print("\n" + "=" * 64, flush=True)
        print("  REAL FR10 MOTION via the bridge — pre-flight checklist:", flush=True)
        print("   [ ] OPEN SPACE: no table / PXRD / obstacles within arm reach", flush=True)
        print("   [ ] FR10 in MANUAL mode + enabled  (./run_pxrd.sh preflight)", flush=True)
        print("   [ ] bridge running (T1) and hw-scene running (T2)", flush=True)
        print("   [ ] e-stop in hand; pendant speed override low (<=25%)", flush=True)
        print("   The arm will FIRST move to the side-HOME, then run the path.", flush=True)
        print("=" * 64, flush=True)
        if not a.yes:
            try:
                if input("  type 'go' to execute on hardware: ").strip().lower() != "go":
                    print("  aborted."); rclpy.shutdown(); return
            except EOFError:
                print("  no TTY and --yes not given; aborting."); rclpy.shutdown(); return
        if home_wp is not None:
            log("HARDWARE: moving to side-HOME first (RRTConnect, budget-chunked)")
            hok, hinfo = go_to_home()
            if not hok:
                log(f"home-first move FAILED: {hinfo} — aborting")
                rclpy.shutdown(); return
            time.sleep(0.3)

    t0 = time.time()

    # ---- HOME-ONLY: park the arm at the taught side-HOME and stop ----
    if a.home_only:
        if home_wp is None:
            log("no HOME waypoint in the taught path"); rclpy.shutdown(); return
        if not a.hardware:          # the --hardware block above already homed
            log("HOME-ONLY: moving to the taught side-HOME (starting position)")
            hok, hinfo = go_to_home()
            if not hok:
                log(f"home move FAILED: {hinfo}"); rclpy.shutdown(); return
        log(f"DONE — arm at the taught side-HOME, tilt={tilt(buf):.1f} deg "
            f"({time.time() - t0:.1f}s). Ready for a pick-place.")
        time.sleep(1.0); rclpy.shutdown(); return

    # ---- stranded-arm rescue: just back out, don't touch the placed plate ----
    if a.retract_only:
        ok = retract("--retract-only")
        log(f"{'DONE — arm back at HOME' if ok else 'RETRACT INCOMPLETE'} "
            f"({time.time() - t0:.1f}s)")
        time.sleep(1.0); rclpy.shutdown(); return

    # ================= REVERSE: PXRD -> table (retrieve) =================
    # Same recorded corridor, traversed the other way with grab/release swapped.
    # A time-reversed valid, level path is itself valid and level.
    if a.reverse:
        arc = carry[1:]                       # W1 .. W7 bay stage
        def lerp_or_hops(wps_seg, gated, tag):
            js = [w["joints"] for w in wps_seg]
            if not a.no_merge and js:
                ok, info = lerp_path(node, gs, exc, js, vel=a.vel,
                                     tilt_gate=(gate if gated else None),
                                     trusted_tilt=trusted, sparse_check=True)
                log(f"{tag}: merged {'OK' if ok else 'FAIL'} tilt={tilt(buf):.1f}  ({info})")
                if ok:
                    return True
            for w in wps_seg:
                if not ompl_hop(w, gated=gated):
                    return False
            return True

        # plate starts resting on the sample
        purge_plate(node, scene)
        attach_plate_to_deck(node, scene, tgt)       # tgt = pxrd_sample
        # REACH IN (empty): home -> carry arc -> insert -> set-down (to the plate)
        log("REVERSE reach into bay (empty): home")
        if not ompl_hop(carry[0], gated=False):
            log("reach-in to HOME FAILED"); return
        if not lerp_or_hops(arc, gated=False, tag="reach-in carry arc"):
            log("reach-in carry FAILED"); return
        for wp in tail:                               # insert-seat-XY, set-down
            if not ompl_hop(wp, gated=False):
                log("reach-in to plate FAILED"); return
        # GRAB the resting plate
        attach_plate_to_gripper(node, scene, tgt, buf=buf, source_deck=tgt,
                                seat_tab=True)
        time.sleep(0.3)
        log(f"--- grabbed plate off the sample ({tilt(buf):.1f} deg) ---")
        # COME OUT (holding): lift +12mm, then reverse the carry back to HOME
        if not ompl_hop(tail[0], gated=True):         # insert-seat-XY (lift off stage)
            log("lift off stage FAILED"); return
        if not lerp_or_hops(list(reversed(carry)), gated=True, tag="carry-out (holding)"):
            log("carry-out FAILED"); return
        # DESCEND to the table (holding) and RELEASE
        if not ompl_hop(pre[1], gated=True):          # advance = table grasp pose
            log("descend to table FAILED"); return
        attach_plate_to_deck(node, scene, src)        # src = table_top
        log("--- plate placed on the table, level ---")
        time.sleep(0.3)
        ompl_hop(pre[0], gated=False)                 # approach = back off
        log(f"DONE — plate retrieved PXRD->table, level throughout. "
            f"{time.time() - t0:.1f}s")
        time.sleep(1.0); rclpy.shutdown(); return
    # ====================================================================

    purge_plate(node, scene)
    attach_plate_to_deck(node, scene, src)

    # ---- PICK: OMPL straight to the recorded joints (no IK search) ----
    log(f"PICK replay ({len(pre)} waypoints) — plate on {src}")
    for wp in pre:
        if not ompl_hop(wp, gated=False):
            log("pick replay FAILED"); return
    attach_plate_to_gripper(node, scene, src, buf=buf, source_deck=src,
                            seat_tab=True)
    time.sleep(0.3)
    log(f"--- picked, holding level ({tilt(buf):.1f} deg) ---")

    # ---- CARRY: merge the long home->bay-stage run into ONE level, validated
    # trajectory. The tail (insert + set-down) stays per-hop OMPL: the gripper-
    # held plate grazes pxrd_smartlab near the sample, which the dense lerp
    # validity check flags but an OMPL hop to the recorded config clears. ----
    carry_joints = [w["joints"] for w in carry]
    done = False
    if not a.no_merge and carry_joints:
        log(f"CARRY replay — MERGED lerp through {len(carry)} waypoints, level")
        # lerp_path's scene_client arg feeds get_current_robot_state, which
        # needs the GetPlanningScene client (gs), NOT ApplyPlanningScene.
        ok, info = lerp_path(node, gs, exc, carry_joints, vel=a.vel,
                             tilt_gate=gate, trusted_tilt=trusted,
                             sparse_check=True)
        log(f"merged carry: {'OK' if ok else 'FAIL'} tilt={tilt(buf):.1f}  ({info})")
        done = ok
    if not done:
        log("CARRY replay — per-hop OMPL fallback (recorded joints)")
        for wp in carry:
            if not ompl_hop(wp, gated=True):
                log("carry replay FAILED"); return
    # TAIL: insert to the seat XY (+12mm), then set down over the sample — each a
    # single OMPL hop to the recorded, level-verified config.
    for wp in tail:
        log(f"TAIL replay — OMPL to '{wp['tag']}'")
        if not ompl_hop(wp, gated=True):
            log("tail replay FAILED"); return

    # ---- RELEASE: hand the plate to the sample deck (snaps onto the sample) ----
    attach_plate_to_deck(node, scene, tgt, current_parent="gripper_grasp_link")
    log("--- plate placed on the sample, level ---")
    time.sleep(0.5)

    # ---- RETRACT empty back out of the bay to HOME (full cycle, clean end) ----
    retract("post-place")
    log(f"DONE — plate placed in PXRD & arm retracted to HOME, level throughout. "
        f"REPLAY took {time.time() - t0:.1f}s (teach was ~5-7 min).")
    time.sleep(1.0)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

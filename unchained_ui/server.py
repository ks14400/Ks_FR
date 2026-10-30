#!/usr/bin/env python3
"""
Unchained cell test UI — thin web wrapper over the PROVEN CLI workflow.

Serves one page (static/index.html) plus a JSON API. Every action shells out
to the exact commands used by hand (run_unchained.sh / ros2 run ...), so the
CLI stays the single source of truth. UNCHAINED ONLY (domain 42): all kills
are domain-scoped and never touch the pxrd cell (43) or the bridge unless
explicitly asked.

Run:  python3 unchained_ui/server.py            (port 8420)
      or  ./run_unchained.sh ui
Deps: stdlib + PyYAML only.
"""
import json
import os
import re
import signal
import subprocess
import threading
import time
from datetime import datetime
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import yaml

ROOT = os.path.dirname(os.path.abspath(__file__))          # unchained_ui/
REPO = os.path.dirname(ROOT)                               # Ks_FR/
RUNS = os.path.join(ROOT, "runs")
PROC_LOGS = os.path.join(ROOT, "proc_logs")
CONFIG = os.path.join(ROOT, "ui_config.yaml")
DOMAIN = "42"
PORT = 8420

os.makedirs(RUNS, exist_ok=True)
os.makedirs(PROC_LOGS, exist_ok=True)

ENV_PREFIX = ("source /opt/ros/humble/setup.bash >/dev/null 2>&1; "
              f"source {REPO}/ros2_ws/install/setup.bash >/dev/null 2>&1; "
              f"export ROS_DOMAIN_ID={DOMAIN}; ")

DECKS = ["stand_pos1", "stand_pos2", "stand_pos3", "stand_pos4",
         "stand_pos5", "stand_pos6", "stand_pos7", "stand_pos8",
         "deck_vortex_pos1", "deck_vortex_pos2", "deck_vortex_pos3",
         "deck_9_10_pos1", "deck_9_10_pos2", "deck_9_10_pos3",
         "deck_vacuum_filtration", "table_top"]
PLATE_TYPES = ["8ml", "20ml", "sd", "96"]

DEFAULT_CONFIG = {
    # per-deck Z trims (mm) — the calibration ledger. Stand pockets: NEVER
    # trim (table correction is baked into table_z).
    "deck_z_trim_mm": {"deck_vortex_pos1": 2.0},
    "grip_mm_long": 122.0,     # long-side clamp (stand picks), REAL mm on hw
    "grip_mm_short": 0.0,      # 0 = AUTO (83.5 real)
    "pre_release_lift_m": 0.005,
    "pick_lift_m": 0.004,
    "bridge_movej_vel_pct": 10,
    "bridge_movej_acc_pct": 15,
    "bridge_gripper_force_pct": 40,
}


def load_config():
    cfg = dict(DEFAULT_CONFIG)
    if os.path.exists(CONFIG):
        with open(CONFIG) as f:
            cfg.update(yaml.safe_load(f) or {})
    return cfg


def save_config(cfg):
    with open(CONFIG, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)


def sh(cmd, timeout=20):
    """Run a short command in the sourced env; return (rc, output)."""
    try:
        r = subprocess.run(["bash", "-c", ENV_PREFIX + cmd],
                           capture_output=True, text=True, timeout=timeout)
        return r.returncode, (r.stdout + r.stderr)
    except subprocess.TimeoutExpired:
        return -1, "TIMEOUT"


def proc_domain(pid):
    try:
        with open(f"/proc/{pid}/environ", "rb") as f:
            env = f.read().decode(errors="ignore")
        m = re.search(r"ROS_DOMAIN_ID=(\d+)", env)
        return m.group(1) if m else None
    except OSError:
        return None


def pids_matching(pattern, domain=None):
    r = subprocess.run(["pgrep", "-f", pattern], capture_output=True, text=True)
    out = []
    for pid in r.stdout.split():
        if domain is None or proc_domain(pid) == domain:
            out.append(pid)
    return out


SCENE_PATTERN = ("unchained_full.launch|move_group --ros|rviz2|"
                 "robot_state_pub|mimic_repeater|ros2_control_node|"
                 "static_transform|joint_state|spawner")


def kill_scene_42():
    for pid in pids_matching(SCENE_PATTERN, DOMAIN):
        try:
            os.kill(int(pid), signal.SIGKILL)
        except OSError:
            pass
    time.sleep(2)


def desktop_env():
    """Borrow DISPLAY/XAUTHORITY from the active desktop session so rviz
    windows open on the NUC's screen even though the server runs headless."""
    env = dict(os.environ)
    try:
        pid = subprocess.run(["pgrep", "-u", str(os.getuid()), "Xwayland"],
                             capture_output=True, text=True).stdout.split()[0]
        with open(f"/proc/{pid}/environ", "rb") as f:
            for item in f.read().decode(errors="ignore").split("\0"):
                if item.split("=")[0] in ("DISPLAY", "XAUTHORITY",
                                          "WAYLAND_DISPLAY", "XDG_RUNTIME_DIR"):
                    k, _, v = item.partition("=")
                    env[k] = v
    except (IndexError, OSError):
        env.setdefault("DISPLAY", ":0")
    return env


def spawn_detached(cmd, logname):
    """Launch a long-lived process (bridge/scene/sim) detached, log to file."""
    log = open(os.path.join(PROC_LOGS, logname), "a")
    log.write(f"\n===== {datetime.now().isoformat()} :: {cmd}\n")
    log.flush()
    subprocess.Popen(["bash", "-c", ENV_PREFIX + "exec " + cmd],
                     stdout=log, stderr=log, stdin=subprocess.DEVNULL,
                     start_new_session=True, env=desktop_env())


# ───────────────────────── status ─────────────────────────

def status():
    rsp = pids_matching("robot_state_pub", DOMAIN)
    mg = pids_matching("move_group --ros", DOMAIN)
    bridge = pids_matching("sdk_executor", DOMAIN)
    pxrd = pids_matching("move_group --ros|robot_state_pub", "43")
    ping = subprocess.run(["ping", "-c", "1", "-W", "1", "192.168.58.2"],
                          capture_output=True).returncode == 0
    return {
        "bridge": bool(bridge),
        "scene": bool(mg),
        "rsp_count": len(rsp),
        "duplicate_scene": len(rsp) > 1 or len(mg) > 1,
        "robot_ping": ping,
        "pxrd_running": bool(pxrd),
        "active_run": ACTIVE_RUN["id"] if ACTIVE_RUN["proc"] else None,
    }


def gripper_state():
    rc, out = sh(
        "python3 - <<'EOF'\n"
        "import rclpy, time\n"
        "from rclpy.node import Node\n"
        "from sensor_msgs.msg import JointState\n"
        "rclpy.init(); n=Node('ui_grip'); v={}\n"
        "n.create_subscription(JointState,'/joint_states',"
        "lambda m:[v.__setitem__(a,b) for a,b in zip(m.name,m.position)],20)\n"
        "t=time.time()\n"
        "while time.time()-t<2.5: rclpy.spin_once(n,timeout_sec=0.1)\n"
        "print('F1=%.3f' % v.get('gripper_finger1_joint', 99))\n"
        "rclpy.shutdown()\nEOF", timeout=15)
    m = re.search(r"F1=(-?\d+\.\d+)", out)
    if not m or float(m.group(1)) > 10:
        return {"ok": False}
    j = float(m.group(1))
    return {"ok": True, "joint": j,
            "label": "open" if j > -0.05 else ("closed" if j < -0.55 else f"{j:+.2f}")}


CALIB = os.path.join(ROOT, "deck_calib.yaml")


def load_calib():
    if os.path.exists(CALIB):
        with open(CALIB) as f:
            return yaml.safe_load(f) or {}
    return {}


def save_calib(c):
    with open(CALIB, "w") as f:
        yaml.safe_dump(c, f, sort_keys=False)


def teach_pose(deck, plate_type):
    """Current TCP vs deck marker (base_link) + delta vs the AUTO grasp
    height for the selected plate type. All mm."""
    rc, out = sh(
        "python3 - <<'EOF'\n"
        "import sys, threading, time\n"
        f"sys.path.insert(0, '{REPO}/ros2_ws/src/unchained_cell')\n"
        "import rclpy\n"
        "from rclpy.node import Node\n"
        "from rclpy.executors import MultiThreadedExecutor\n"
        "from tf2_ros import Buffer, TransformListener\n"
        "from unchained_cell import pick_place as pp\n"
        f"pp.set_plate_type('{plate_type}')\n"
        "rclpy.init(); n=Node('ui_teach')\n"
        "ex=MultiThreadedExecutor(); ex.add_node(n)\n"
        "threading.Thread(target=ex.spin, daemon=True).start()\n"
        "buf=Buffer(); TransformListener(buf, n); time.sleep(2.5)\n"
        "g=pp.lookup_tf(n, buf, 'base_link', 'gripper_grasp_link').transform.translation\n"
        f"m=pp.lookup_tf(n, buf, 'base_link', '{deck}').transform.translation\n"
        "go=pp.default_grasp_offset()\n"
        "print('TP %.4f %.4f %.4f %.4f %.4f %.4f %.4f' % (g.x,g.y,g.z,m.x,m.y,m.z,go))\n"
        "rclpy.shutdown()\nEOF", timeout=25)
    m = re.search(r"TP ([-\d. ]+)", out)
    if not m:
        return {"ok": False, "err": out[-300:]}
    gx, gy, gz, mx, my, mz, go = [float(v) for v in m.group(1).split()]
    return {"ok": True,
            "tcp_mm": [round(gx*1000,1), round(gy*1000,1), round(gz*1000,1)],
            "marker_mm": [round(mx*1000,1), round(my*1000,1), round(mz*1000,1)],
            "dx_mm": round((gx-mx)*1000, 1),
            "dy_mm": round((gy-my)*1000, 1),
            "dz_grasp_mm": round((gz-(mz+go))*1000, 1),
            "grasp_offset_mm": round(go*1000, 1)}


# ───────────────────────── run executor ─────────────────────────

ACTIVE_RUN = {"proc": None, "id": None}
RUN_LOCK = threading.Lock()

PHASE_RE = re.compile(r"\[(OK|FAIL)\s*\]\s+(.+?)\s{2,}(\S+)\s+([\d.]+)s\s+(.*)")


def parse_summary(output):
    phases, notes = [], {}
    for line in output.splitlines():
        m = PHASE_RE.search(line)
        if m:
            phases.append({"result": m.group(1), "phase": m.group(2).strip(),
                           "method": m.group(3), "secs": float(m.group(4)),
                           "info": m.group(5).strip()})
        for key, pat in (("rigid_hold", r"rigid hold: (.+)"),
                         ("grip_auto", r"grip width AUTO: (.+)"),
                         ("place_height", r"place height: (.+)"),
                         ("grip_cmd", r"\[gripper\] (.+)"),
                         ("hover", r"\[hover\] (.+)")):
            m2 = re.search(pat, line)
            if m2:
                notes[key] = m2.group(1).strip()
    failed = [p for p in phases if p["result"] == "FAIL"]
    done = bool(re.search(r"\bDONE\b", output))
    return {"phases": phases, "notes": notes, "pass": done and not failed,
            "failed_phase": failed[0] if failed else None}


def start_run(body, cfg):
    action = body.get("action", "pick_place")   # pick_place | pick_only | place_held
    src, tgt = body["source"], body["target"]
    plate = body["plate_type"]
    hardware = bool(body.get("hardware", False))
    dry = bool(body.get("no_gripper", False))

    cmd = (f"ros2 run unchained_cell pick_place --source {src} --target {tgt} "
           f"--plate-type {plate}")
    if hardware:
        cmd += " --hardware"
    if dry:
        cmd += " --no-gripper"
    if action == "pick_only":
        cmd += " --pick-only"
    if action == "place_held":
        cmd += " --place-held"
    if action == "hover_only":
        cmd += " --hover-only"

    trims = cfg.get("deck_z_trim_mm", {}) or {}
    flags = {}
    if action != "place_held" and trims.get(src):
        cmd += f" --source-z-trim {trims[src]}"; flags["source_z_trim"] = trims[src]
    if trims.get(tgt):
        cmd += f" --target-z-trim {trims[tgt]}"; flags["target_z_trim"] = trims[tgt]
    # grip override: long side picks come from stand pockets, short from others
    grip = None
    if action != "place_held" and not dry:
        grip = (cfg.get("grip_mm_long") if src.startswith("stand_pos")
                else cfg.get("grip_mm_short"))
        if grip:
            cmd += f" --grip-mm {grip}"; flags["grip_mm"] = grip
    if cfg.get("pre_release_lift_m") not in (None, "", 0.015):
        cmd += f" --pre-release-lift {cfg['pre_release_lift_m']}"
        flags["pre_release_lift"] = cfg["pre_release_lift_m"]
    if cfg.get("pick_lift_m") not in (None, "", 0.004):
        cmd += f" --pick-lift {cfg['pick_lift_m']}"
        flags["pick_lift"] = cfg["pick_lift_m"]
    if body.get("extra_flags"):
        cmd += " " + body["extra_flags"]

    run_id = (datetime.now().strftime("%Y-%m-%d_%H%M%S") +
              f"_{action}_{src}_to_{tgt}_{plate}" +
              ("_hw" if hardware else "_sim"))
    rdir = os.path.join(RUNS, run_id)
    os.makedirs(rdir, exist_ok=True)
    with open(os.path.join(rdir, "params.json"), "w") as f:
        json.dump({"id": run_id, "cmd": cmd, "action": action, "source": src,
                   "target": tgt, "plate_type": plate, "hardware": hardware,
                   "no_gripper": dry, "flags": flags, "config": cfg,
                   "started": datetime.now().isoformat()}, f, indent=2)

    def worker():
        out_path = os.path.join(rdir, "output.log")
        with open(out_path, "w") as out:
            p = subprocess.Popen(["bash", "-c", ENV_PREFIX + cmd],
                                 stdout=out, stderr=subprocess.STDOUT,
                                 stdin=subprocess.DEVNULL,
                                 start_new_session=True)
            ACTIVE_RUN["proc"], ACTIVE_RUN["id"] = p, run_id
            p.wait()
        with open(out_path) as f:
            summary = parse_summary(f.read())
        summary["rc"] = p.returncode
        summary["finished"] = datetime.now().isoformat()
        with open(os.path.join(rdir, "summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        ACTIVE_RUN["proc"], ACTIVE_RUN["id"] = None, None

    with RUN_LOCK:
        if ACTIVE_RUN["proc"]:
            return {"error": "a run is already active", "id": ACTIVE_RUN["id"]}
        threading.Thread(target=worker, daemon=True).start()
    return {"started": run_id, "cmd": cmd}


def list_runs(limit=40):
    out = []
    for name in sorted(os.listdir(RUNS), reverse=True)[:limit]:
        d = os.path.join(RUNS, name)
        item = {"id": name, "running": name == ACTIVE_RUN["id"]}
        for fn in ("params.json", "summary.json"):
            fp = os.path.join(d, fn)
            if os.path.exists(fp):
                with open(fp) as f:
                    item[fn.split(".")[0]] = json.load(f)
        out.append(item)
    return out


# ───────────────────────── HTTP ─────────────────────────

class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _json(self, obj, code=200):
        data = json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            with open(os.path.join(ROOT, "static", "index.html"), "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        elif self.path == "/api/status":
            self._json(status())
        elif self.path == "/api/gripper_state":
            self._json(gripper_state())
        elif self.path == "/api/config":
            self._json({"config": load_config(), "decks": DECKS,
                        "plate_types": PLATE_TYPES})
        elif self.path == "/api/runs":
            self._json(list_runs())
        elif self.path.startswith("/api/run_log/"):
            rid = os.path.basename(self.path[len("/api/run_log/"):])
            fp = os.path.join(RUNS, rid, "output.log")
            txt = open(fp).read()[-40000:] if os.path.exists(fp) else "(no log)"
            self._json({"log": txt})
        else:
            self._json({"error": "not found"}, 404)

    def do_POST(self):
        n = int(self.headers.get("Content-Length") or 0)
        body = json.loads(self.rfile.read(n) or b"{}")
        p = self.path
        cfg = load_config()
        if p == "/api/launch/bridge":
            spawn_detached(
                f"{REPO}/run_unchained.sh bridge "
                f"movej_vel_pct:={cfg['bridge_movej_vel_pct']} "
                f"movej_acc_pct:={cfg['bridge_movej_acc_pct']} "
                f"gripper_force_pct:={cfg['bridge_gripper_force_pct']}",
                "bridge.log")
            self._json({"ok": True})
        elif p == "/api/launch/scene":
            kill_scene_42()
            spawn_detached(f"{REPO}/run_unchained.sh hw-scene", "scene.log")
            self._json({"ok": True})
        elif p == "/api/launch/sim":
            kill_scene_42()
            spawn_detached(f"{REPO}/run_unchained.sh sim", "sim.log")
            self._json({"ok": True})
        elif p == "/api/kill/scene":
            kill_scene_42()
            self._json({"ok": True})
        elif p == "/api/kill/bridge":
            for pid in pids_matching("sdk_executor", DOMAIN):
                os.kill(int(pid), signal.SIGKILL)
            self._json({"ok": True})
        elif p == "/api/stop":
            rc, out = sh("ros2 run unchained_cell stop", timeout=20)
            if ACTIVE_RUN["proc"]:
                try:
                    os.killpg(ACTIVE_RUN["proc"].pid, signal.SIGINT)
                except OSError:
                    pass
            self._json({"ok": True, "out": out[-500:]})
        elif p == "/api/run":
            self._json(start_run(body, cfg))
        elif p == "/api/gripper_free":
            # TEACH-ONLY free jaw control: no MoveIt planning/collision check,
            # straight FollowJointTrajectory to the bridge gripper server.
            # Safe by construction: the real AG-145 is force-limited and
            # stalls on contact; the model is known-wrong during teaching.
            mm = float(body.get("mm", 148))
            mm = max(0.0, min(148.0, mm))
            rc, out = sh(
                "python3 - <<'EOF'\n"
                "import sys, time\n"
                f"sys.path.insert(0, '{REPO}/ros2_ws/src/unchained_cell')\n"
                "import rclpy\n"
                "from rclpy.node import Node\n"
                "from rclpy.action import ActionClient\n"
                "from control_msgs.action import FollowJointTrajectory\n"
                "from trajectory_msgs.msg import JointTrajectoryPoint\n"
                "from builtin_interfaces.msg import Duration\n"
                "from unchained_cell.pick_place import hw_grip_mm_to_joint\n"
                f"target = hw_grip_mm_to_joint({mm})\n"
                "rclpy.init(); n=Node('ui_free_jaw')\n"
                "ac=ActionClient(n, FollowJointTrajectory, '/gripper_controller/follow_joint_trajectory')\n"
                "assert ac.wait_for_server(timeout_sec=8), 'bridge gripper server missing'\n"
                "g=FollowJointTrajectory.Goal()\n"
                "g.trajectory.joint_names=['gripper_finger1_joint']\n"
                "pt=JointTrajectoryPoint(); pt.positions=[target]; pt.time_from_start=Duration(sec=2)\n"
                "g.trajectory.points=[pt]\n"
                "f=ac.send_goal_async(g); rclpy.spin_until_future_complete(n,f,timeout_sec=10)\n"
                "gh=f.result(); assert gh and gh.accepted, 'goal rejected'\n"
                "rf=gh.get_result_async(); rclpy.spin_until_future_complete(n,rf,timeout_sec=30)\n"
                "print('FREEJAW_OK target_joint=%.3f' % target)\n"
                "rclpy.shutdown()\nEOF", timeout=50)
            self._json({"ok": "FREEJAW_OK" in out,
                        "out": out.splitlines()[-1] if out.strip() else "no output"})
        elif p == "/api/gripper":
            act = body.get("act")
            if act == "mm":
                cmd = (f"ros2 run unchained_cell go_home --hardware "
                       f"--gripper-only --gripper-mm {body['mm']}")
            else:
                cmd = (f"ros2 run unchained_cell go_home --hardware "
                       f"--gripper-only --gripper {act}")
            rc, out = sh(cmd, timeout=60)
            self._json({"ok": rc == 0, "out": out[-800:]})
        elif p == "/api/home":
            hw = " --hardware" if body.get("hardware") else ""
            rc, out = sh(f"ros2 run unchained_cell go_home{hw}", timeout=180)
            self._json({"ok": rc == 0, "out": out[-800:]})
        elif p == "/api/purge":
            rc, out = sh("ros2 run unchained_cell purge_scene", timeout=30)
            self._json({"ok": rc == 0, "out": out[-500:]})
        elif p == "/api/config":
            cfg.update(body.get("config", {}))
            save_config(cfg)
            self._json({"ok": True, "config": cfg})
        elif p == "/api/teach/goto":
            # SAME pipeline as a real pick (home -> IK-seeded approach ->
            # jaws open), stopping at the hover. Trims + taught calib apply.
            self._json(start_run({"action": "hover_only",
                                  "source": body["deck"],
                                  "target": body["deck"],
                                  "plate_type": body.get("plate_type", "20ml"),
                                  "hardware": True}, cfg))
        elif p == "/api/teach/drag":
            on = "true" if body.get("on") else "false"
            rc, out = sh("ros2 service call /sdk_executor/drag_teach "
                         f"std_srvs/srv/SetBool \"data: {on}\"", timeout=25)
            self._json({"ok": "success=True" in out, "out": out[-400:]})
        elif p == "/api/jog":
            d = body.get("d", {})
            cmd = "ros2 run unchained_cell jog_tcp " + " ".join(
                f"--{k} {v}" for k, v in d.items() if v)
            rc, out = sh(cmd, timeout=90)
            tail = [l for l in out.splitlines() if "JOG" in l or "REFUSED" in l]
            self._json({"ok": rc == 0, "out": (tail[-1] if tail else out[-200:])})
        elif p == "/api/teach/nest":
            # SAME scoped allowance production picks use at instrument decks
            # (4 finger links <-> unchained_junior) — teach-time toggle.
            flag = "True" if body.get("open") else "False"
            rc, out = sh(
                "python3 - <<'EOF'\n"
                "import sys, threading, time\n"
                f"sys.path.insert(0, '{REPO}/ros2_ws/src/unchained_cell')\n"
                "import rclpy\n"
                "from rclpy.node import Node\n"
                "from rclpy.executors import MultiThreadedExecutor\n"
                "from moveit_msgs.srv import GetPlanningScene, ApplyPlanningScene\n"
                "from unchained_cell import pick_place as pp\n"
                "rclpy.init(); n=Node('ui_nest')\n"
                "ex=MultiThreadedExecutor(); ex.add_node(n)\n"
                "threading.Thread(target=ex.spin, daemon=True).start()\n"
                "g=n.create_client(GetPlanningScene,'/get_planning_scene')\n"
                "a=n.create_client(ApplyPlanningScene,'/apply_planning_scene')\n"
                "g.wait_for_service(timeout_sec=10); a.wait_for_service(timeout_sec=10)\n"
                "time.sleep(1)\n"
                f"pp.set_nest_window(n, g, a, {flag})\n"
                "print('NEST_OK')\n"
                "rclpy.shutdown()\nEOF", timeout=30)
            self._json({"ok": "NEST_OK" in out,
                        "out": f"nest window {'OPEN' if flag=='True' else 'closed (strict)'}"
                               if "NEST_OK" in out else out[-300:]})
        elif p == "/api/teach/pose":
            self._json(teach_pose(body["deck"], body.get("plate_type", "20ml")))
        elif p == "/api/teach/capture":
            tp = teach_pose(body["deck"], body.get("plate_type", "20ml"))
            if not tp.get("ok"):
                self._json(tp); return
            # sanity: a taught correction is millimetres, not decimetres.
            # Huge deltas mean the dropdown deck isn't the deck being taught.
            worst = max(abs(tp["dx_mm"]), abs(tp["dy_mm"]), abs(tp["dz_grasp_mm"]))
            if worst > 50:
                self._json({"ok": False, "error":
                            f"capture REFUSED: delta {worst:.0f}mm vs {body['deck']} — "
                            "is the Deck dropdown set to the deck you are "
                            "actually teaching?", "pose": tp})
                return
            c = load_calib()
            entry = c.get(body["deck"], {})
            if body.get("what") == "xy":
                entry["dx_mm"], entry["dy_mm"] = tp["dx_mm"], tp["dy_mm"]
            elif body.get("what") == "z":
                entry["dz_mm"] = tp["dz_grasp_mm"]
            entry["plate_type"] = body.get("plate_type")
            entry["captured"] = datetime.now().isoformat()
            c[body["deck"]] = entry
            save_calib(c)
            self._json({"ok": True, "entry": entry, "calib": c, "pose": tp})
        elif p == "/api/teach/calib":
            self._json({"calib": load_calib()})
        elif p == "/api/bridge_param":
            # live-tune bridge params (speed/force) without relaunch
            outs = []
            for k, v in body.get("params", {}).items():
                rc, out = sh(f"ros2 param set /sdk_executor {k} {v}", timeout=15)
                outs.append(f"{k}={v}: {out.strip()[-80:]}")
            self._json({"ok": True, "out": outs})
        else:
            self._json({"error": "not found"}, 404)


if __name__ == "__main__":
    print(f"Unchained UI on http://0.0.0.0:{PORT}  (runs -> {RUNS})")
    ThreadingHTTPServer(("0.0.0.0", PORT), Handler).serve_forever()

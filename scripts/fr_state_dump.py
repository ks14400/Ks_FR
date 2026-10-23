#!/usr/bin/env python3
"""Read-only controller state dump for a Fairino arm (NO motion).

Run with the bridge STOPPED. Prints the fields that determine whether SDK
motion will execute, so the FR16 (known-good) and FR10 can be compared
field-by-field.

    cd ~/Ks_FR/linux && python3 ../scripts/fr_state_dump.py
"""
import sys
import time

import os
_sdk=os.path.expanduser("~/Ks_FR/linux")
if _sdk not in sys.path:
    sys.path.insert(0, _sdk)
from fairino import Robot

IP = sys.argv[1] if len(sys.argv) > 1 else "192.168.58.2"
r = Robot.RPC(IP)
v = r.GetSDKVersion()
if not (isinstance(v, (tuple, list)) and v[0] == 0):
    print("BRIDGE STILL UP / no connection:", v)
    sys.exit(1)
print(f"=== {IP} :: {v[1]} ===")
time.sleep(0.3)   # let the realtime state packet populate

pkg = r.robot_state_pkg
PROG = {1: "stopped", 2: "running", 3: "paused"}
RSTATE = {1: "stopped", 2: "running", 3: "paused", 4: "dragging"}
MODE = {0: "AUTO", 1: "MANUAL"}

print(f"robot_mode      : {pkg.robot_mode}  ({MODE.get(pkg.robot_mode,'?')})")
print(f"program_state   : {pkg.program_state}  ({PROG.get(pkg.program_state,'?')})")
print(f"robot_state     : {pkg.robot_state}  ({RSTATE.get(pkg.robot_state,'?')})")
print(f"main/sub code   : {pkg.main_code} / {pkg.sub_code}")
print(f"EmergencyStop   : {pkg.EmergencyStop}")
print(f"motion_done     : {pkg.motion_done}")
print(f"mc_queue_len    : {pkg.mc_queue_len}")
print(f"collisionState  : {pkg.collisionState}")
print(f"safety_stop SI0 : {pkg.safety_stop0_state}   SI1: {pkg.safety_stop1_state}")
print(f"tool / user     : {pkg.tool} / {pkg.user}")
print(f"joints (deg)    : {[round(x,2) for x in pkg.jt_cur_pos]}")

# SDK-level cross-checks
print("\n-- SDK queries --")
for name in ("GetSafetyCode", "GetRobotEmergencyStopState",
             "IsInDragTeach", "GetRobotMotionDone"):
    try:
        print(f"  {name}: {getattr(r, name)()}")
    except Exception as e:
        print(f"  {name}: err {e}")

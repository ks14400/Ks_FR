#!/usr/bin/env python3
"""Direct SDK motion test for the FR10 — NO ROS, NO bridge.

Diagnoses "MoveJ accepted but arm doesn't move". Commands ONE tiny ~4 deg
wrist (j6) move and reports the robot's exact return code + whether it
physically moved. Run with the bridge STOPPED (it holds the SDK port).

    cd ~/Ks_FR/linux && python3 ../scripts/fr10_direct_move_test.py

Keep the e-stop in hand. Open space only.
"""
import sys
import time

import os
_sdk=os.path.expanduser("~/Ks_FR/linux")
if _sdk not in sys.path:
    sys.path.insert(0, _sdk)
from fairino import Robot

r = Robot.RPC("192.168.58.2")
v = r.GetSDKVersion()
if not (isinstance(v, (tuple, list)) and v[0] == 0):
    print("BRIDGE STILL UP (SDK port busy):", v)
    sys.exit(1)
print("connected:", v[1])


def jp():
    e = r.GetActualJointPosDegree()
    return e[1] if e[0] == 0 else None


print("safety :", r.GetSafetyCode())
print("estop  :", r.GetRobotEmergencyStopState())
print("drag   :", r.IsInDragTeach())
print("enable :", r.RobotEnable(1))
print("mode(1)[MANUAL — SDK MoveJ needs this]:", r.Mode(1))
print("speed  :", r.SetSpeed(25))
time.sleep(0.3)

import math

start = jp()
print("joints before:", [round(x, 2) for x in start])

# HOME config (deg) = the cell's HOME_JOINTS (rad) converted.
HOME_RAD = [-0.35, -1.10, 1.90, -2.40, -1.571, 0.10]
tgt = [round(math.degrees(a), 3) for a in HOME_RAD]
print("home target  :", tgt)
print("biggest joint move:",
      round(max(abs(t - s) for t, s in zip(tgt, start)), 1), "deg "
      "(elbow j3 is the large one — WATCH IT)")

fk = r.GetForwardKin(tgt)                 # FK of the TARGET, for desc_pos
print("GetForwardKin(home):", fk[0],
      "->", [round(x, 1) for x in fk[1]] if fk[0] == 0 else fk)
desc = fk[1] if fk[0] == 0 else [0.0] * 6

print("\n>>> MoveJ to HOME (vel=5%, slow) — e-stop ready ...")
ret = r.MoveJ(tgt, 0, 0, desc, 5.0, 0.0, 0.0)   # ovl=0: single move, no blend
print("MoveJ returned:", ret)

time.sleep(2.0)
after = jp()
print("joints after :", [round(x, 2) for x in after])
moved = max(abs(a - s) for a, s in zip(after, start))
print(f"\n>>> biggest joint change {moved:.2f} deg  =>",
      "MOVING — SDK motion works!" if moved > 0.5 else "DID NOT MOVE")
print("    (note: MoveJ returns as soon as the move is ACCEPTED — the arm "
      "may still be traveling to home; watch it finish.)")

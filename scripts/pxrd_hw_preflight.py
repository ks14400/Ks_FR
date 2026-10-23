#!/usr/bin/env python3
"""PXRD hardware preflight — put the FR10 in a state where SDK motion runs.

PXRD-ONLY. Touches nothing the unchained cell uses. Run ONCE with the
bridge STOPPED (it holds the single SDK port), before ./run_pxrd.sh bridge.

Sets, over the SDK (NO arm motion):
  - MANUAL mode  (Mode(1)) — REQUIRED: SDK MoveJ executes only in manual
    mode; in auto mode the controller waits for a stored program and
    silently ignores SDK moves (arm accepts commands but never moves).
    Verified against the hardware-proven FR16, which runs in manual.
  - enable       (RobotEnable(1))
  - global speed (SetSpeed), default 25%% — a 0%% override also stalls MoveJ.

Then prints the resulting state so you can confirm robot_mode == 1 (MANUAL)
before starting the bridge.

    cd ~/Ks_FR/linux && python3 ../scripts/pxrd_hw_preflight.py [speed_pct]
"""
import os
import sys
import time

# Fairino SDK lives in ~/Ks_FR/linux — add it regardless of cwd.
_sdk = os.path.expanduser("~/Ks_FR/linux")
if _sdk not in sys.path:
    sys.path.insert(0, _sdk)
from fairino import Robot

IP = "192.168.58.2"
speed = int(sys.argv[1]) if len(sys.argv) > 1 else 25

r = Robot.RPC(IP)
v = r.GetSDKVersion()
if not (isinstance(v, (tuple, list)) and v[0] == 0):
    print("Cannot connect (is the bridge still running? stop it first):", v)
    sys.exit(1)
print(f"connected: {IP} :: {v[1]}")

# Safety read-back before anything
print("safety:", r.GetSafetyCode(), " estop:", r.GetRobotEmergencyStopState(),
      " drag:", r.IsInDragTeach())

print("\nSetting movable state (no arm motion):")
print("  RobotEnable(1):", r.RobotEnable(1))
print("  Mode(1) MANUAL:", r.Mode(1))
print(f"  SetSpeed({speed}):", r.SetSpeed(speed))
time.sleep(0.4)

pkg = r.robot_state_pkg
MODE = {0: "AUTO", 1: "MANUAL"}
print("\nResulting state:")
print(f"  robot_mode : {pkg.robot_mode} ({MODE.get(pkg.robot_mode,'?')})")
print(f"  main/sub   : {pkg.main_code} / {pkg.sub_code}")
print(f"  EmergencyStop: {pkg.EmergencyStop}")
if pkg.robot_mode == 1 and pkg.main_code == 0:
    print("\nOK — FR10 is MANUAL + fault-free. Start the bridge: "
          "./run_pxrd.sh bridge")
else:
    print("\nWARNING — not in the expected state; check the WebApp "
          "(Manual mode, clear faults) before starting the bridge.")

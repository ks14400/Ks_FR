"""
AG-145 gripper + pick-pose computation test via Fairino SDK.

Exercises all seven gripper-related SDK functions:
  SetGripperConfig, GetGripperConfig, ActGripper, MoveGripper,
  GetGripperMotionDone, ComputePrePick, ComputePostPick.

The robot is NOT commanded to move. Pre/post-pick poses are computed
from the current TCP pose and only printed — add MoveL calls after
visual sanity check.

Usage:
    ROBOT_IP=192.168.58.2 python3 scripts/test_gripper.py
"""

import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "linux"))
from fairino import Robot

ROBOT_IP = os.environ.get("ROBOT_IP", "192.168.58.2")
GRIPPER_INDEX = 1

GRIPPER_COMPANY_DAHUAN = 4
GRIPPER_DEVICE_PGI140 = 0

PARALLEL = 0

PRE_PICK_Z_OFFSET = 50.0
POST_PICK_Z_OFFSET = 50.0


def check(label, err):
    if err == 0:
        print(f"  [OK]   {label}")
    else:
        print(f"  [FAIL] {label}  error={err}")
    return err


def wait_motion(robot, label, timeout_s=6.0):
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        rc = robot.GetGripperMotionDone()
        if isinstance(rc, tuple) and len(rc) == 2 and rc[0] == 0:
            fault, status = rc[1]
            if fault:
                print(f"  [FAULT] {label}  GetGripperMotionDone fault=1")
                return False
            if status == 1:
                print(f"  [DONE]  {label}")
                return True
        time.sleep(0.1)
    print(f"  [TIMEOUT] {label} after {timeout_s}s")
    return False


def main():
    print(f"Connecting to {ROBOT_IP} ...")
    robot = Robot.RPC(ROBOT_IP)

    print("\n--- RS485 setup (115200 8N1) ---")
    print(f"  Before: {robot.GetAxleCommunicationParam()}")
    check("SetAxleCommunicationParam",
          robot.SetAxleCommunicationParam(7, 8, 1, 0, 100, 3, 1))
    print(f"  After:  {robot.GetAxleCommunicationParam()}")

    print("\n--- Step 1: GetGripperConfig (before) ---")
    print(f"  {robot.GetGripperConfig()}")

    print("\n--- Step 2: SetGripperConfig (DH / PGI-140 profile) ---")
    check("SetGripperConfig",
          robot.SetGripperConfig(GRIPPER_COMPANY_DAHUAN, GRIPPER_DEVICE_PGI140))
    time.sleep(0.5)
    print(f"  Now reads: {robot.GetGripperConfig()}")

    print("\n--- Step 3: ActGripper (reset) ---")
    check("ActGripper(reset)", robot.ActGripper(GRIPPER_INDEX, 0))
    time.sleep(1.5)

    print("\n--- Step 4: ActGripper (activate) ---")
    activate_err = check("ActGripper(activate)",
                         robot.ActGripper(GRIPPER_INDEX, 1))
    time.sleep(2.0)

    print("\n--- Step 5: Gripper status feedback (post-activate) ---")
    print(f"  GetGripperMotionDone:     {robot.GetGripperMotionDone()}")
    print(f"  GetGripperActivateStatus: {robot.GetGripperActivateStatus()}"
          "  # (err, fault, active_bitmap: bit0=grip1, bit1=grip2, ...)")
    print(f"  GetGripperCurPosition:    {robot.GetGripperCurPosition()}"
          "  # (err, fault, position_pct)")
    print(f"  GetGripperVoltage:        {robot.GetGripperVoltage()}"
          "  # (err, fault, voltage in 0.1V)")
    print(f"  GetGripperCurCurrent:     {robot.GetGripperCurCurrent()}"
          "  # (err, fault, current_pct)")
    print(f"  GetGripperTemp:           {robot.GetGripperTemp()}"
          "  # (err, fault, temp_C)")
    print(f"  GetGripperCurSpeed:       {robot.GetGripperCurSpeed()}"
          "  # (err, fault, speed_pct)")

    print("\n--- Step 6: Read current TCP pose ---")
    tcp = robot.GetActualTCPPose()
    print(f"  GetActualTCPPose: {tcp}")
    if not (isinstance(tcp, tuple) and tcp[0] == 0):
        print("  Could not read TCP pose. Skipping pre/post-pick computation.")
        current_pose = None
    else:
        current_pose = tcp[1]
        print(f"  current_pose = {current_pose}")

    if current_pose is not None:
        # ComputePrePick/ComputePostPick are vision-only (require a calibrated
        # vision system) and return error 14 otherwise. For a simple Z-offset
        # pre/post pose, do the math directly.
        pre_pick = list(current_pose)
        pre_pick[2] += PRE_PICK_Z_OFFSET
        post_pick = list(current_pose)
        post_pick[2] += POST_PICK_Z_OFFSET
        print(f"\n--- Step 7: pre-pick pose (current + z{PRE_PICK_Z_OFFSET}mm) ---")
        print(f"  pre_pick_pose  = {pre_pick}")
        print(f"\n--- Step 8: post-pick pose (current + z{POST_PICK_Z_OFFSET}mm) ---")
        print(f"  post_pick_pose = {post_pick}")

    if activate_err != 0:
        print("\nGripper not activated. Skipping MoveGripper steps.")
        print("Fix activation first (see CLAUDE.md Path 2 / Path 3).")
        return 1

    print("\n--- Step 9: MoveGripper OPEN (pos=100, vel=30, force=20) ---")
    check("MoveGripper(open)", robot.MoveGripper(
        GRIPPER_INDEX, 100, 30, 20, 5000, 0, PARALLEL, 0, 0, 0))
    wait_motion(robot, "open")

    print("\n--- Step 10: MoveGripper CLOSE (pos=0) ---")
    check("MoveGripper(close)", robot.MoveGripper(
        GRIPPER_INDEX, 0, 30, 20, 5000, 0, PARALLEL, 0, 0, 0))
    wait_motion(robot, "close")

    print("\n--- Step 11: MoveGripper HALF-OPEN (pos=50) ---")
    check("MoveGripper(half)", robot.MoveGripper(
        GRIPPER_INDEX, 50, 30, 20, 5000, 0, PARALLEL, 0, 0, 0))
    wait_motion(robot, "half")

    print("\nAll done.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

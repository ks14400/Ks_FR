#!/usr/bin/env python3
"""
Record waypoints on the REAL FR20 robot using drag teach mode.

Workflow:
  1. Robot enters drag teach (free-drive) mode — you can move it by hand
  2. Move the robot to a desired position
  3. Press Enter to record the position
  4. Repeat for all waypoints
  5. Press 's' to save to YAML (compatible with job_runner)

Usage:
    python3 hardware_waypoint_recorder.py --ip 192.168.58.2 --output ~/jobs/waypoints.yaml

Requirements:
    - Physical FR20 robot powered on and connected via Ethernet
    - Fairino Python SDK (already in this repo)
"""
import argparse
import os
import sys
import yaml

# Add the SDK to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "linux"))
from fairino import Robot


def main():
    parser = argparse.ArgumentParser(description="Record waypoints on real FR20")
    parser.add_argument("--ip", default="192.168.58.2", help="Robot IP address")
    parser.add_argument("--output", default="recorded_waypoints.yaml", help="Output YAML file")
    args = parser.parse_args()

    print(f"Connecting to robot at {args.ip}...")
    robot = Robot.RPC(args.ip)

    # Check connection via a harmless query (SDK version)
    ret = robot.GetSDKVersion()
    if ret[0] != 0:
        print(f"Failed to connect to robot: error code {ret[0]}")
        sys.exit(1)
    print(f"Connected. SDK: {ret[1]}")

    # Check current errors
    err = robot.GetRobotErrorCode()
    if err[0] == 0 and err[1] != [0, 0]:
        print(f"Robot has errors: {err[1]}. Resetting...")
        robot.ResetAllError()

    # Enable robot
    robot.RobotEnable(1)

    # Enter drag teach mode
    print("\n" + "=" * 60)
    print("DRAG TEACH MODE")
    print("=" * 60)
    print("The robot will now enter free-drive mode.")
    print("You can physically move the arm by hand.")
    print()
    input("Press Enter to enable drag teach mode...")

    ret = robot.DragTeachSwitch(1)
    if ret != 0:
        print(f"Failed to enable drag teach: error {ret}")
        sys.exit(1)
    print("Drag teach mode ENABLED — move the robot freely.\n")

    recorded = []

    print("Commands:")
    print("  Enter  — record current position")
    print("  l      — list recorded waypoints")
    print("  s      — save and exit")
    print("  q      — quit without saving")
    print("  d      — disable drag teach (lock joints)")
    print("  e      — re-enable drag teach")
    print()

    try:
        while True:
            cmd = input("\n> ").strip().lower()

            if cmd in ("", "r"):
                # Read current positions
                joint_ret = robot.GetActualJointPosDegree(1)
                tcp_ret = robot.GetActualTCPPose(1)

                if joint_ret[0] != 0:
                    print(f"  Failed to read joints: {joint_ret[0]}")
                    continue

                joints_deg = joint_ret[1]
                print(f"  Joints (deg): {[f'{j:.2f}' for j in joints_deg]}")

                if tcp_ret[0] == 0:
                    tcp_pose = tcp_ret[1]  # [x, y, z, rx, ry, rz] in mm and degrees
                    print(f"  TCP (mm/deg): xyz=[{tcp_pose[0]:.1f}, {tcp_pose[1]:.1f}, {tcp_pose[2]:.1f}] "
                          f"rpy=[{tcp_pose[3]:.1f}, {tcp_pose[4]:.1f}, {tcp_pose[5]:.1f}]")

                name = input("  Waypoint name (or skip): ").strip()
                if not name:
                    print("  Skipped.")
                    continue

                wp_type = input("  Type [j]oint or [c]artesian (default: j): ").strip().lower()

                if wp_type in ("c", "cartesian") and tcp_ret[0] == 0:
                    wp = {
                        "type": "cartesian",
                        "xyz_mm": [round(tcp_pose[0], 1), round(tcp_pose[1], 1), round(tcp_pose[2], 1)],
                        "rpy_deg": [round(tcp_pose[3], 1), round(tcp_pose[4], 1), round(tcp_pose[5], 1)],
                    }
                    print(f"  Recorded '{name}' (cartesian)")
                else:
                    wp = {
                        "type": "joint",
                        "joints_deg": [round(j, 2) for j in joints_deg],
                    }
                    print(f"  Recorded '{name}' (joint)")

                recorded.append((name, wp))

            elif cmd == "l":
                if not recorded:
                    print("  No waypoints recorded.")
                else:
                    print(f"\n  Recorded waypoints ({len(recorded)}):")
                    for name, wp in recorded:
                        if wp["type"] == "joint":
                            print(f"    {name}: joint {wp['joints_deg']}")
                        else:
                            print(f"    {name}: cartesian xyz={wp['xyz_mm']} rpy={wp['rpy_deg']}")

            elif cmd == "d":
                robot.DragTeachSwitch(0)
                print("  Drag teach DISABLED — joints locked.")

            elif cmd == "e":
                robot.DragTeachSwitch(1)
                print("  Drag teach ENABLED — move freely.")

            elif cmd == "s":
                _save(recorded, args.output)
                break

            elif cmd == "q":
                print("  Quit without saving.")
                break

            else:
                print("  Commands: Enter=record, l=list, s=save, q=quit, d=disable drag, e=enable drag")

    except KeyboardInterrupt:
        print("\n  Interrupted.")

    finally:
        # Always disable drag teach on exit
        print("Disabling drag teach mode...")
        robot.DragTeachSwitch(0)
        robot.CloseRPC()
        print("Disconnected from robot.")


def _save(recorded, output_path):
    if not recorded:
        print("  Nothing to save.")
        return

    output = {"waypoints": {}}
    for name, wp in recorded:
        output["waypoints"][name] = wp

    path = os.path.expanduser(output_path)
    os.makedirs(os.path.dirname(path) if os.path.dirname(path) else ".", exist_ok=True)
    with open(path, "w") as f:
        yaml.dump(output, f, default_flow_style=False, sort_keys=False)
    print(f"\n  Saved {len(recorded)} waypoints to {path}")
    print("  Copy the waypoints section into your job YAML.")


if __name__ == "__main__":
    main()

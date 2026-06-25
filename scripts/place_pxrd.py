#!/usr/bin/env python3
"""
Spawn or reposition the PXRD instrument in a running Gazebo session.
Accepts position in mm and orientation in degrees.

Usage:
    # Spawn PXRD at 500mm forward, on the floor (-585mm), rotated 90°:
    python3 place_pxrd.py --x 500 --y 0 --z -585 --yaw 90

    # Reposition (removes old, spawns new at updated pose):
    python3 place_pxrd.py --x 600 --y -100 --z -585 --yaw 180 --move

All values:
    --x     mm forward from robot base (+X)
    --y     mm left of robot base (+Y)  (negative = right)
    --z     mm above robot base (+Z)    (negative = below, e.g. -585 for floor)
    --roll  degrees rotation around X axis
    --pitch degrees rotation around Y axis
    --yaw   degrees rotation around Z axis
    --move  remove existing PXRD first, then spawn at new pose
"""
import argparse
import math
import subprocess
import os
import time


def run_ign(service, reqtype, reptype, req):
    """Run an ign service call and return (success, stdout, stderr)."""
    cmd = [
        "ign", "service",
        "-s", service,
        "--reqtype", reqtype,
        "--reptype", reptype,
        "--timeout", "5000",
        "--req", req,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    return result.returncode == 0, result.stdout, result.stderr


def main():
    p = argparse.ArgumentParser(description="Place PXRD in Gazebo (mm/degrees)")
    p.add_argument("--x", type=float, default=500, help="X in mm (forward)")
    p.add_argument("--y", type=float, default=0, help="Y in mm (left, neg=right)")
    p.add_argument("--z", type=float, default=0, help="Z in mm (up, neg=below)")
    p.add_argument("--roll", type=float, default=0, help="Roll in degrees")
    p.add_argument("--pitch", type=float, default=0, help="Pitch in degrees")
    p.add_argument("--yaw", type=float, default=0, help="Yaw in degrees")
    p.add_argument("--move", action="store_true",
                   help="Remove existing PXRD first, then re-spawn")
    args = p.parse_args()

    # Convert mm → meters, degrees → radians
    x_m = args.x / 1000.0
    y_m = args.y / 1000.0
    z_m = args.z / 1000.0
    roll_r = math.radians(args.roll)
    pitch_r = math.radians(args.pitch)
    yaw_r = math.radians(args.yaw)

    # RPY → quaternion
    cr, sr = math.cos(roll_r / 2), math.sin(roll_r / 2)
    cp, sp = math.cos(pitch_r / 2), math.sin(pitch_r / 2)
    cy, sy = math.cos(yaw_r / 2), math.sin(yaw_r / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    pose_str = (
        f"position: {{x: {x_m:.4f}, y: {y_m:.4f}, z: {z_m:.4f}}} "
        f"orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}"
    )

    print(f"Position:  x={args.x}mm  y={args.y}mm  z={args.z}mm")
    print(f"           ({x_m:.4f}m, {y_m:.4f}m, {z_m:.4f}m)")
    print(f"Rotation:  roll={args.roll}°  pitch={args.pitch}°  yaw={args.yaw}°")
    print(f"           ({roll_r:.4f}rad, {pitch_r:.4f}rad, {yaw_r:.4f}rad)")
    print()

    # Step 1: If --move, remove the existing model first
    if args.move:
        print("Removing existing PXRD...")
        ok, out, err = run_ign(
            "/world/empty/remove",
            "ignition.msgs.Entity",
            "ignition.msgs.Boolean",
            'name: "pxrd_instrument" type: MODEL',
        )
        if ok:
            print("  Removed.")
        else:
            print(f"  Remove failed (may not exist): {err.strip()}")
        time.sleep(1)

    # Step 2: Spawn with inline SDF using absolute file paths
    visual_mesh = os.path.expanduser(
        "~/ros2_ws/install/pxrd_cell/share/pxrd_cell/meshes/pxrd_smartlab_visual.stl"
    )
    collision_mesh = os.path.expanduser(
        "~/ros2_ws/install/pxrd_cell/share/pxrd_cell/meshes/pxrd_smartlab_collision.stl"
    )

    sdf = (
        '<?xml version=\\"1.0\\"?>'
        '<sdf version=\\"1.8\\">'
        '<model name=\\"pxrd_instrument\\"><static>true</static>'
        '<link name=\\"pxrd_link\\">'
        '<visual name=\\"v\\"><geometry><mesh>'
        f'<uri>file://{visual_mesh}</uri>'
        '<scale>0.001 0.001 0.001</scale>'
        '</mesh></geometry>'
        '<material>'
        '<ambient>0.7 0.7 0.75 1.0</ambient>'
        '<diffuse>0.7 0.7 0.75 1.0</diffuse>'
        '</material></visual>'
        '<collision name=\\"c\\"><geometry><mesh>'
        f'<uri>file://{collision_mesh}</uri>'
        '<scale>0.001 0.001 0.001</scale>'
        '</mesh></geometry></collision>'
        '</link></model></sdf>'
    )

    req = f'sdf: "{sdf}" pose: {{{pose_str}}} name: "pxrd_instrument"'

    print("Spawning PXRD...")
    ok, out, err = run_ign(
        "/world/empty/create",
        "ignition.msgs.EntityFactory",
        "ignition.msgs.Boolean",
        req,
    )

    if ok:
        print("Success!")
        print(f"\nTo reposition, run:")
        print(f"  python3 ~/Ks_FR/scripts/place_pxrd.py"
              f" --x {args.x} --y {args.y} --z {args.z}"
              f" --roll {args.roll} --pitch {args.pitch} --yaw {args.yaw} --move")
    else:
        print(f"Failed: {err.strip()}")
        if out:
            print(f"stdout: {out}")
        print("\nMake sure Gazebo is running:")
        print("  export IGN_GAZEBO_MODEL_PATH=~/ros2_ws/install/pxrd_cell/share/pxrd_cell/models")
        print("  ros2 launch fairino20_v6_moveit2_config digitial_fr20_gazebo_sim.launch.py")


if __name__ == "__main__":
    main()

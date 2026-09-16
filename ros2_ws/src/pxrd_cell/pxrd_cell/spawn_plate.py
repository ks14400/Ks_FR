"""
Spawn (or move) the tabbed well-plate as a MoveIt CollisionObject — LIVE,
no relaunch. Use it to dial the plate's resting pose quickly:

    ros2 run pxrd_cell spawn_plate --x 0 --y 0 --z 0
    ros2 run pxrd_cell spawn_plate --x 0.02 --z -0.01      # nudge & re-run
    ros2 run pxrd_cell spawn_plate --remove                # clear it

The mesh (the_plate_collision.stl) is re-centered on the WELL-BODY center, so
position is the well center and rotation spins about the wells. Pose is given
in the --frame frame (default pxrd_smartlab = PXRD CAD origin). Defaults match
the orientation dialed in the URDF (rot_x=180, rot_y=90).

Shows up green in rviz's PlanningScene display. Turn the URDF viz plate OFF
(launch with include_tabbed_plate:=false, the default) so they don't overlap.
"""
import argparse
import math
import os
import struct
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import Mesh, MeshTriangle

PLATE_VIZ_ID = "the_plate_viz"


def load_stl_mesh(path):
    """Parse a binary STL into a shape_msgs/Mesh (dedup vertices)."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        vmap = {}
        verts = []
        tris = []
        for _ in range(n):
            d = f.read(50)
            if len(d) < 50:
                break
            v = struct.unpack("<12f", d[:48])
            idx = []
            for k in range(3):
                key = v[3 + k * 3:6 + k * 3]
                if key not in vmap:
                    vmap[key] = len(verts)
                    verts.append(key)
                idx.append(vmap[key])
            t = MeshTriangle()
            t.vertex_indices = [idx[0], idx[1], idx[2]]
            tris.append(t)
    mesh = Mesh()
    mesh.vertices = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in verts]
    mesh.triangles = tris
    return mesh


def rpy_to_quat(r, p, y):
    """URDF rpy (R = Rz(y)Ry(p)Rx(r)) -> Quaternion."""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--x", type=float, default=0.005, help="pos X in --frame (m)")
    p.add_argument("--y", type=float, default=0.0085, help="pos Y in --frame (m)")
    p.add_argument("--z", type=float, default=0.0, help="pos Z in --frame (m)")
    p.add_argument("--rx", type=float, default=180.0, help="roll about frame X (deg)")
    p.add_argument("--ry", type=float, default=90.0, help="pitch about frame Y (deg)")
    p.add_argument("--rz", type=float, default=0.0, help="yaw about frame Z (deg)")
    p.add_argument("--frame", default="pxrd_smartlab",
                   help="reference frame (default pxrd_smartlab = PXRD origin)")
    p.add_argument("--id", default=PLATE_VIZ_ID, help="collision object id")
    p.add_argument("--remove", action="store_true", help="remove the object")
    args = p.parse_args()

    rclpy.init()
    node = Node("spawn_plate")
    ex = MultiThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()

    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    node.get_logger().info("Waiting for /apply_planning_scene ...")
    if not cli.wait_for_service(timeout_sec=10.0):
        node.get_logger().error("apply_planning_scene unavailable — is the sim up?")
        rclpy.shutdown(); sys.exit(1)

    co = CollisionObject()
    co.id = args.id
    co.header.frame_id = args.frame

    if args.remove:
        co.operation = CollisionObject.REMOVE
        node.get_logger().info(f"Removing '{args.id}'")
    else:
        share = get_package_share_directory("pxrd_cell")
        stl = os.path.join(share, "meshes", "the_plate_collision.stl")
        mesh = load_stl_mesh(stl)
        pose = Pose()
        pose.position = Point(x=args.x, y=args.y, z=args.z)
        pose.orientation = rpy_to_quat(
            math.radians(args.rx), math.radians(args.ry), math.radians(args.rz))
        co.meshes = [mesh]
        co.mesh_poses = [pose]
        co.operation = CollisionObject.ADD
        node.get_logger().info(
            f"Spawning '{args.id}' in {args.frame} at "
            f"xyz=({args.x:+.3f},{args.y:+.3f},{args.z:+.3f}) "
            f"rpy=({args.rx:.0f},{args.ry:.0f},{args.rz:.0f})deg "
            f"({len(mesh.vertices)} verts, {len(mesh.triangles)} tris)")

    ps = PlanningScene()
    ps.is_diff = True
    ps.world.collision_objects.append(co)
    req = ApplyPlanningScene.Request()
    req.scene = ps
    fut = cli.call_async(req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if fut.done() and fut.result() is not None and fut.result().success:
        node.get_logger().info("  [OK] applied — see it (green) in rviz PlanningScene")
    else:
        node.get_logger().warning("  apply returned failure/timeout (may still show)")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

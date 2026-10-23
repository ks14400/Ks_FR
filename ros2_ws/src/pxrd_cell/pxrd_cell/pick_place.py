"""
Pick a 96-well plate from one deck and place it on another.

The plate is managed as a MoveIt CollisionObject so it actually moves with the
gripper. URDF static plate must be OFF (include_plate:=false, which is now default).

Usage:
  ros2 run unchained_cell pick_place --source deck_9_10_pos1 --target deck_9_10_pos2
  ros2 run unchained_cell pick_place --source deck_9_10_pos1 --target deck_vortex_pos2 --hover 0.18

Sequence:
  1. Spawn plate at --source (if not already present)
  2. Open gripper
  3. Move arm to approach above source (hover)
  4. Descend to grasp height
  5. Close gripper (default nearly shut; force-controlled — stops on the plate)
  6. Attach plate to gripper
  7. Lift back to hover
  8. Transit to hover above target
  9. Descend to grasp height
 10. Open gripper
 11. Detach plate, re-spawn at target deck
 12. Retreat to hover above target
"""
import argparse
import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import Pose, Point, Quaternion
from shape_msgs.msg import Mesh, MeshTriangle
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import (
    Constraints, PositionConstraint, OrientationConstraint, JointConstraint,
    CollisionObject, AttachedCollisionObject, PlanningScene,
    AllowedCollisionMatrix,
)
from moveit_msgs.srv import (
    ApplyPlanningScene, GetCartesianPath, GetPositionIK, GetPlanningScene,
    GetPositionFK,
)
from moveit_msgs.msg import PlanningSceneComponents
from shape_msgs.msg import SolidPrimitive


# Plate geometry (matches the URDF macro)
PLATE_X = 0.0855  # m  (along deck local X)
PLATE_Y = 0.025   # m  (vertical thickness in deck frame)
PLATE_Z = 0.1278  # m  (along deck local Z)
PLATE_DECK_Y_OFFSET = PLATE_Y / 2.0  # raise plate so bottom = deck dot

PLATE_ID = "the_plate"

# ============================================================
# TABBED PLATE (custom PXRD grab-handle design, from check.STL)
# ============================================================
# Mesh `the_plate_collision.stl` is re-centered on the WELL-BODY center, so the
# mesh origin = well center. A grab tab protrudes ~93 mm off one end (narrower,
# ~12 mm thick) so the gripper grabs the TAB while the wells go into the
# enclosure — keeping the wrist outside. Geometry (mesh frame, meters):
#   well center      : (0, 0, 0)
#   tab center       : X = -0.0925 (tab points -X at zero rotation)
#   tab thickness (Y): 0.012   tab width (Z): ~0.077
TABBED_PLATE_MESH = "the_plate_collision.stl"
TAB_CENTER_X = -0.1106  # m, v2 paddle center offset from well center (mesh X)
TAB_THICKNESS = 0.020    # m, v2 paddle thickness (flat faces)
TAB_WIDTH = 0.077        # m, tab width (for top-down grasp across the width)

# ---- SIDE-PINCH grasp (the geometry-mandated mode for the enclosed PXRD) ----
# The instrument bay is a slot: open below Y~330mm, tall front wall above.
# A vertical (top-down) wrist stack cannot cross the wall plane — measured
# contacts: forearm/wrist1 vs pxrd_smartlab, every IK branch. So the gripper
# goes in HORIZONTALLY through the slot like a human hand, pinching the tab
# paddle's flat faces with the jaw axis vertical.
# The paddle is RAISED on the mock mesh: pinch center at deck-local Y=0.058
# (paddle 49..67mm above deck) so the lower jaw clears the table at pick and
# the bay's low structure (top +9mm) at place.
TAB_PINCH_Y = 0.0615     # m, deck-local pinch height (v2 REAL plate, measured)
SIDE_STANDOFF = 0.060    # m, approach standoff behind the paddle
SIDE_JAW_GAP_MM = 40.0   # jaw opening while approaching/releasing: enough to
                         # clear the 12-18mm paddle, small enough that the
                         # lower jaw never dips into the table / bay floor
ENCLOSED_JAW_GAP_MM = 30.0  # tighter gap for the enclosed (in-bay) acquire

# ============================================================
# PLATE REVISIONS — selected with --plate (default v2)
# ============================================================
# v2  riser design (sim-validated 2026-09): paddle RAISED on a stem, pinch
#     center 61.5mm above deck, 110.6mm from the well center.
# v3  overhang design (check.STL 2026-10-20): LOW tab 155.7mm out — the paddle
#     (15mm thick, pinch center deck+32.1mm) OVERHANGS the support edge at
#     both decks (table edge = 150mm from marker; SmartLab stage ends ~Z110),
#     so the AG-145 lower fingertip hangs in free air below deck level.
#     Jaw gap must be 30mm even on the open deck: at 40mm the fingertip's
#     forward tip passes ~3mm BELOW the table top over the table corner
#     (measured analytically from the finger FK; gripper-vs-table is masked
#     in the SRDF, so the planner would NOT catch it). 30mm clears by +2.5mm.
PLATES = {
    "v2": dict(mesh="the_plate_collision.stl",
               tab_center_x=-0.1106, tab_thickness=0.020, tab_width=0.077,
               tab_pinch_y=0.0615, side_jaw_gap_mm=40.0,
               enclosed_jaw_gap_mm=30.0),
    "v3": dict(mesh="the_plate_v3_collision.stl",
               tab_center_x=-0.1557, tab_thickness=0.015, tab_width=0.060,
               tab_pinch_y=0.0321, side_jaw_gap_mm=30.0,
               enclosed_jaw_gap_mm=30.0),
}
DEFAULT_PLATE = "v2"


def apply_plate(name):
    """Point the module-level plate constants at one PLATES revision.

    Module-level because every helper (and the importing tools) reads these
    globals at call time; mutating them keeps the whole file on one plate.
    """
    global TABBED_PLATE_MESH, TAB_CENTER_X, TAB_THICKNESS, TAB_WIDTH
    global TAB_PINCH_Y, SIDE_JAW_GAP_MM, ENCLOSED_JAW_GAP_MM, _PLATE_MESH_CACHE
    cfg = PLATES[name]
    TABBED_PLATE_MESH = cfg["mesh"]
    TAB_CENTER_X = cfg["tab_center_x"]
    TAB_THICKNESS = cfg["tab_thickness"]
    TAB_WIDTH = cfg["tab_width"]
    TAB_PINCH_Y = cfg["tab_pinch_y"]
    SIDE_JAW_GAP_MM = cfg["side_jaw_gap_mm"]
    ENCLOSED_JAW_GAP_MM = cfg["enclosed_jaw_gap_mm"]
    _PLATE_MESH_CACHE = None

# Canonical resting pose of the tabbed plate at the PXRD sample, in the
# pxrd_smartlab (CAD) frame — dialed in sim and confirmed against CAD.
# Rotation spins the plate about the well center, so the wells stay on the
# sample point for any rotation. Used by spawn_plate defaults and (soon) by
# pick_place to place/attach the real tabbed mesh instead of the box.
TABBED_PLATE_POSE = {
    "frame": "pxrd_smartlab",
    "xyz": (0.005, 0.0085, 0.0),
    "rpy_deg": (180.0, 90.0, 0.0),
}
# The plate rests on a deck with this pose IN THE DECK-LOCAL frame (both
# pxrd_sample and table_top are CAD Y-up, so the same relative pose puts the
# plate flat with the tab along deck-local +Z on either surface).
PLATE_REST_XYZ = TABBED_PLATE_POSE["xyz"]
PLATE_REST_RPY = tuple(math.radians(d) for d in TABBED_PLATE_POSE["rpy_deg"])


# ---- small transform helpers (numpy 4x4) ----
def _quat_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = math.sqrt(x*x + y*y + z*z + w*w) or 1.0
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z),   2*(y*z-x*w)],
        [2*(x*z-y*w),     2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def _R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = np.argmax([R[0, 0], R[1, 1], R[2, 2]])
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return Quaternion(x=x, y=y, z=z, w=w)


def side_grasp_q(v):
    """Horizontal-pinch grasp orientation from a horizontal unit vector v.

    gripper +Z (approach axis) points along v — into the instrument;
    gripper X (jaw-open axis) points world-up, so the jaws pinch the tab
    paddle's flat top/bottom faces. Right-handed: y = z × x.
    """
    zg = np.array([v[0], v[1], 0.0])
    zg = zg / (np.linalg.norm(zg) or 1.0)
    xg = np.array([0.0, 0.0, 1.0])
    yg = np.cross(zg, xg)
    R = np.column_stack([xg, yg, zg])
    return _R_to_quat(R)


def _rpy_to_R(r, p, y):
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _T(R, t):
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = t
    return M


def _tf_to_T(tf):
    q = tf.transform.translation
    return _T(_quat_to_R(tf.transform.rotation),
              [tf.transform.translation.x,
               tf.transform.translation.y,
               tf.transform.translation.z])


def _plate_rest_T():
    """Plate resting pose as a 4x4, in the deck-local frame."""
    return _T(_rpy_to_R(*PLATE_REST_RPY), PLATE_REST_XYZ)


def _T_to_pose(M):
    p = Pose()
    p.position.x, p.position.y, p.position.z = M[:3, 3]
    p.orientation = _R_to_quat(M[:3, :3])
    return p


_PLATE_MESH_CACHE = None


def _load_plate_mesh():
    """Load the tabbed-plate collision mesh as a shape_msgs/Mesh (cached)."""
    global _PLATE_MESH_CACHE
    if _PLATE_MESH_CACHE is not None:
        return _PLATE_MESH_CACHE
    import struct
    share = get_package_share_directory("pxrd_cell")
    path = os.path.join(share, "meshes", TABBED_PLATE_MESH)
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        vmap = {}; verts = []; tris = []
        for _ in range(n):
            d = f.read(50)
            if len(d) < 50:
                break
            v = struct.unpack("<12f", d[:48])
            idx = []
            for k in range(3):
                key = v[3+k*3:6+k*3]
                if key not in vmap:
                    vmap[key] = len(verts); verts.append(key)
                idx.append(vmap[key])
            t = MeshTriangle(); t.vertex_indices = [idx[0], idx[1], idx[2]]
            tris.append(t)
    m = Mesh()
    m.vertices = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in verts]
    m.triangles = tris
    _PLATE_MESH_CACHE = m
    return m


def tab_center_xy_in_base(node, buf, deck):
    """(x, y) in base_link of the plate TAB center when the plate rests on `deck`.
    Used to aim the top-down grasp at the tab instead of the well center."""
    T_deck = _tf_to_T(lookup_tf(node, buf, "base_link", deck))
    T_plate = _plate_rest_T()
    tab_local = np.array([TAB_CENTER_X, 0.0, 0.0, 1.0])
    tab_base = T_deck @ T_plate @ tab_local
    return float(tab_base[0]), float(tab_base[1])

# Gripper limits (URDF): per-gripper, selected with --gripper (apply_gripper).
GRIPPER_OPEN = 0.0
GRIPPER_CLOSED_JOINT = -0.65  # URDF lower limit = jaws fully closed

# ============================================================
# GRIPPER MODELS — selected with --gripper (default ag145). Both URDF macros
# expose identical link/joint names; only the joint<->width mapping differs.
#   ag145: 0.0 = open 145 mm (hardware-measured 148.7), -0.65 = closed
#   ag95 : 0.0 = open 95.2 mm, +0.93 = closed (FK-measured from the vendored
#          dh_ag95_description; pads ARE the fingertips — nothing extends
#          beyond them, so every AG-145-era fingertip clearance has MORE
#          margin on the AG-95)
# Launch the sim with the matching gripper:=<name> xacro arg!
# ============================================================
GRIPPERS = {
    "ag145": dict(stroke_mm=145.0, closed_joint=-0.65),
    "ag95":  dict(stroke_mm=95.2,  closed_joint=0.93),
}
DEFAULT_GRIPPER = "ag145"


def apply_gripper(name):
    """Point the module-level gripper constants at one GRIPPERS entry."""
    global GRIPPER_STROKE_MM, GRIPPER_CLOSED_JOINT
    cfg = GRIPPERS[name]
    GRIPPER_STROKE_MM = cfg["stroke_mm"]
    GRIPPER_CLOSED_JOINT = cfg["closed_joint"]

# AG-145 jaw opening at fully-open (100%), in mm. This is the calibration that
# converts a desired jaw width (mm) to the gripper_finger1_joint value.
# NOMINAL value — verify with:  ros2 run unchained_cell go_home --hardware --gripper-mm 70
# then measure the physical gap; if it differs, set this to (measured_gap /
# fraction_commanded) so the mm command matches reality.
GRIPPER_STROKE_MM = 145.0


def grip_mm_to_joint(mm):
    """Desired jaw opening (mm) -> gripper_finger1_joint value
    (0.0 = fully open, -0.65 = fully closed)."""
    frac = max(0.0, min(1.0, mm / GRIPPER_STROKE_MM))
    return GRIPPER_CLOSED_JOINT * (1.0 - frac)


def joint_to_grip_mm(joint):
    """gripper_finger1_joint value -> jaw opening (mm)."""
    return (1.0 - joint / GRIPPER_CLOSED_JOINT) * GRIPPER_STROKE_MM

# Gripper-down orientation, with a yaw compensation so the jaws meet the
# plate SQUARE on its sides.
#
# Why the compensation: the grasp orientation is expressed in base_link, and
# the robot base is bolted to the pedestal at -45deg (robot_mount_yaw_deg).
# A plain "gripper down" (180deg about base X) therefore lands the jaws 45deg
# off the plate — the plate sits world-aligned on the deck and didn't rotate.
# We pre-rotate by +45deg about base Z (vertical) to cancel the mount yaw.
# If robot_mount_yaw_deg changes, set GRASP_YAW_COMP_DEG = -robot_mount_yaw_deg
# and re-verify jaw alignment in rviz.
GRASP_YAW_COMP_DEG = 0.0  # PXRD arm mount yaw is 0 -> no grasp-yaw comp


def _quat_mul(a, b):
    """Hamilton product a ⊗ b (geometry_msgs Quaternion)."""
    return Quaternion(
        w=a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        x=a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        y=a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        z=a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    )


def _rz_quat(deg):
    """Quaternion for a rotation of `deg` about Z."""
    h = math.radians(deg) / 2.0
    return Quaternion(x=0.0, y=0.0, z=math.sin(h), w=math.cos(h))


# 180° about base_link X = gripper pointing straight down (jaws referenced to
# base_link axes). Pre-rotate about base Z to align jaws with the plate.
_DOWN_BASE_Q = Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
DOWN_Q = _quat_mul(_rz_quat(GRASP_YAW_COMP_DEG), _DOWN_BASE_Q)

# When attached to the gripper, the plate's "thickness" axis should point
# along the gripper's approach (-Z of gripper_grasp_link), so we rotate the
# plate -90° about X. Quaternion: (-sin(45°), 0, 0, cos(45°))
ATTACH_Q = Quaternion(x=-0.7071068, y=0.0, z=0.0, w=0.7071068)

# === GRIPPER PAD GEOMETRY (estimated; tune visually) ===
# gripper_grasp_link is at gripper local Z = 0.143 from base_link of gripper.
# The actual fingertip pad center is ~69 mm further out along gripper +Z.
# (= fingertip joint origin 0.192 m + ~20 mm pad mesh extending toward workspace)
# Visually verify in rviz: at grasp pose, the pad surfaces should land on the
# plate's top 12 mm. If pads are too LOW (under plate), increase. If too HIGH
# (above plate top), decrease.
PAD_OFFSET_FROM_TCP = 0.069   # m, in gripper +Z direction

# === GRASP SPEC ===
# Gripper pads grip the TOP 12 mm of the plate. The remaining 13 mm hangs
# below the pads when the plate is attached. This matches a typical lab
# pick-and-place where the gripper grabs the plate by its top rim.
GRIP_DEPTH = 0.012   # m, height of pad contact on plate

# === DERIVED OFFSETS ===
# When attached, plate center hangs below TCP in gripper +Z direction by:
#   (PAD_OFFSET_FROM_TCP - half_grip)  ← plate top at pad's top edge
#   + plate_half_height                ← plate center 12.5 mm below plate top
PLATE_CENTER_BELOW_TCP = PAD_OFFSET_FROM_TCP - GRIP_DEPTH / 2.0 + PLATE_Y / 2.0
# = 0.069 - 0.006 + 0.0125 = 0.0755 m

# "Home" / "ready" joint config — wrist values (j4, j5, j6) chosen to match
# the IK family MoveIt picks for typical pick/place poses. That keeps the
# gripper-Z pointing world-DOWN (so the attached plate stays HORIZONTAL).
# Only j1 differs from working pickup IK — set to 0 for neutral face-forward.
#
# Reference IK (deck_9_10_pos1 approach, plate verified flat):
#   j1=-1.478, j2=-1.096, j3=+1.906, j4=-2.381, j5=-1.571, j6=+0.093
HOME_JOINTS = {
    # j1 = +0.785 (45deg) cancels the robot's -45deg pedestal mount yaw so the
    # folded home pose faces the workspace the same way it did before the
    # mount-orientation fix. If robot_mount_yaw_deg changes, set j1 = -mount_yaw
    # (in rad) and re-verify in rviz.
    "j1": -0.35,      # yawed slightly WEST: with the pedestal shifted +0.15
                      # east (flat-carry corner clearance, 2026-10-21) the
                      # j1=0 home parks the hanging fingertip against the
                      # SmartLab's east cabinet (measured contact).
    "j2": -1.1,       # shoulder, matches pickup wrist family
    "j3": 1.9,        # elbow, matches pickup wrist family
    "j4": -2.4,       # wrist roll — KEY for plate-flat orientation
    "j5": -1.571,     # wrist pitch (gripper points down)
    "j6": 0.1,        # gripper roll, matches pickup
}

# ============================================================
# LAYER 1 — Trajectory budget thresholds (motion-quality gate)
# ============================================================
# After OMPL/joint-goal planning, before execution, the script verifies
# the trajectory against these budgets. If any check fails, the plan is
# REJECTED and the phase retries with a new OMPL seed (random). Caps the
# arm from ever executing a "crazy rotation" plan that slipped past the
# planner.
# NOTE (PXRD): budgets raised vs the unchained cell. The PXRD sample sits
# near the FR10's reach limit, so legitimate plans to it need more joint
# travel than the unchained decks did. Tunable per run with --max-travel.
MAX_TRAVEL_PER_OMPL_PHASE = 13.0  # rad — total joint travel budget per phase
MAX_SINGLE_JOINT_TRAVEL   = 6.0   # rad — per-joint travel budget
MAX_JOINT_DELTA           = 0.1   # rad — max single-step delta between waypoints
                                  #       (caps jerk; too high = jolty motion)

# For pick from a deck (and place on deck/table): TCP must be high enough
# that the pad center lands on the plate top minus half_grip.
#   pad_center_world_Z = deck_world_Z + plate_height - half_grip
#                      = deck_world_Z + 0.025 - 0.006 = deck + 0.019
#   TCP_world_Z = pad_center + PAD_OFFSET_FROM_TCP
#              = (deck + 0.019) + 0.069 = deck + 0.088
DEFAULT_GRASP_OFFSET = (PLATE_Y - GRIP_DEPTH / 2.0) + PAD_OFFSET_FROM_TCP
# = 0.019 + 0.069 = 0.088 m

GRIPPER_LINKS = [
    "gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link",
    "gripper_finger1_finger_link",     "gripper_finger2_finger_link",
    "gripper_finger1_inner_knuckle_link", "gripper_finger2_inner_knuckle_link",
    "gripper_finger1_knuckle_link",    "gripper_finger2_knuckle_link",
    "gripper_base_link", "gripper_body_link",   # body_link: AG-95 only
]
ARM_LINKS = [
    "base_link", "shoulder_link", "upperarm_link",
    "forearm_link", "wrist1_link", "wrist2_link", "wrist3_link",
]
# Links the plate is always allowed to "touch". Includes the whole gripper
# (so the gripper can approach/grasp), the arm (so the arm can swing past),
# the device body (plate sits on it), the pedestal, and every deck (plate
# may be near any of them as it's moved). Real-world-equivalent.
# Links the plate may LEGITIMATELY touch: the gripper (fingers hold it) and
# the table it rests on. NOT the arm links — the plate sweeping into the
# forearm/wrist during a pivot is a real self-collision that must be checked
# (it was masked before; the user saw the plate clip the arm in rviz).
# Stale unchained deck names removed.
DEFAULT_TOUCH_LINKS = GRIPPER_LINKS + [
    "table", "table_top",  # external lab table (loading station)
]


def lookup_tf(node, buf, parent, child, timeout=5.0):
    """TF lookup. Executor is spinning in the background thread, so we don't
    spin here — just poll the buffer."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            return buf.lookup_transform(parent, child, rclpy.time.Time())
        except Exception:
            time.sleep(0.05)
    raise RuntimeError(f"TF {parent} -> {child} unavailable")


def _joint_travel(trajectory):
    """Sum of |Δjoint| across all waypoints, all joints — a rough smoothness
    metric. A wandering path has high travel; a direct one is low."""
    pts = trajectory.joint_trajectory.points
    if len(pts) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(pts)):
        for j in range(len(pts[i].positions)):
            total += abs(pts[i].positions[j] - pts[i - 1].positions[j])
    return total


def _trajectory_stats(trajectory):
    """Compute multiple metrics from a trajectory for diagnostic logging.
    Returns dict with: travel (rad), max_joint_delta (rad), n_pts,
    duration (sec), worst_joint (name of the joint that moved the most)."""
    pts = trajectory.joint_trajectory.points
    names = list(trajectory.joint_trajectory.joint_names)
    if len(pts) < 2:
        return {"travel": 0.0, "max_joint_delta": 0.0, "n_pts": len(pts),
                "duration": 0.0, "worst_joint": "-", "worst_travel": 0.0}
    n_joints = len(pts[0].positions)
    per_joint = [0.0] * n_joints
    max_single_delta = 0.0
    for i in range(1, len(pts)):
        for j in range(n_joints):
            d = abs(pts[i].positions[j] - pts[i - 1].positions[j])
            per_joint[j] += d
            if d > max_single_delta:
                max_single_delta = d
    total = sum(per_joint)
    worst_idx = max(range(n_joints), key=lambda i: per_joint[i])
    worst_name = names[worst_idx] if names else f"joint{worst_idx}"
    # duration from last waypoint's time_from_start
    last_t = pts[-1].time_from_start
    duration = last_t.sec + last_t.nanosec * 1e-9
    return {
        "travel": total,
        "max_joint_delta": max_single_delta,
        "n_pts": len(pts),
        "duration": duration,
        "worst_joint": worst_name,
        "worst_travel": per_joint[worst_idx],
    }


def _validate_budget(trajectory,
                     max_travel=MAX_TRAVEL_PER_OMPL_PHASE,
                     max_joint_travel=MAX_SINGLE_JOINT_TRAVEL,
                     max_delta=MAX_JOINT_DELTA):
    """Layer-1 gate: check a planned trajectory against motion-quality budgets.
    Returns (ok: bool, message: str). If ok=False, the plan should be
    discarded and the phase should retry with a new random seed."""
    s = _trajectory_stats(trajectory)
    if s["travel"] > max_travel:
        return False, (f"REJECTED budget: total travel {s['travel']:.2f} > "
                       f"{max_travel:.2f} rad")
    if s["worst_travel"] > max_joint_travel:
        return False, (f"REJECTED budget: {s['worst_joint']} travels "
                       f"{s['worst_travel']:.2f} > {max_joint_travel:.2f} rad "
                       "(one joint swinging too far)")
    if s["max_joint_delta"] > max_delta:
        return False, (f"REJECTED budget: max Δ {s['max_joint_delta']:.3f} > "
                       f"{max_delta:.3f} rad (path too jerky)")
    return True, (f"travel={s['travel']:.2f}rad "
                  f"({s['worst_joint']}={s['worst_travel']:.2f}, "
                  f"maxΔ={s['max_joint_delta']:.3f}), "
                  f"{s['n_pts']}pts, traj_dur={s['duration']:.1f}s")


def _trajectory_max_tilt(node, fk_client, trajectory,
                         link="gripper_grasp_link", timeout=15.0):
    """Max tilt (deg) of the plate top-face normal across ALL waypoints of a
    planned trajectory, via /compute_fk. In the side grasp the plate normal is
    the gripper X axis; tilt = its angle from base_link +Z (vertical).
    Returns None if FK is unavailable (caller must treat as gate failure)."""
    jt = trajectory.joint_trajectory
    names = list(jt.joint_names)
    worst = 0.0
    for pt in jt.points:
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [link]
        req.robot_state.joint_state.name = names
        req.robot_state.joint_state.position = list(pt.positions)
        fut = fk_client.call_async(req)
        deadline = time.time() + timeout
        while not fut.done() and time.time() < deadline:
            time.sleep(0.002)
        if not fut.done():
            return None
        res = fut.result()
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            return None
        R = _quat_to_R(res.pose_stamped[0].pose.orientation)
        # gripper X axis in base frame = column 0; tilt = angle from +Z
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R[2][0]))))
        worst = max(worst, tilt)
    return worst


def _check_tilt_gate(node, tilt_gate, trajectory):
    """Apply the carry-tilt gate (if armed) to a planned trajectory.
    tilt_gate = (fk_client, max_deg) or None. Returns (ok, suffix_or_error)."""
    if tilt_gate is None:
        return True, ""
    fk_client, max_deg = tilt_gate
    tilt = _trajectory_max_tilt(node, fk_client, trajectory)
    if tilt is None:
        return False, "REJECTED tilt gate: /compute_fk unavailable"
    if tilt > max_deg:
        return False, (f"REJECTED tilt: plate would tip {tilt:.1f}° "
                       f"(> {max_deg:.1f}° carry limit)")
    return True, f", tilt≤{tilt:.1f}°"


# Keep each executed sub-trajectory comfortably under the shared bridge's
# budget gate (BRIDGE_MAX_TOTAL_TRAVEL_RAD=8, BRIDGE_MAX_SINGLE_JOINT_RAD=4).
CHUNK_TOTAL_RAD = 6.5
CHUNK_SINGLE_RAD = 3.5


def _execute_one_traj(node, exec_client, jtraj, timeout=None):
    """Execute a single JointTrajectory via ExecuteTrajectory; wait for it."""
    from moveit_msgs.msg import RobotTrajectory
    rt = RobotTrajectory()
    rt.joint_trajectory = jtraj
    exec_goal = ExecuteTrajectory.Goal()
    exec_goal.trajectory = rt
    fut = exec_client.send_goal_async(exec_goal)
    deadline = time.time() + 15.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    h = fut.result() if fut.done() else None
    if h is None or not h.accepted:
        return False, "execute rejected"
    rfut = h.get_result_async()
    deadline = time.time() + (timeout or EXEC_RESULT_TIMEOUT)
    while not rfut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not rfut.done():
        return False, "execute timeout"
    code = rfut.result().result.error_code.val
    if code != 1:
        return False, f"execute MoveItErrorCode {code}"
    return True, "ok"


def _execute_traj_chunked(node, exec_client, jtraj):
    """Execute a JointTrajectory, splitting it into sub-budget chunks so the
    shared bridge's per-goal travel gate never rejects a large move. Chunks
    are consecutive point ranges whose cumulative total / single-joint travel
    stays under CHUNK_*; each is re-timed from zero. One pass when the whole
    trajectory already fits."""
    pts = jtraj.points
    if len(pts) < 2:
        return True, "ok"
    names = list(jtraj.joint_names)
    nj = len(names)

    def travel(a, b):
        tot = 0.0; per = [0.0] * nj
        for i in range(a + 1, b + 1):
            for j in range(nj):
                d = abs(pts[i].positions[j] - pts[i - 1].positions[j])
                tot += d; per[j] += d
        return tot, (max(per) if per else 0.0)

    tot_all, _ = travel(0, len(pts) - 1)
    if tot_all <= CHUNK_TOTAL_RAD:
        return _execute_one_traj(node, exec_client, jtraj)

    # Build chunk boundaries greedily.
    bounds = []
    start = 0
    i = 0
    while i < len(pts) - 1:
        # extend [start..i+1] while under budget
        t, s = travel(start, i + 1)
        if (t > CHUNK_TOTAL_RAD or s > CHUNK_SINGLE_RAD) and i > start:
            bounds.append((start, i)); start = i
        else:
            i += 1
    bounds.append((start, len(pts) - 1))

    node.get_logger().info(
        f"   large move ({tot_all:.1f} rad) > bridge budget — executing in "
        f"{len(bounds)} chunks")
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    for k, (a, b) in enumerate(bounds):
        sub = JointTrajectory(); sub.joint_names = names
        t0 = pts[a].time_from_start.sec + pts[a].time_from_start.nanosec * 1e-9
        for idx in range(a, b + 1):
            p = JointTrajectoryPoint()
            p.positions = list(pts[idx].positions)
            if pts[idx].velocities:
                p.velocities = list(pts[idx].velocities)
            rel = (pts[idx].time_from_start.sec
                   + pts[idx].time_from_start.nanosec * 1e-9) - t0
            rel = max(rel, 0.0)
            p.time_from_start.sec = int(rel)
            p.time_from_start.nanosec = int((rel - int(rel)) * 1e9)
            sub.points.append(p)
        ok, info = _execute_one_traj(node, exec_client, sub)
        if not ok:
            return False, f"chunk {k+1}/{len(bounds)}: {info}"
    return True, "ok"


def _send_plan_validate_execute(node, mg_client, exec_client, goal,
                                  timeout=90.0, tilt_gate=None,
                                  traj_sink=None):
    """Plan-only via MoveGroup → validate against budgets → execute via
    ExecuteTrajectory if validated. This separates planning from execution
    so we can reject "crazy" trajectories BEFORE the arm moves."""
    # Force plan_only so we get the trajectory back without moving the arm
    goal.planning_options.plan_only = True

    fut = mg_client.send_goal_async(goal)
    deadline = time.time() + timeout
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    h = fut.result() if fut.done() else None
    if h is None or not h.accepted:
        return False, "plan rejected"
    rfut = h.get_result_async()
    deadline = time.time() + timeout
    while not rfut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not rfut.done():
        return False, "plan timeout"
    result = rfut.result().result
    if result.error_code.val != 1:
        return False, f"MoveItErrorCode {result.error_code.val}"

    trajectory = result.planned_trajectory
    # Layer-1 budget check
    ok, info = _validate_budget(trajectory)
    if not ok:
        return False, info
    # Layer-2 carry-tilt gate (only armed while the plate is held)
    ok, tilt_info = _check_tilt_gate(node, tilt_gate, trajectory)
    if not ok:
        return False, tilt_info
    info += tilt_info

    # Validated — execute. A single trajectory whose total joint travel
    # exceeds the shared bridge's safety budget (8 rad total / 4 rad single
    # joint) is REJECTED whole by the bridge on hardware (CONTROL_FAILED -4).
    # Split it into sub-budget chunks executed back-to-back; one clean goal
    # in sim (where the budget gate is not active).
    ok, einfo = _execute_traj_chunked(node, exec_client,
                                      trajectory.joint_trajectory)
    if not ok:
        return False, einfo
    if traj_sink is not None:
        traj_sink.append(trajectory.joint_trajectory)
    return True, info


# Execution wait ceiling. Only matters when a result NEVER arrives (true
# hang) — on normal completion OR abort the action returns immediately and
# we exit the loop. 90s was too short for slow hardware moves (e.g. a big
# home sweep at movej_vel_pct 3): the arm physically reached the goal but
# the wait expired and false-reported "timeout". Generous ceiling fixes that
# without hurting sim (sim results arrive in seconds).
EXEC_RESULT_TIMEOUT = 600.0


def _send_and_wait(node, client, goal, timeout=EXEC_RESULT_TIMEOUT):
    """Send a MoveGroup action goal and wait synchronously.
    Background executor handles the response callbacks while we block here.
    On success, returns details including joint_travel score."""
    fut = client.send_goal_async(goal)
    deadline = time.time() + 15.0           # goal ACCEPTANCE is fast
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    h = fut.result() if fut.done() else None
    if h is None or not h.accepted:
        return False, "rejected"
    rfut = h.get_result_async()
    deadline = time.time() + timeout         # EXECUTION may be slow on hw
    while not rfut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not rfut.done():
        return False, "timeout"
    result = rfut.result().result
    code = result.error_code.val
    if code != 1:
        return False, f"MoveItErrorCode {code}"
    s = _trajectory_stats(result.planned_trajectory)
    return True, (
        f"score: travel={s['travel']:.2f}rad "
        f"({s['worst_joint']}={s['worst_travel']:.2f}, "
        f"maxΔ={s['max_joint_delta']:.3f}), "
        f"{s['n_pts']}pts, traj_dur={s['duration']:.1f}s")


def plan_arm_pose(node, mg, base, link, x, y, z, q,
                  group="fairino10_v6_group", vel=0.2, yaw_tol=0.15,
                  planning_attempts=20, planning_time=5.0,
                  keep_orientation_along_path=False,
                  path_tilt_tol=0.3,
                  planner_id="RRTstar",
                  exec_client=None):
    """Plan the arm to a pose goal with OMPL.

    `keep_orientation_along_path`: if True, also adds the gripper orientation
    as a PATH constraint — the gripper stays within ±0.2 rad of `q` for every
    point on the trajectory, not just the final pose. Use this when carrying
    a plate (or anything that must stay flat). Yaw is left free on the path
    so OMPL has room to find a solution; it tightens to `yaw_tol` at the goal.
    """
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = group
    req.planner_id = planner_id            # asymptotically-optimal planner
    req.num_planning_attempts = planning_attempts
    req.allowed_planning_time = planning_time
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = vel

    pc = PositionConstraint()
    pc.header.frame_id = base
    pc.link_name = link
    box = SolidPrimitive(); box.type = SolidPrimitive.BOX
    box.dimensions = [0.01, 0.01, 0.01]
    pc.constraint_region.primitives.append(box)
    p = Pose(); p.position.x = x; p.position.y = y; p.position.z = z
    p.orientation = Quaternion(w=1.0)
    pc.constraint_region.primitive_poses.append(p)
    pc.weight = 1.0

    oc = OrientationConstraint()
    oc.header.frame_id = base
    oc.link_name = link
    oc.orientation = q
    oc.absolute_x_axis_tolerance = 0.1
    oc.absolute_y_axis_tolerance = 0.1
    oc.absolute_z_axis_tolerance = yaw_tol   # tight yaw → fingers locked in
    oc.weight = 1.0

    c = Constraints()
    c.position_constraints.append(pc)
    c.orientation_constraints.append(oc)
    req.goal_constraints.append(c)

    if keep_orientation_along_path:
        path_oc = OrientationConstraint()
        path_oc.header.frame_id = base
        path_oc.link_name = link
        path_oc.orientation = q
        # Tilt tolerance: configurable, default ±0.3 rad ≈ ±17°. Loose enough
        # for OMPL to plan with this constraint; tight enough for typical
        # capped vials. Tighten for open liquid containers, loosen if the
        # planner can't find a path.
        path_oc.absolute_x_axis_tolerance = path_tilt_tol
        path_oc.absolute_y_axis_tolerance = path_tilt_tol
        # Yaw is free along the path (plate is rotationally symmetric about
        # vertical anyway). Tight yaw only enforced at the goal.
        path_oc.absolute_z_axis_tolerance = 3.1416
        path_oc.weight = 1.0
        path_constraints = Constraints()
        path_constraints.orientation_constraints.append(path_oc)
        req.path_constraints = path_constraints

    # Layer-1 budget gate: if exec_client provided, validate before executing
    if exec_client is not None:
        return _send_plan_validate_execute(node, mg, exec_client, goal)
    goal.planning_options.plan_only = False
    return _send_and_wait(node, mg, goal)


def get_current_robot_state(node, scene_client):
    """Fetch the current robot state from the planning scene.
    Used as seed for IK so the solver picks a config near the current arm."""
    req = GetPlanningScene.Request()
    req.components.components = (
        PlanningSceneComponents.ROBOT_STATE
        | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
    )
    fut = scene_client.call_async(req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done():
        return None
    return fut.result().scene.robot_state


def compute_ik(node, ik_cli, base_frame, link_name, x, y, z, q,
               current_state, group="fairino10_v6_group",
               avoid_collisions=True, timeout_s=2):
    """Compute IK at (x, y, z, q) biased by current_state.
    Returns dict {joint_name: position} for the arm joints (j1..j6), or None."""
    req = GetPositionIK.Request()
    req.ik_request.group_name = group
    req.ik_request.ik_link_name = link_name
    req.ik_request.avoid_collisions = avoid_collisions
    req.ik_request.timeout.sec = timeout_s
    req.ik_request.pose_stamped.header.frame_id = base_frame
    req.ik_request.pose_stamped.pose.position.x = x
    req.ik_request.pose_stamped.pose.position.y = y
    req.ik_request.pose_stamped.pose.position.z = z
    req.ik_request.pose_stamped.pose.orientation = q
    # Seed the IK with the current state — solver biases toward solutions
    # near here, which prevents OMPL from having to traverse the long way
    if current_state is not None:
        req.ik_request.robot_state = current_state

    fut = ik_cli.call_async(req)
    deadline = time.time() + 5.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done() or fut.result() is None:
        return None
    res = fut.result()
    if res.error_code.val != 1:
        return None
    return {n: p for n, p in zip(res.solution.joint_state.name,
                                  res.solution.joint_state.position)
            if n.startswith("j")}


def compute_ik_best_of_many(node, ik_cli, base, link, x, y, z, q,
                            current_state, group="fairino10_v6_group"):
    """Try MULTIPLE IK seeds (perturbed around current state) and return the
    joint config with smallest distance from current. KDL IK is iterative
    and converges to whichever local solution is nearest the seed — so by
    sampling several seeds, we explore the IK solution space and pick the
    closest valid one. Avoids the long-way-around branch problem.
    """
    import copy
    cur = {n: p for n, p in zip(
        current_state.joint_state.name, current_state.joint_state.position)}
    cur_j1 = cur.get("j1", 0.0)

    # Explore the IK solution space over MULTIPLE joints, not just j1. KDL IK
    # converges to whichever branch is nearest the seed, so a grid of seeds
    # over base rotation (j1), elbow (j3), and wrist (j4) surfaces genuinely
    # different arm postures — important for hard/far targets where the naive
    # j1-only search keeps returning the same contorted pose every retry.
    cur_j3 = cur.get("j3", 1.5)
    cur_j4 = cur.get("j4", -2.4)
    j1_offsets = [0.0, +1.0, -1.0, +2.0, -2.0, +3.0, -3.0]
    j3_offsets = [0.0, +1.0, -1.0]       # elbow up/down branches
    j4_offsets = [0.0, +1.5, -1.5]       # wrist flip branches
    seed_configs = []
    for o1 in j1_offsets:
        for o3 in j3_offsets:
            for o4 in j4_offsets:
                s = copy.deepcopy(current_state)
                for i, n in enumerate(s.joint_state.name):
                    if n == "j1":
                        s.joint_state.position[i] = cur_j1 + o1
                    elif n == "j3":
                        s.joint_state.position[i] = cur_j3 + o3
                    elif n == "j4":
                        s.joint_state.position[i] = cur_j4 + o4
                seed_configs.append((f"j1{o1:+.0f}/j3{o3:+.0f}/j4{o4:+.0f}", s))

    best = None
    best_travel = float("inf")
    best_name = "none"
    n_tried = 0
    n_ok = 0
    for name, seed in seed_configs:
        result = compute_ik(node, ik_cli, base, link, x, y, z, q,
                            seed, group=group)
        n_tried += 1
        if result is None:
            continue
        n_ok += 1
        travel = sum(abs(result[k] - cur.get(k, 0.0)) for k in result)
        if travel < best_travel:
            best_travel = travel
            best = result
            best_name = name
    node.get_logger().info(
        f"   IK seed search: {n_ok}/{n_tried} valid → best seed '{best_name}' "
        f"with Δjoints = {best_travel:.2f} rad")
    return best, best_travel


def plan_arm_to_pose_via_ik(node, mg, ik_cli, scene_client,
                            base, link, x, y, z, q,
                            group="fairino10_v6_group", vel=0.2,
                            planning_time=10.0, planning_attempts=30,
                            planner_id="RRTstar",
                            exec_client=None,
                            keep_orientation_along_path=False,
                            path_tilt_tol=0.3, hint=None):
    """Plan to a pose by FIRST searching multiple IK seeds for the joint
    config closest to current, THEN planning a joint-space goal to it.
    OMPL is given a deterministic goal that's by construction close to
    where the arm already is — avoiding long-way-around swings.

    `keep_orientation_along_path`: hold the gripper flat (within path_tilt_tol
    roll/pitch of `q`) for the whole trajectory so a carried plate can't swing.

    `hint`: joint dict that biases the IK seed toward a known-good wrist
    family (same mechanism as plan_side_stage). Used to make the OPEN-deck
    pick land in the family the subsequent flat carry glide needs.
    """
    current_state = get_current_robot_state(node, scene_client)
    if current_state is None:
        return False, "could not fetch current robot state"
    ik_state = current_state
    if hint is not None:
        import copy
        ik_state = copy.deepcopy(current_state)
        names = list(ik_state.joint_state.name)
        pos = list(ik_state.joint_state.position)
        for i, nm in enumerate(names):
            if nm in hint:
                pos[i] = hint[nm]
        ik_state.joint_state.position = pos
        node.get_logger().info("   pose IK seeded from hint family "
                               + ", ".join(f"{k}={v:+.2f}" for k, v in hint.items()))
    target_joints = None
    if hint is not None:
        # hint-first shortcut: one collision-aware IK call from the exact
        # hint seed; the 63-seed grid (~18 s) only runs if this fails.
        target_joints = compute_ik(node, ik_cli, base, link, x, y, z, q,
                                   ik_state, group=group)
        if target_joints is not None:
            node.get_logger().info("   hint-seed IK valid — grid skipped")
    if target_joints is None:
        target_joints, ik_delta = compute_ik_best_of_many(
            node, ik_cli, base, link, x, y, z, q, ik_state, group=group)
    if target_joints is None:
        return False, "IK failed (no collision-free solution at the requested pose)"
    return plan_arm_to_joints(node, mg, target_joints, vel=vel,
                              planning_time=planning_time,
                              planning_attempts=planning_attempts,
                              planner_id=planner_id,
                              exec_client=exec_client,
                              keep_orientation_along_path=keep_orientation_along_path,
                              orient_q=q, base_frame=base, link=link,
                              path_tilt_tol=path_tilt_tol)


# Known-good stage config family per enclosed deck (measured & validated:
# all clearance numbers for the pxrd acquire/deposit were taken in this
# branch). Seeding stage IK here makes the branch DETERMINISTIC — without
# it, IK sometimes lands in the mirrored wrist family where the measured
# margins don't hold.
STAGE_HINTS = {
    # Legacy (v2-era validated) bay family. Its j1/j5 room points WEST —
    # which is why the raised staging table lives WEST of the robot
    # (flat-carry layout 2026-10-21). Eastward this family dies on the j1
    # limit (-3.05) ~0.6 rad out; the east-capable mirror family (j1=+1.66,
    # j3=-2.67, j5=-1.51) was probed but its carry needs a 256-degree wrist
    # yaw that sweeps the plate through the robot's own upper arm — east
    # placement is geometrically dead for ANY single flat-carry family.
    "pxrd_sample": {"j1": -2.44, "j2": -1.53, "j3": 2.57,
                    "j4": -1.04, "j5": 0.65, "j6": 1.57},
    # Probed at the table deposit pose: the only collision-free family
    # (mirror branches: forearm-vs-pedestal or wrist3-vs-forearm).
    "table_top":   {"j1": 0.48, "j2": -0.05, "j3": 1.98,
                    "j4": -1.93, "j5": -2.72, "j6": 1.57},
}


# Tucked carry posture (legacy family, canonical layout): IK-probed at the
# bay-stage pose pulled radially in to r=0.45 m (85%), z unchanged. During
# the carried j1 swing this keeps the plate nose at ~0.73 m from the base
# axis — clear of the SmartLab cabinet's west corner (at 0.816 m) that the
# full-radius arc grazes. j3 sits near its fold limit: do NOT tuck deeper.
TUCK_CARRY = {"j2": -1.790, "j3": 2.803, "j4": -1.013}
# Tucked-posture j1 differs from the stage j1 at the same bearing (shoulder
# lean changes with the fold): bay-side tucked j1 = stage j1 + this trim.
# The settle hop absorbs it (yaw-only + pitch-locked = still level).
TUCK_J1_TRIM = -0.465
# Extended posture for the wrist yaw (IK-probed, legacy family, r=0.60 m,
# z=0.252): far enough out that the plate nose clears the robot's shoulder
# while the pure-j5 yaw sweeps it through the inward heading.
FLIP_POSTURE = {"j2": -1.502, "j3": 2.566, "j4": -1.064}


def lerp_path(node, scene_client, exec_client, waypoints, vel=0.2,
              tilt_gate=None, step_rad=0.02, traj_sink=None,
              trusted_tilt=None, sparse_check=False):
    """Execute a DETERMINISTIC joint-space polyline through `waypoints`
    (list of {joint: value} dicts), starting from the CURRENT state.

    Every sampled state along every segment is collision-checked via
    /check_state_validity, and the merged trajectory passes the FK tilt
    gate before execution. Returns (ok, info). No avoidance — validation
    only; a blocked segment means the caller should fall back to a planned
    (OMPL) hop. The straight segments keep the flat hop decompositions
    (pure yaw / pitch-locked) level the whole way.
    """
    from moveit_msgs.srv import GetStateValidity
    import copy as _copy
    current = get_current_robot_state(node, scene_client)
    if current is None:
        return False, "could not fetch current robot state"
    cur = dict(zip(current.joint_state.name, current.joint_state.position))
    names = ["j1", "j2", "j3", "j4", "j5", "j6"]
    pts = [[cur[j] for j in names]]
    for wp in waypoints:
        pts.append([wp[j] for j in names])

    if not hasattr(node, "_sv_client"):
        node._sv_client = node.create_client(
            GetStateValidity, "/check_state_validity")
        node._sv_client.wait_for_service(timeout_sec=5.0)
    sv = node._sv_client

    def _mkreq(q_i):
        st = _copy.deepcopy(current)
        pos = list(st.joint_state.position)
        for k, nm in enumerate(st.joint_state.name):
            if nm in names:
                pos[k] = q_i[names.index(nm)]
        st.joint_state.position = pos
        req = GetStateValidity.Request()
        req.robot_state = st
        req.group_name = "fairino10_v6_group"
        return req

    def _valid_batch(samples):
        """Validate many configs with a window of in-flight service calls.
        Returns (ok, err_info). Serial round-trips cost ~100 ms each under
        rviz load; 8 in flight cuts the pre-carry pause ~8x."""
        WINDOW = 8
        idx = 0
        inflight = []   # (sample_index, future, sent_at, retried)
        while idx < len(samples) or inflight:
            while idx < len(samples) and len(inflight) < WINDOW:
                inflight.append((idx, sv.call_async(_mkreq(samples[idx])),
                                 time.time(), False))
                idx += 1
            time.sleep(0.002)
            still = []
            for (k, fut, t0, retried) in inflight:
                if fut.done():
                    res = fut.result()
                    if res is None or not res.valid:
                        pairs = {f"{c.contact_body_1}<->{c.contact_body_2}"
                                 for c in (res.contacts if res else [])}
                        return False, (f"blocked at sample {k+1}/{len(samples)}: "
                                       + (", ".join(sorted(pairs)[:3])
                                          or "in collision"))
                elif time.time() - t0 > 10.0:
                    if retried:
                        return False, (f"validity check timed out "
                                       f"(sample {k+1}/{len(samples)})")
                    still.append((k, sv.call_async(_mkreq(samples[k])),
                                  time.time(), True))
                else:
                    still.append((k, fut, t0, retried))
            inflight = still
        return True, ""

    from trajectory_msgs.msg import JointTrajectoryPoint
    from moveit_msgs.msg import RobotTrajectory
    traj = RobotTrajectory()
    traj.joint_trajectory.joint_names = names
    v_joint = max(0.05, vel * 1.5)   # rad/s nominal, scaled by vel
    t = 0.0
    total_d = 0.0
    check_samples = []
    # the trajectory ALWAYS starts at the exact current state at t=0 —
    # controllers abort otherwise when the first real point is offset.
    pt0 = JointTrajectoryPoint()
    pt0.positions = list(pts[0])
    traj.joint_trajectory.points.append(pt0)
    for s_i in range(len(pts) - 1):
        start, goal = pts[s_i], pts[s_i + 1]
        max_d = max(abs(g - s) for s, g in zip(start, goal))
        if max_d < 1e-4:
            continue
        n = max(2, int(math.ceil(max_d / step_rad)) + 1)
        for i in range(n):
            a = i / (n - 1)
            if i == 0:
                continue   # vertex already added (or it's the t=0 point)
            q_i = [s + a * (g - s) for s, g in zip(start, goal)]
            # sparse mode (taught path + unchanged layout fingerprint): the
            # dense check was done at teach time; spot-check ~1 in 16.
            keep = (i % 16 == 0) if sparse_check else (i % 2 == 0)
            if keep or i == n - 1:
                check_samples.append(q_i)
            pt = JointTrajectoryPoint()
            pt.positions = q_i
            # strictly increasing time with a floor per step: near-zero-length
            # bridge segments otherwise emit sub-ms-spaced points that abort
            # the trajectory controller (CONTROL_FAILED -4).
            t = max(t + 0.02, (total_d + a * max_d) / v_joint)
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.joint_trajectory.points.append(pt)
        total_d += max_d

    if len(traj.joint_trajectory.points) < 2:
        return True, "already there (Δ<1e-4)"

    ok, err = _valid_batch(check_samples)
    if not ok:
        return False, f"lerp {err}"

    if trusted_tilt is not None:
        # tilt of a FIXED joint path never changes (FK is a pure function of
        # joint values) — the teach-time measurement stands; skip re-FKing
        # every waypoint (this was ~60 s of the pre-carry pause).
        tilt_info = f", tilt≤{trusted_tilt:.1f}° (taught, FK-verified at teach time)"
    else:
        ok, tilt_info = _check_tilt_gate(node, tilt_gate, traj)
        if not ok:
            return False, tilt_info

    # Execute in bridge-budget chunks (a long carry exceeds the shared
    # bridge's 8-rad per-goal cap and would be rejected whole on hardware).
    ok, einfo = _execute_traj_chunked(node, exec_client, traj.joint_trajectory)
    if not ok:
        return False, f"lerp {einfo}"
    if traj_sink is not None:
        traj_sink.append(traj.joint_trajectory)
    return True, (f"LERP {len(traj.joint_trajectory.points)}pts "
                  f"({len(pts)-1} segs), Δ={total_d:.2f}rad, "
                  f"traj_dur={t:.1f}s{tilt_info}")


def lerp_hop(node, scene_client, exec_client, goal_joints, vel=0.2,
             tilt_gate=None, step_rad=0.02, traj_sink=None):
    """Single-goal wrapper around lerp_path."""
    return lerp_path(node, scene_client, exec_client, [goal_joints],
                     vel=vel, tilt_gate=tilt_gate, step_rad=step_rad,
                     traj_sink=traj_sink)


def _taught_carry_path(source, target):
    """Load TAUGHT carry waypoints for (source -> target).

    Looks for config/taught_carry_<source>__<target>.json; falls back to
    the opposite direction file REVERSED (a time-reversed valid flat path
    is itself valid and flat). Returns list of {joint: value} or None.
    """
    share = get_package_share_directory("pxrd_cell")
    fwd = os.path.join(share, "config", f"taught_carry_{source}__{target}.json")
    rev = os.path.join(share, "config", f"taught_carry_{target}__{source}.json")
    if os.path.exists(fwd):
        with open(fwd) as f:
            data = json.load(f)
        wps = data["waypoints"]
    elif os.path.exists(rev):
        with open(rev) as f:
            data = json.load(f)
        wps = list(reversed(data["waypoints"]))
    else:
        return None
    names = data["joint_names"]
    return ([dict(zip(names, w)) for w in wps],
            {"max_tilt_deg": data.get("max_tilt_deg"),
             "layout_fingerprint": data.get("layout_fingerprint")})


def _layout_fingerprint(node, scene_client):
    """Hash of everything that defines collision geometry: the URDF (layout
    args are xacro-baked into robot_description) + world collision objects.
    Lets a taught-path replay skip the dense per-sample re-validation when
    the world is PROVABLY unchanged since teach time (the validity service
    is serialized server-side: ~250 ms/sample, ~60 s per carry otherwise).
    Returns a hex digest or None if anything cannot be fetched."""
    import hashlib
    h = hashlib.sha256()
    try:
        from rcl_interfaces.srv import GetParameters
        if not hasattr(node, "_rsp_param_cli"):
            node._rsp_param_cli = node.create_client(
                GetParameters, "/robot_state_publisher/get_parameters")
            node._rsp_param_cli.wait_for_service(timeout_sec=5.0)
        req = GetParameters.Request()
        req.names = ["robot_description"]
        fut = node._rsp_param_cli.call_async(req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done() or not fut.result().values:
            return None
        urdf = fut.result().values[0].string_value
        # Strip the <ros2_control> and <gazebo> blocks: they differ between
        # mock/gazebo/hardware control modes but DON'T change collision
        # geometry. Without this a sim-taught path's fingerprint never matches
        # on hardware, forcing the slow dense re-check every carry.
        import re as _re
        urdf = _re.sub(r"<ros2_control\b.*?</ros2_control>", "", urdf,
                       flags=_re.DOTALL)
        urdf = _re.sub(r"<gazebo\b.*?</gazebo>", "", urdf, flags=_re.DOTALL)
        h.update(urdf.encode())
    except Exception:
        return None
    try:
        req = GetPlanningScene.Request()
        req.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_NAMES
            | PlanningSceneComponents.WORLD_OBJECT_GEOMETRY)
        if not hasattr(node, "_gps_fp_cli"):
            node._gps_fp_cli = node.create_client(
                GetPlanningScene, "/get_planning_scene")
            node._gps_fp_cli.wait_for_service(timeout_sec=5.0)
        fut = node._gps_fp_cli.call_async(req)
        deadline = time.time() + 5.0
        while not fut.done() and time.time() < deadline:
            time.sleep(0.01)
        if not fut.done():
            return None
        for co in sorted(fut.result().scene.world.collision_objects,
                         key=lambda c: c.id):
            h.update(co.id.encode())
            pp = co.pose
            h.update((f"{pp.position.x:.3f},{pp.position.y:.3f},"
                      f"{pp.position.z:.3f}").encode())
            h.update(str(len(co.meshes)).encode())
    except Exception:
        return None
    return h.hexdigest()


def plan_side_stage(node, mg, ik_cli, scene_client, x, y, z, q,
                    vel=0.2, planning_time=15.0, planning_attempts=30,
                    planner_id="RRTConnect", exec_client=None, hint=None,
                    j1_first=False, tilt_gate=None,
                    source=None, target=None):
    """Two-hop staging for the side carry: OMPL reliably fails to connect the
    lifted-pick config to the stage config in one go (long j1 swing + posture
    change with a long plate attached). So: (1) solve IK for the stage pose,
    (2) hop 1 — swing ONLY j1 to the stage branch's heading (posture held),
    (3) hop 2 — close the remaining wrist/elbow gap. Both hops are short,
    collision-checked joint goals."""
    # ── TAUGHT CARRY REPLAY (before any IK — the taught endpoint IS the
    # stage config for this fixed layout, so the 63-call IK seed search is
    # skipped entirely). Every sample is collision-checked against the LIVE
    # scene (parallel batched); the tilt is the teach-time FK measurement
    # (a fixed joint path can never change tilt). Recorded one way, replayed
    # REVERSED for the other direction.
    if tilt_gate is not None and source and target:
        found = _taught_carry_path(source, target)
        if found is not None:
            taught, meta = found
            fkc, max_deg = tilt_gate
            trusted = meta.get("max_tilt_deg")
            if trusted is not None and trusted > max_deg:
                node.get_logger().warning(
                    f"   taught path tilt {trusted:.1f}° exceeds the gate "
                    f"({max_deg:.1f}°) — ignoring the taught path")
            else:
                fp_now = _layout_fingerprint(node, scene_client)
                fp_rec = meta.get("layout_fingerprint")
                sparse = (fp_now is not None and fp_rec is not None
                          and fp_now == fp_rec)
                node.get_logger().info(
                    f"   taught carry path found ({len(taught)} waypoints) — "
                    "replaying ("
                    + ("layout unchanged: sparse spot-check" if sparse
                       else "layout changed/unknown: dense collision check")
                    + (", teach-time tilt" if trusted is not None
                       else ", tilt gated") + ")")
                ok, info = lerp_path(node, scene_client, exec_client, taught,
                                     vel=vel, tilt_gate=tilt_gate,
                                     trusted_tilt=trusted,
                                     sparse_check=sparse)
                if ok:
                    return True, f"taught replay: {info}"
                node.get_logger().warning(
                    f"   whole-path replay rejected ({info}) — retrying in "
                    "chunks")
                CH = 12
                all_ok = True
                for c0 in range(0, len(taught), CH):
                    ok, info = lerp_path(node, scene_client, exec_client,
                                         taught[c0:c0 + CH], vel=vel,
                                         tilt_gate=tilt_gate,
                                         trusted_tilt=trusted)
                    if not ok:
                        all_ok = False
                        node.get_logger().warning(
                            f"   chunk {c0//CH+1} rejected ({info}) — "
                            "falling back to the gated hop ladder")
                        break
                if all_ok:
                    return True, f"taught replay (chunked): {info}"

    current = get_current_robot_state(node, scene_client)
    if current is None:
        return False, "could not fetch current robot state"
    # Seed the IK search around the HINT config (proven branch) when given,
    # so the solution lands in the measured family, not its mirror.
    ik_state = current
    if hint is not None:
        import copy
        ik_state = copy.deepcopy(current)
        names = list(ik_state.joint_state.name)
        pos = list(ik_state.joint_state.position)
        for i, nm in enumerate(names):
            if nm in hint:
                pos[i] = hint[nm]
        ik_state.joint_state.position = pos
        node.get_logger().info("   stage IK seeded from known-good branch "
                               + ", ".join(f"{k}={v:+.2f}" for k, v in hint.items()))
    target_joints, ik_delta = compute_ik_best_of_many(
        node, ik_cli, "base_link", "gripper_grasp_link", x, y, z, q, ik_state)
    if target_joints is None:
        return False, "IK failed (no collision-free solution at stage pose)"
    cur = dict(zip(current.joint_state.name, current.joint_state.position))



    # ── Recording: set PXRD_RECORD_CARRY=/path/out.json to capture the
    # executed carry (all hop trajectories, concatenated) for teaching. ──
    record_path = os.environ.get("PXRD_RECORD_CARRY") \
        if tilt_gate is not None else None
    traj_sink = [] if record_path else None

    # Hop ladder (VALIDATED order — used both empty-jawed and carrying).
    # ORDER IS START-DEPENDENT:
    #  from HOME (low, folded): posture FIRST — raising j2/j3 before the
    #    j1 swing keeps the hanging gripper out of the pedestal footprint.
    #  from the BAY MOUTH (j1_first=True): j1 swing FIRST — re-posturing
    #    while still aimed at the instrument reaches into it (99999);
    #    swinging away in the current carry posture is safe.
    # While CARRYING (tilt_gate armed) each hop is additionally:
    #  - tried first as a straight joint-space LERP (deterministic, often
    #    exactly level), falling back to OMPL when the segment is blocked;
    #  - FK-tilt-gated with ITS OWN retry loop: OMPL only sometimes samples
    #    a level path for a hop, but hops are short, so a handful of rolls
    #    converges (measured) where one-shot whole-carry planning never
    #    does (80 mining rolls: best 41.7 deg). The wrist re-wrap MUST be a
    #    combined j4/j5/j6 hop — a pure-j5 yaw sweeps the plate through the
    #    robot's own shoulder at table-pick radii.
    # Hop ladder (legacy VALIDATED order, both empty-jawed and carrying).
    # ORDER IS START-DEPENDENT:
    #  from HOME: posture FIRST (low j1 swing parks the gripper in the
    #  pedestal); from the BAY MOUTH (j1_first): swing away first.
    # While carrying, each hop is tried as a straight LERP first and then
    # as gated OMPL rolls — the FK tilt gate rejects any plan that would
    # tip the plate, and retries explore until a level one is found.
    hops = []
    via = {j: cur[j] for j in ("j1", "j2", "j3", "j4", "j5", "j6")}
    if tilt_gate is not None:
        # CARRYING — every hop LEVEL BY CONSTRUCTION (verified by the gate):
        #  - pure j1: yaw about the base axis;
        #  - posture {j2,j3,j4} together: start and target postures share
        #    pitch-sum zero, so the straight lerp is pitch-locked AND lands
        #    exactly on the target posture;
        #  - pure j5: yaw about the (vertical) wrist axis — done at the OPEN
        #    deck end, never at the bay mouth.
        # Requires the pedestal_off_x=0.15 layout: the carried j1 swing arc
        # then clears the SmartLab cabinet's west corner by ~10 cm (it
        # GRAZES at 1 mm with the pedestal at x=0 — structurally, since the
        # legacy family cannot fold below the stage radius).
        # The wrist yaw happens at the BAY end: with the pedestal stepped
        # back (off_z=0.70) the bay-mouth yaw circle clears the face by
        # 4-14 cm, and the swing then runs with the DECK-family wrist so the
        # plate nose TRAILS inboard — clearing the face mullions/pillars
        # that an outward nose rakes at bearings 90-130 deg (measured).
        if j1_first:
            via["j5"] = target_joints["j5"]
            hops.append(("wrist yaw (pure j5, bay mouth)", dict(via)))
            via["j1"] = target_joints["j1"]
            hops.append(("j1 swing (nose trailing)", dict(via)))
            for j in ("j2", "j3", "j4"):
                via[j] = target_joints[j]
            hops.append(("posture j2/j3/j4 (pitch-locked)", dict(via)))
        else:
            for j in ("j2", "j3", "j4"):
                via[j] = target_joints[j]
            hops.append(("posture j2/j3/j4 (pitch-locked)", dict(via)))
            via["j1"] = target_joints["j1"]
            hops.append(("j1 swing (nose trailing)", dict(via)))
            via["j5"] = target_joints["j5"]
            hops.append(("wrist yaw (pure j5, bay mouth)", dict(via)))
        hops.append(("settle stage config", dict(target_joints)))
    else:
        if j1_first:
            via["j1"] = target_joints["j1"]
            hops.append(("j1 swing (carry posture held)", dict(via)))
            via["j2"] = target_joints["j2"]; via["j3"] = target_joints["j3"]
            hops.append(("posture (j2/j3)", dict(via)))
        else:
            via["j2"] = target_joints["j2"]; via["j3"] = target_joints["j3"]
            hops.append(("posture (j2/j3)", dict(via)))
            via["j1"] = target_joints["j1"]
            hops.append(("j1 swing", dict(via)))
        for j in ("j4", "j5", "j6"):
            via[j] = target_joints[j]
        hops.append(("wrist pre-orient", dict(via)))
        hops.append(("settle stage config", dict(target_joints)))
    for i, (label, jgoal) in enumerate(hops):
        node.get_logger().info(f"   stage hop {i+1}/{len(hops)}: {label}")
        if tilt_gate is not None:
            ok, info = lerp_hop(node, scene_client, exec_client, jgoal,
                                vel=vel, tilt_gate=tilt_gate,
                                traj_sink=traj_sink)
            if not ok and "could not fetch current robot state" in info:
                # the node's service executor is wedged — nothing downstream
                # can work; fail fast instead of burning retries.
                return False, f"stage hop {i+1} ({label}): {info} (fast abort)"
            if not ok:
                node.get_logger().info(
                    f"   hop lerp blocked ({info}) — gated OMPL rolls")
                for roll in range(14):
                    ok, info = plan_arm_to_joints(
                        node, mg, jgoal, vel=vel,
                        planning_time=planning_time,
                        planning_attempts=planning_attempts,
                        planner_id=planner_id, exec_client=exec_client,
                        tilt_gate=tilt_gate, traj_sink=traj_sink)
                    if ok:
                        break
                    node.get_logger().info(
                        f"   hop roll {roll+1}/14 rejected: {info}")
        else:
            ok, info = plan_arm_to_joints(node, mg, jgoal, vel=vel,
                                          planning_time=planning_time,
                                          planning_attempts=planning_attempts,
                                          planner_id=planner_id,
                                          exec_client=exec_client,
                                          tilt_gate=tilt_gate)
        if not ok:
            return False, f"stage hop {i+1} ({label}): {info}"

    if record_path and traj_sink:
        names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        wps = []
        for jt in traj_sink:
            idx = [list(jt.joint_names).index(n) for n in names]
            for p in jt.points:
                wps.append([p.positions[k] for k in idx])
        # downsample: keep points at least 0.04 rad apart (always keep last)
        slim = [wps[0]]
        for w in wps[1:]:
            if max(abs(a - b) for a, b in zip(w, slim[-1])) >= 0.04:
                slim.append(w)
        if slim[-1] != wps[-1]:
            slim.append(wps[-1])
        # teach-time tilt: FK the slim waypoints once and store it — replays
        # then skip the per-run FK pass (a fixed joint path cannot change
        # tilt).
        rec_tilt = None
        if tilt_gate is not None:
            from moveit_msgs.msg import RobotTrajectory as _RT
            from trajectory_msgs.msg import JointTrajectoryPoint as _JP
            _tr = _RT(); _tr.joint_trajectory.joint_names = names
            for w in slim:
                _p = _JP(); _p.positions = list(w)
                _tr.joint_trajectory.points.append(_p)
            rec_tilt = _trajectory_max_tilt(node, tilt_gate[0], _tr)
        with open(record_path, "w") as f:
            json.dump({"joint_names": names, "waypoints": slim,
                       "source": source, "target": target,
                       "max_tilt_deg": rec_tilt,
                       "layout_fingerprint":
                           _layout_fingerprint(node, scene_client)},
                      f, indent=1)
        node.get_logger().info(
            f"   RECORDED carry: {len(slim)} waypoints -> {record_path}")
    return True, info


def plan_arm_to_joints(node, mg, joint_values, vel=0.2, planning_time=5.0,
                       planning_attempts=10, planner_id="RRTstar",
                       exec_client=None,
                       keep_orientation_along_path=False, orient_q=None,
                       base_frame="base_link", link="gripper_grasp_link",
                       path_tilt_tol=0.3, tilt_gate=None, traj_sink=None):
    """Plan the arm group to a specific joint configuration (deterministic).

    Use this for "go to home" or any time you want the arm in a known config
    regardless of previous state.

    joint_values: dict {joint_name: target_position_rad}

    `keep_orientation_along_path`: if True and `orient_q` is given, the gripper
    is held within ±path_tilt_tol rad of `orient_q` (roll/pitch) for EVERY
    point on the trajectory — keeps a carried plate flat so it can't swing
    while the arm reconfigures. Yaw is left free so OMPL can still plan.
    """
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "fairino10_v6_group"
    req.planner_id = planner_id
    req.num_planning_attempts = planning_attempts
    req.allowed_planning_time = planning_time
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = vel

    c = Constraints()
    for jname, jval in joint_values.items():
        jc = JointConstraint()
        jc.joint_name = jname
        jc.position = jval
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)

    if keep_orientation_along_path and orient_q is not None:
        path_oc = OrientationConstraint()
        path_oc.header.frame_id = base_frame
        path_oc.link_name = link
        path_oc.orientation = orient_q
        path_oc.absolute_x_axis_tolerance = path_tilt_tol
        path_oc.absolute_y_axis_tolerance = path_tilt_tol
        path_oc.absolute_z_axis_tolerance = 3.1416   # yaw free along path
        path_oc.weight = 1.0
        path_constraints = Constraints()
        path_constraints.orientation_constraints.append(path_oc)
        req.path_constraints = path_constraints

    if exec_client is not None:
        return _send_plan_validate_execute(node, mg, exec_client, goal,
                                           tilt_gate=tilt_gate,
                                           traj_sink=traj_sink)
    goal.planning_options.plan_only = False
    return _send_and_wait(node, mg, goal)


def plan_gripper(node, mg, joint_value, vel=0.5):
    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "gripper"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 2.0
    req.max_velocity_scaling_factor = vel
    req.max_acceleration_scaling_factor = vel

    jc = JointConstraint()
    jc.joint_name = "gripper_finger1_joint"
    jc.position = joint_value
    jc.tolerance_above = 0.01
    jc.tolerance_below = 0.01
    jc.weight = 1.0

    c = Constraints()
    c.joint_constraints.append(jc)
    req.goal_constraints.append(c)
    goal.planning_options.plan_only = False
    return _send_and_wait(node, mg, goal)


def _apply_scene(node, scene_client, ps, timeout=5.0):
    """Push a PlanningScene diff via the /apply_planning_scene SERVICE.

    Synchronous (blocks until MoveIt applies the change). Relies on a
    MultiThreadedExecutor spinning the node in a background thread, so the
    service response callback can run concurrently with our blocking wait.
    """
    req = ApplyPlanningScene.Request()
    req.scene = ps
    future = scene_client.call_async(req)
    deadline = time.time() + timeout
    while not future.done() and time.time() < deadline:
        time.sleep(0.01)
    if not future.done():
        node.get_logger().error("apply_planning_scene timed out")
        return False
    result = future.result()
    if result is None or not result.success:
        node.get_logger().error("apply_planning_scene returned failure")
        return False
    return True


def spawn_obstacle_plates(node, scene_client, deck_names):
    """Spawn plates as STATIC CollisionObjects (not attached) at the given
    decks. These act as obstacles the planner avoids. They have unique IDs
    so they don't conflict with `the_plate` (which is what pick_place picks).
    """
    if not deck_names:
        return True
    for i, deck in enumerate(deck_names):
        obstacle_id = f"obstacle_plate_at_{deck}"
        co = CollisionObject()
        co.header.frame_id = deck
        co.id = obstacle_id
        box = SolidPrimitive(); box.type = SolidPrimitive.BOX
        box.dimensions = [PLATE_X, PLATE_Y, PLATE_Z]
        co.primitives.append(box)
        p = Pose()
        p.position.y = PLATE_DECK_Y_OFFSET
        p.orientation = Quaternion(w=1.0)
        co.primitive_poses.append(p)
        co.operation = CollisionObject.ADD

        # Allow this obstacle to touch the deck/unchained surfaces it sits on
        acm = AllowedCollisionMatrix()
        acm.default_entry_names = [obstacle_id]
        acm.default_entry_values = [True]

        ps = PlanningScene(); ps.is_diff = True
        ps.world.collision_objects.append(co)
        ps.allowed_collision_matrix = acm
        _apply_scene(node, scene_client, ps)
        node.get_logger().info(
            f"   spawned obstacle plate '{obstacle_id}' at {deck}")
    return True


def purge_obstacle_plates(node, scene_client, deck_names):
    """Remove obstacle plates spawned by spawn_obstacle_plates."""
    for deck in deck_names:
        obstacle_id = f"obstacle_plate_at_{deck}"
        rm = CollisionObject(); rm.id = obstacle_id
        rm.operation = CollisionObject.REMOVE
        ps = PlanningScene(); ps.is_diff = True
        ps.world.collision_objects.append(rm)
        _apply_scene(node, scene_client, ps)
    return True


def purge_plate(node, scene_client):
    """Remove `the_plate` (and stray viz/obstacle copies) from the scene.
    Idempotent; call at script startup.

    ORDER MATTERS (MoveIt semantics): removing an AttachedCollisionObject
    RE-INSERTS the object into the world scene — so detach from every
    candidate link FIRST, then remove from the world. The old world-remove-
    first order silently left ghost plates in the world (found via the
    unchained cell's back-to-back run poisoning; same lineage here).
    """
    candidate_parents = ["gripper_grasp_link"] + [
        "pxrd_sample", "table_top",
    ]
    for parent in candidate_parents:
        rm = AttachedCollisionObject()
        rm.link_name = parent
        rm.object.id = PLATE_ID
        rm.object.operation = CollisionObject.REMOVE
        ps = PlanningScene(); ps.is_diff = True
        ps.robot_state.is_diff = True
        ps.robot_state.attached_collision_objects.append(rm)
        _apply_scene(node, scene_client, ps)

    ps2 = PlanningScene(); ps2.is_diff = True
    for oid in [PLATE_ID, "the_plate_viz"]:
        rm_world = CollisionObject(); rm_world.id = oid
        rm_world.operation = CollisionObject.REMOVE
        ps2.world.collision_objects.append(rm_world)
    _apply_scene(node, scene_client, ps2)
    return True


def _attach_plate(node, scene_client, link_name, pose, current_parent=None,
                  extra_touch=None):
    """Attach the plate to `link_name` with the given pose-in-link.
    If `current_parent` is given, first detach from there.

    Always uses AttachedCollisionObject — the plate is never a free world
    object, so we never depend on ACM diffs for non-URDF links. touch_links
    handles all the collision whitelisting cleanly.

    `extra_touch`: additional links the plate may legitimately contact in
    THIS attachment only (e.g. pxrd_smartlab while RESTING on the sample
    shelf). Never added for the gripper-carry attach, so the carried plate
    is always fully collision-checked against the instrument.
    """
    # Detach from previous parent first (if any)
    if current_parent is not None:
        rm = AttachedCollisionObject()
        rm.link_name = current_parent
        rm.object.id = PLATE_ID
        rm.object.operation = CollisionObject.REMOVE
        ps_rm = PlanningScene(); ps_rm.is_diff = True
        ps_rm.robot_state.is_diff = True
        ps_rm.robot_state.attached_collision_objects.append(rm)
        if not _apply_scene(node, scene_client, ps_rm):
            return False

    # Attach to new link — the tabbed plate MESH (single source of truth).
    aco = AttachedCollisionObject()
    aco.link_name = link_name
    aco.object.id = PLATE_ID
    aco.object.header.frame_id = link_name
    aco.object.meshes.append(_load_plate_mesh())
    aco.object.mesh_poses.append(pose)
    aco.object.operation = CollisionObject.ADD
    aco.touch_links = DEFAULT_TOUCH_LINKS + [link_name] + (extra_touch or [])

    ps = PlanningScene(); ps.is_diff = True
    ps.robot_state.is_diff = True
    ps.robot_state.attached_collision_objects.append(aco)
    return _apply_scene(node, scene_client, ps)


def attach_plate_to_deck(node, scene_client, deck_name, current_parent=None):
    """Attach the tabbed plate to a deck at its canonical resting pose (wells on
    the deck marker, tab along deck-local +Z).

    Resting on the PXRD sample stage means legitimate contact with the
    instrument body — whitelisted HERE ONLY (the carried plate stays fully
    checked against pxrd_smartlab)."""
    pose = _T_to_pose(_plate_rest_T())
    extra = ["pxrd_smartlab"] if deck_name == "pxrd_sample" else None
    return _attach_plate(node, scene_client, deck_name, pose, current_parent,
                         extra_touch=extra)


TAB_BELOW_TCP = PAD_OFFSET_FROM_TCP  # seat the tab center at the pad center


def attach_plate_to_gripper(node, scene_client, current_parent, buf=None,
                            source_deck=None, seat_tab=False):
    """Attach the tabbed plate to gripper_grasp_link as a RIGID grasp.

    Read the actual TF at grasp time: the plate rests on `source_deck`, so its
    world pose is T(base<-deck) * T(plate_rest); in the gripper frame
    plate_in_gripper = inv(T(base<-gripper)) * T(base<-plate), keeping the plate
    where it physically is (rigid grab).

    `seat_tab` (top mode): the AG-145 grip is cosmetic in sim (the plate is held
    by the attached object, not pad friction; the REAL grip is force-controlled).
    So slide the plate along the gripper so the TAB sits centered under the pads
    — it looks gripped, and puts the tab exactly under the TCP, which also makes
    the downstream place land the wells right on the target.
    """
    if buf is not None and source_deck is not None:
        T_grip = _tf_to_T(lookup_tf(node, buf, "base_link", "gripper_grasp_link"))
        T_deck = _tf_to_T(lookup_tf(node, buf, "base_link", source_deck))
        T_plate_base = T_deck @ _plate_rest_T()
        T_plate_grip = np.linalg.inv(T_grip) @ T_plate_base
        if seat_tab:
            R = T_plate_grip[:3, :3]
            t = T_plate_grip[:3, 3]
            tab_now = R @ np.array([TAB_CENTER_X, 0.0, 0.0]) + t
            tab_target = np.array([0.0, 0.0, TAB_BELOW_TCP])
            T_plate_grip[:3, 3] = t + (tab_target - tab_now)
        pose = _T_to_pose(T_plate_grip)
        return _attach_plate(node, scene_client, "gripper_grasp_link", pose,
                             current_parent)
    # legacy fallback
    pose = Pose()
    pose.position.z = PLATE_CENTER_BELOW_TCP
    pose.orientation = ATTACH_Q
    return _attach_plate(
        node, scene_client, "gripper_grasp_link", pose, current_parent)


def goto_above_deck(node, mg, buf, deck, hover, vel, yaw_tol):
    tf = lookup_tf(node, buf, "base_link", deck)
    return plan_arm_pose(node, mg, "base_link", "gripper_grasp_link",
                         tf.transform.translation.x,
                         tf.transform.translation.y,
                         tf.transform.translation.z + hover,
                         DOWN_Q, vel=vel, yaw_tol=yaw_tol)


def goto_at_deck(node, mg, buf, deck, grasp_offset, vel, yaw_tol):
    tf = lookup_tf(node, buf, "base_link", deck)
    return plan_arm_pose(node, mg, "base_link", "gripper_grasp_link",
                         tf.transform.translation.x,
                         tf.transform.translation.y,
                         tf.transform.translation.z + grasp_offset,
                         DOWN_Q, vel=vel, yaw_tol=yaw_tol)


def cartesian_move(node, cart_client, exec_client, base_frame, link,
                   x, y, z, q, vel=0.2, max_step=0.005, tilt_gate=None):
    """Plan + execute a straight-line Cartesian move from current pose to (x,y,z,q).

    Collision-checked every `max_step` (5 mm). Returns (ok, fraction, n_pts, details).
    `ok` is True only if fraction >= 0.99 AND execution succeeded.
    """
    target = Pose()
    target.position.x = x; target.position.y = y; target.position.z = z
    target.orientation = q

    # ----- Plan -----
    req = GetCartesianPath.Request()
    req.header.frame_id = base_frame
    req.group_name = "fairino10_v6_group"
    req.link_name = link
    req.max_step = max_step
    # Enforce IK continuity: with 0.0 (no check) the follower can FLIP wrist
    # family mid-path — the waypoints stay on the line but the arm teleports
    # through a huge joint jump (seen on long flat-carry glides: path "valid"
    # yet end config in a different family). The threshold is RELATIVE to the
    # path's mean joint step: keep it loose (5.0) — family-flip teleports are
    # 10x+ the mean, while a tight 2.0 false-positives on short few-point
    # moves (it broke the validated 60mm back-off at 66.7%).
    req.jump_threshold = 5.0
    req.avoid_collisions = True
    req.waypoints = [target]

    fut = cart_client.call_async(req)
    deadline = time.time() + 10.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done():
        return False, 0.0, 0, "cartesian-plan: service timed out"
    res = fut.result()
    fraction = res.fraction
    traj = res.solution
    n_pts = len(traj.joint_trajectory.points) if traj.joint_trajectory.points else 0
    if fraction < 0.99:
        return False, fraction, n_pts, (
            f"cartesian-plan: only {fraction*100:.1f}% reachable as a straight line "
            f"(blocked by collision or singularity beyond that point)")

    # Carry-tilt gate (armed only while the plate is held)
    gate_ok, gate_info = _check_tilt_gate(node, tilt_gate, traj)
    if not gate_ok:
        return False, fraction, n_pts, gate_info

    # ----- Execute -----
    goal = ExecuteTrajectory.Goal()
    goal.trajectory = traj
    fut = exec_client.send_goal_async(goal)
    deadline = time.time() + 10.0
    while not fut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not fut.done():
        return False, fraction, n_pts, "cartesian-exec: send timeout"
    h = fut.result()
    if h is None or not h.accepted:
        return False, fraction, n_pts, "cartesian-exec: rejected"
    rfut = h.get_result_async()
    deadline = time.time() + 60.0
    while not rfut.done() and time.time() < deadline:
        time.sleep(0.02)
    if not rfut.done():
        return False, fraction, n_pts, "cartesian-exec: result timeout"
    code = rfut.result().result.error_code.val
    if code != 1:
        return False, fraction, n_pts, f"cartesian-exec: MoveItErrorCode {code}"

    s = _trajectory_stats(traj)
    return True, fraction, n_pts, (
        f"frac=1.0, {n_pts}pts, travel={s['travel']:.2f}rad "
        f"({s['worst_joint']}={s['worst_travel']:.2f}), "
        f"traj_dur={s['duration']:.1f}s{gate_info}")


# ============================================================================
# Phase-logged step runner. Prints planner, target, duration, outcome for every
# phase so a failure tells you WHAT failed and HOW LONG it took, no guessing.
# ============================================================================

def _run_phase(node, phases, name, planner, fn, max_retries=4, **details):
    """Run a phase function, log start/end, append to phases list.

    On failure, automatically retries up to `max_retries` times. OMPL planning
    has random sampling — a phase that fails once may succeed on retry with a
    different seed. Real-cobot pick-and-place loops do the same thing.
    """
    bar = "=" * 64
    node.get_logger().info(bar)
    node.get_logger().info(f"PHASE: {name}   [{planner}]")
    for k, v in details.items():
        node.get_logger().info(f"   {k}: {v}")

    t0 = time.time()
    last_detail_str = ""
    for attempt in range(max_retries + 1):
        attempt_t0 = time.time()
        ok, *extra = fn()
        attempt_dur = time.time() - attempt_t0
        last_detail_str = extra[-1] if extra else ""
        if ok:
            total_dur = time.time() - t0
            note = f" (succeeded on retry {attempt})" if attempt > 0 else ""
            phases.append((name, planner, total_dur, True, last_detail_str + note))
            node.get_logger().info(f"   [OK]   {total_dur:.2f}s   {last_detail_str}{note}")
            return True
        # Failed this attempt
        node.get_logger().warning(
            f"   attempt {attempt+1}/{max_retries+1} failed in {attempt_dur:.2f}s: "
            f"{last_detail_str}")

    # All attempts exhausted
    total_dur = time.time() - t0
    phases.append((name, planner, total_dur, False, last_detail_str))
    node.get_logger().error(f"   [FAIL] {total_dur:.2f}s   {last_detail_str}")
    return False


def _print_config(node, args):
    """Print all run parameters at the start so logs are self-documenting."""
    bar = "=" * 64
    node.get_logger().info(bar)
    node.get_logger().info("RUN CONFIG")
    node.get_logger().info(bar)
    for k, v in sorted(vars(args).items()):
        node.get_logger().info(f"  {k:24s} = {v}")
    node.get_logger().info(bar)


def _extract_travel(info_str):
    """Pull the 'travel=X.XX' number out of a phase info string for aggregating."""
    import re
    m = re.search(r"travel=([\d.]+)", info_str)
    return float(m.group(1)) if m else 0.0


def _print_summary(node, phases):
    bar = "=" * 64
    node.get_logger().info(bar)
    node.get_logger().info("PHASE SUMMARY")
    node.get_logger().info(bar)
    total_dur = 0.0
    total_travel = 0.0
    n_retried = 0
    n_failed = 0
    for name, planner, dur, ok, info in phases:
        mark = "OK  " if ok else "FAIL"
        node.get_logger().info(
            f"  [{mark}] {name:35s} {planner:11s} {dur:6.2f}s   {info}")
        total_dur += dur
        total_travel += _extract_travel(info)
        if "retry" in info: n_retried += 1
        if not ok: n_failed += 1
    node.get_logger().info(bar)
    node.get_logger().info(f"  TOTAL motion time:  {total_dur:6.2f} s")
    node.get_logger().info(f"  TOTAL joint travel: {total_travel:6.2f} rad   "
                           f"(lower = smoother across whole cycle)")
    node.get_logger().info(f"  Phases retried:     {n_retried}")
    node.get_logger().info(f"  Phases failed:      {n_failed}")
    node.get_logger().info(bar)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True)
    p.add_argument("--target", required=True)
    p.add_argument("--hover", type=float, default=0.15,
                   help="Height (m) above deck for approach/lift (default 0.15)")
    p.add_argument("--grasp-offset", type=float, default=DEFAULT_GRASP_OFFSET,
                   help=f"TCP height above the deck/table dot at grasp & place. "
                        f"Computed default ({DEFAULT_GRASP_OFFSET:.3f}) puts the "
                        f"gripper pads on the top 12 mm of the plate, with the "
                        f"rest of the plate hanging below. Tune PAD_OFFSET_FROM_TCP "
                        f"in pick_place.py if the pads don't visually align.")
    p.add_argument("--pick-lift", type=float, default=0.020,
                   help="Extra height (m) added to --grasp-offset for the DECK "
                        "PICK only (not the table place). Raises where the gripper "
                        "closes so the fingertips don't bottom out on the deck. "
                        "Default 0.020 = grip 20 mm higher than the place height.")
    p.add_argument("--place-lift", type=float, default=0.0,
                   help="Extra height (m) added to --grasp-offset for the PLACE "
                        "descend only (not the pick). Raises where the plate is "
                        "released so the deep reach into an enclosed instrument "
                        "stops short of the arm's full-extension singularity. "
                        "Default 0.0 = place at grasp_offset (flush). Try 0.015 "
                        "if the place descend stalls near the bottom.")
    p.add_argument("--grasp-mode", choices=["side", "top", "center"],
                   default="side",
                   help="How to grab the tabbed plate. 'side' (default): "
                        "HORIZONTAL pinch on the raised tab paddle — the only "
                        "mode that fits through the PXRD's front slot (a "
                        "vertical wrist stack cannot cross the bay wall — "
                        "measured). 'top': top-down grab of the tab (works at "
                        "open decks, NOT into the PXRD). 'center': legacy grab "
                        "over the well center.")
    p.add_argument("--insert-depth", type=float, default=0.20,
                   help="Top-grasp place: horizontal slide-in distance (m). The "
                        "arm stages this far outside the target along the "
                        "tab->wells axis, then slides the plate straight in so "
                        "the wells pass through the opening while the wrist "
                        "stays outside the enclosure.")
    p.add_argument("--grasp-yaw-deg", type=float, default=0.0,
                   help="Rotate the gripper about vertical (deg) at the grasp so "
                        "the open jaws STRADDLE the tab instead of hitting it. "
                        "Dial visually: if the jaws cross the tab wrong, try 90. "
                        "Applied to the whole carry so the plate stays consistent.")
    p.add_argument("--via-joints", default="",
                   help="Comma-separated j1,j2,j3,j4,j5,j6 (rad) for the staging "
                        "pose the arm routes THROUGH while carrying the plate "
                        "(replaces the default 'via home'). Use this to pre-orient "
                        "the arm toward the PXRD opening (turned toward it, plate "
                        "flat, folded) so the final move is a clean straight "
                        "extension into the enclosure instead of a wall-breaking "
                        "swing. Empty = use HOME. E.g. --via-joints "
                        "1.57,-1.3,2.2,-2.4,-1.571,0.1")
    p.add_argument("--grip-value", type=float, default=None,
                   help="Gripper joint target when gripping (default: 98.5%% "
                        "of the selected gripper's closed value). "
                        "The AG gripper is FORCE-controlled: "
                        "it closes toward this target and STOPS on the plate when "
                        "it meets resistance (force = gripper_force_pct on the "
                        "bridge). So this just needs to be tighter than the plate; "
                        "the force sensor decides the real grip. Use --grip-mm to "
                        "command a specific width instead.")
    p.add_argument("--grip-mm", type=float, default=None,
                   help="Jaw opening (mm) to close to when gripping the plate. "
                        "Overrides --grip-value. E.g. --grip-mm 70. Uses the "
                        "GRIPPER_STROKE_MM calibration in pick_place.py.")
    p.add_argument("--vel", type=float, default=0.2,
                   help="Arm velocity scaling (default 0.2 for sim). In "
                        "hardware mode, clamped to <=0.10 unless explicitly "
                        "overridden with --vel.")
    p.add_argument("--hardware", action="store_true",
                   help="Hardware-safe defaults: clamps --vel to 0.10 max "
                        "and sets it to 0.05 if not otherwise specified. "
                        "Set this flag whenever running against the real "
                        "robot (bridge.launch.py is active).")
    p.add_argument("--pick-only", action="store_true",
                   help="Stop right after the pick+lift, holding the plate "
                        "(attached in the scene). Use to teach the reach-in "
                        "path incrementally with goto_joints, finding one good "
                        "joint config at a time.")
    p.add_argument("--no-return-home", action="store_true",
                   help="Skip the post-place exit (reverse out through staging "
                        "then home). By default the arm pulls back out of the "
                        "instrument and returns to HOME so it doesn't end deep "
                        "in the enclosure. Use this to leave the arm at the "
                        "retreat pose (e.g. for chaining moves).")
    p.add_argument("--no-gripper", action="store_true",
                   help="Skip all gripper open/close phases. Required when "
                        "running on real hardware until a gripper bridge "
                        "(separate from the arm bridge) is implemented. "
                        "Useful for testing the arm motion sequence in "
                        "isolation. Plate attach/detach (visual only) still "
                        "happens for sim consistency.")
    p.add_argument("--pre-release-lift", type=float, default=0.015,
                   help="Cartesian lift before opening at place (m, default 0.015). "
                        "Gives the fingertips clearance from the table/deck before "
                        "they spread — canonical industrial release behavior.")
    p.add_argument("--path-tilt-tol", type=float, default=0.3,
                   help="Tilt tolerance (rad) for the gripper during transit "
                        "while carrying the plate. Default 0.3 (~17°). Tighten "
                        "for sensitive contents (open liquids); loosen if "
                        "transit planning fails.")
    p.add_argument("--keep-plate-flat", action="store_true",
                   help="Add an explicit path orientation constraint during "
                        "transit (gripper stays within --path-tilt-tol of "
                        "vertical for the entire trajectory). OFF by default "
                        "because it makes OMPL planning much more fragile. "
                        "Enable only if your contents (open liquids etc.) "
                        "genuinely require it; otherwise the natural OMPL "
                        "path keeps the gripper roughly vertical anyway.")
    p.add_argument("--transit-time", type=float, default=15.0,
                   help="OMPL planning time for the constrained transit phase "
                        "(seconds). Path constraints make planning slower, so "
                        "this is longer than the default 5s used for other "
                        "OMPL phases (default 15.0).")
    p.add_argument("--planner", default="RRTstar",
                   help="OMPL planner ID. RRTstar = asymptotically optimal "
                        "(keeps refining path as long as planning time allows). "
                        "RRTConnect = fast but greedy. PRMstar / BITstar also "
                        "available (depends on moveit config). Default RRTstar.")
    p.add_argument("--planning-time", type=float, default=10.0,
                   help="OMPL planning time per attempt for non-transit phases "
                        "(seconds). Longer time on RRTstar = smoother path. "
                        "Default 10.0.")
    p.add_argument("--num-attempts", type=int, default=30,
                   help="OMPL number of independent planning attempts. MoveIt "
                        "picks the best (shortest) result among all attempts. "
                        "More attempts = higher chance of a clean path but "
                        "slower wall-clock. Default 30.")
    p.add_argument("--yaw-tol", type=float, default=0.4,
                   help="Yaw tolerance (rad) for gripper orientation. ~0.4 (≈23°) "
                        "keeps fingers along the plate's short side but gives IK "
                        "room. Tighten to 0.15 if grip is rotating too far; loosen "
                        "to 1.0 if a deck refuses to plan")
    p.add_argument("--no-plate", action="store_true",
                   help="Diagnostic: skip spawning the plate entirely. Lets us "
                        "isolate whether the descent failure is caused by the "
                        "plate's presence in the planning scene")
    p.add_argument("--also-spawn-at", default="",
                   help="Comma-separated list of additional deck names to spawn "
                        "static plate obstacles at, BEFORE the pick. Lets you "
                        "simulate scenarios like 'plate already on deck_9_10_pos1, "
                        "now pick from deck_9_10_pos2'. The arm will plan around "
                        "those plates. Example: "
                        "--also-spawn-at deck_9_10_pos1,deck_9_10_pos3")
    p.add_argument("--plate", choices=sorted(PLATES), default=DEFAULT_PLATE,
                   help="Plate revision (mesh + tab geometry + jaw gaps). "
                        "v2 = riser design (sim-validated 2026-09). "
                        "v3 = low overhang-tab design (check.STL 2026-10-20). "
                        f"Default {DEFAULT_PLATE}.")
    p.add_argument("--gripper", choices=sorted(GRIPPERS), default=DEFAULT_GRIPPER,
                   help="Gripper model (joint<->width mapping). MUST match the "
                        f"sim's gripper:= xacro arg. Default {DEFAULT_GRIPPER}.")
    p.add_argument("--max-carry-tilt", type=float, default=10.0,
                   help="HARD tilt gate while the plate is held (deg, default "
                        "10). Every waypoint of every carried transit "
                        "trajectory is FK-checked; a plan that would tip the "
                        "plate top face more than this is rejected before "
                        "execution (sample must not spill). 0 disables.")
    p.add_argument("--no-glide", action="store_true",
                   help="Skip the flat Cartesian carry glide and always use "
                        "the staged-ladder carry (debug). The ladder is still "
                        "tilt-gated while the plate is held.")
    args = p.parse_args()

    # ── Gripper + plate: must be applied before any geometry is computed ──
    apply_gripper(args.gripper)
    if args.grip_value is None:
        args.grip_value = 0.985 * GRIPPER_CLOSED_JOINT
    print(f"[gripper] model {args.gripper}: stroke {GRIPPER_STROKE_MM:.1f}mm "
          f"closed_joint {GRIPPER_CLOSED_JOINT:+.2f} grip_value "
          f"{args.grip_value:+.3f}", file=sys.stderr)
    apply_plate(args.plate)
    print(f"[plate] revision {args.plate}: mesh={TABBED_PLATE_MESH} "
          f"tab_center_x={TAB_CENTER_X:+.4f} pinch_y={TAB_PINCH_Y:.4f} "
          f"jaw_gap={SIDE_JAW_GAP_MM:.0f}/{ENCLOSED_JAW_GAP_MM:.0f}mm",
          file=sys.stderr)

    # ── Pick grasp height: deck pick stops --pick-lift higher than place ──
    # so the fingertips clear the deck. Place still uses plain grasp_offset.
    pick_offset = args.grasp_offset + args.pick_lift

    # ── Place height: --place-lift raises the release point above grasp_offset
    # so the deep reach into an enclosed instrument stops short of the arm's
    # full-extension singularity. 0 = flush place (original behaviour). ──
    place_offset = args.grasp_offset + args.place_lift

    # ── Staging (via) pose the arm routes through while carrying the plate ──
    # Default = HOME. --via-joints overrides it with a PXRD-ready pose.
    via_joints = dict(HOME_JOINTS)
    if args.via_joints.strip():
        vals = [float(v) for v in args.via_joints.split(",")]
        if len(vals) != 6:
            print(f"--via-joints needs 6 values, got {len(vals)}", file=sys.stderr)
            sys.exit(2)
        via_joints = {f"j{i+1}": vals[i] for i in range(6)}

    # ── Gripper width: --grip-mm overrides --grip-value ──
    if args.grip_mm is not None:
        args.grip_value = grip_mm_to_joint(args.grip_mm)
        print(f"[gripper] --grip-mm {args.grip_mm} -> grip-value "
              f"{args.grip_value:.3f} (stroke {GRIPPER_STROKE_MM:.0f}mm)",
              file=sys.stderr)

    # ── Hardware velocity clamp ──
    # When running against real hardware, refuse to use sim-typical fast speeds.
    # NOTE: the EFFECTIVE arm speed is also gated by the bridge's
    # `movej_vel_pct` (default 3%), so --vel here is a planning scaling only.
    if args.hardware:
        HARDWARE_VEL_CEIL = 0.50
        HARDWARE_VEL_DEFAULT_SAFE = 0.05
        if args.vel > HARDWARE_VEL_CEIL:
            print(f"[hardware] --vel {args.vel} exceeds safety ceiling "
                  f"{HARDWARE_VEL_CEIL}; clamping.", file=sys.stderr)
            args.vel = HARDWARE_VEL_CEIL
        elif args.vel == 0.2:   # user left at sim default
            print(f"[hardware] using safe default --vel "
                  f"{HARDWARE_VEL_DEFAULT_SAFE} instead of sim default 0.2",
                  file=sys.stderr)
            args.vel = HARDWARE_VEL_DEFAULT_SAFE

    rclpy.init()
    node = Node("pick_place")
    _print_config(node, args)
    buf = Buffer(); TransformListener(buf, node)

    # MultiThreadedExecutor spinning in the background so service-call responses
    # arrive even while we're blocked waiting for them. TF callbacks and the
    # service-response callback can run concurrently on different threads.
    cb_group = ReentrantCallbackGroup()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    mg = ActionClient(node, MoveGroup, "/move_action", callback_group=cb_group)
    exec_client = ActionClient(node, ExecuteTrajectory, "/execute_trajectory",
                               callback_group=cb_group)
    scene_client = node.create_client(
        ApplyPlanningScene, "/apply_planning_scene", callback_group=cb_group)
    cart_client = node.create_client(
        GetCartesianPath, "/compute_cartesian_path", callback_group=cb_group)
    ik_client = node.create_client(
        GetPositionIK, "/compute_ik", callback_group=cb_group)
    get_scene_client = node.create_client(
        GetPlanningScene, "/get_planning_scene", callback_group=cb_group)
    fk_client = node.create_client(
        GetPositionFK, "/compute_fk", callback_group=cb_group)

    node.get_logger().info("Waiting for /move_action ...")
    if not mg.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("move_group not available")
        rclpy.shutdown(); sys.exit(1)

    node.get_logger().info("Waiting for /execute_trajectory ...")
    if not exec_client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("execute_trajectory not available")
        rclpy.shutdown(); sys.exit(1)

    node.get_logger().info("Waiting for /apply_planning_scene service ...")
    if not scene_client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("apply_planning_scene service not available")
        rclpy.shutdown(); sys.exit(1)

    node.get_logger().info("Waiting for /compute_cartesian_path service ...")
    if not cart_client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("compute_cartesian_path not available")
        rclpy.shutdown(); sys.exit(1)

    node.get_logger().info("Waiting for /compute_ik service ...")
    if not ik_client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("compute_ik not available")
        rclpy.shutdown(); sys.exit(1)

    node.get_logger().info("Waiting for /get_planning_scene service ...")
    if not get_scene_client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("get_planning_scene not available")
        rclpy.shutdown(); sys.exit(1)

    phases = []  # phase log for the end-of-run summary

    def fail(msg):
        node.get_logger().error(msg)
        _print_summary(node, phases)
        rclpy.shutdown(); sys.exit(1)

    # ── Carry-tilt gate: armed for every phase executed while the plate is
    # held. FK-checks all waypoints of the planned trajectory and REJECTS
    # plans that would tip the plate (sample must not spill). ──
    carry_gate = ((fk_client, args.max_carry_tilt)
                  if args.max_carry_tilt > 0 else None)

    def attempt_glide(name, gx, gy, gz, gq):
        """Try the FLAT CARRY GLIDE: one straight Cartesian segment with the
        orientation locked (plate stays level by construction). Returns True
        and logs an OK phase on success; returns False (caller falls back to
        the staged ladder) without recording a failed phase."""
        bar = "=" * 64
        node.get_logger().info(bar)
        node.get_logger().info(f"PHASE: {name}   [Cartesian]")
        node.get_logger().info("   mode: flat carry glide — straight line, "
                               "orientation locked (plate level)")
        node.get_logger().info(f"   to: ({gx:+.3f}, {gy:+.3f}, {gz:+.3f})")
        t0 = time.time()
        ok, frac, n_pts, info = cartesian_move(
            node, cart_client, exec_client, "base_link", "gripper_grasp_link",
            gx, gy, gz, gq, vel=args.vel, max_step=0.01,
            tilt_gate=carry_gate)
        dur = time.time() - t0
        if ok:
            phases.append((name, "Cartesian", dur, True, info))
            node.get_logger().info(f"   [OK]   {dur:.2f}s   {info}")
            return True
        node.get_logger().warning(
            f"   glide unavailable ({info}) — falling back to staged ladder")
        return False

    # ============================================================
    # PHASE 0a: purge any stale plate state  (idempotent reset)
    # ============================================================
    _run_phase(node, phases, "purge stale plate state", "scene-diff",
               lambda: (purge_plate(node, scene_client), "ok"))

    # ============================================================
    # PHASE 0a': spawn extra obstacle plates  (multi-plate scenarios)
    # ============================================================
    extra_decks = [d.strip() for d in args.also_spawn_at.split(",") if d.strip()]
    if extra_decks:
        _run_phase(node, phases,
                   f"spawn obstacle plates at: {', '.join(extra_decks)}",
                   "scene-diff",
                   lambda: (spawn_obstacle_plates(node, scene_client, extra_decks),
                            f"{len(extra_decks)} obstacle(s)"))

    # ============================================================
    # PHASE 0b: go to home pose  (deterministic starting state)
    # ============================================================
    if not _run_phase(node, phases, "go to home", "OMPL",
                      lambda: plan_arm_to_joints(node, mg, HOME_JOINTS, vel=args.vel,
                                                  exec_client=exec_client),
                      target_joints=", ".join(f"{k}={v:+.2f}" for k, v in HOME_JOINTS.items())):
        fail("go-to-home failed")

    plate_parent = None  # tracks the link the plate is currently attached to
    if args.no_plate:
        node.get_logger().warning("--no-plate: skipping plate spawn (diagnostic)")
    else:
        ok = _run_phase(node, phases,
                        f"attach plate to {args.source}", "scene-diff",
                        lambda: (attach_plate_to_deck(node, scene_client, args.source),
                                 "ok"),
                        link=args.source)
        if not ok: fail("scene-diff failed")
        plate_parent = args.source

    # Resolve source/target deck poses once
    src_tf = lookup_tf(node, buf, "base_link", args.source)
    tgt_tf = lookup_tf(node, buf, "base_link", args.target)
    sx, sy, sz = (src_tf.transform.translation.x,
                  src_tf.transform.translation.y,
                  src_tf.transform.translation.z)
    tx, ty, tz = (tgt_tf.transform.translation.x,
                  tgt_tf.transform.translation.y,
                  tgt_tf.transform.translation.z)
    node.get_logger().info(f"Source deck in base_link: ({sx:+.3f}, {sy:+.3f}, {sz:+.3f})")
    node.get_logger().info(f"Target deck in base_link: ({tx:+.3f}, {ty:+.3f}, {tz:+.3f})")

    # ── TAB GRASP: aim the grasp at the plate TAB, not the wells ──
    # The plate rests with its wells on the deck marker and the tab protruding
    # ~93mm along deck-local +Z. Shifting the grasp XY to the tab keeps the
    # wrist outside the enclosure while the wells still land on the marker.
    insert_v = None    # unit XY tab->wells at the TARGET (deposit axis)
    src_v = None       # unit XY tab->wells at the SOURCE (acquire axis)
    if args.grasp_mode in ("top", "side"):
        stab_x, stab_y = tab_center_xy_in_base(node, buf, args.source)
        ttab_x, ttab_y = tab_center_xy_in_base(node, buf, args.target)
        node.get_logger().info(
            f"Tab grasp (source): ({stab_x:+.3f}, {stab_y:+.3f})  "
            f"shift=({stab_x-sx:+.3f}, {stab_y-sy:+.3f})m from wells")
        node.get_logger().info(
            f"Tab grasp (target): ({ttab_x:+.3f}, {ttab_y:+.3f})  "
            f"shift=({ttab_x-tx:+.3f}, {ttab_y-ty:+.3f})m from wells")
        # Insert axes: from the tab toward the wells at each deck — the
        # direction the plate slides in over that deck's marker while the
        # wrist stays behind the paddle.
        ivx, ivy = tx - ttab_x, ty - ttab_y
        norm = math.hypot(ivx, ivy) or 1.0
        insert_v = (ivx / norm, ivy / norm)
        svx, svy = sx - stab_x, sy - stab_y
        snorm = math.hypot(svx, svy) or 1.0
        src_v = (svx / snorm, svy / snorm)
        node.get_logger().info(
            f"Insert axis target (tab->wells): ({insert_v[0]:+.3f}, {insert_v[1]:+.3f})")
        node.get_logger().info(
            f"Insert axis source (tab->wells): ({src_v[0]:+.3f}, {src_v[1]:+.3f})")
        sx, sy = stab_x, stab_y
        tx, ty = ttab_x, ttab_y

    # Decks inside an enclosure: reached ONLY by the stage->insert slide
    # through the front slot (both acquiring and depositing). Open decks
    # (table) use direct approach + vertical lift/descend.
    ENCLOSED_DECKS = {"pxrd_sample"}
    src_enclosed = args.source in ENCLOSED_DECKS
    tgt_enclosed = args.target in ENCLOSED_DECKS
    # Vertical pinch offset for the enclosed acquire (see acquire comment):
    # the paddle is clamped 4mm above center; the deposit releases 4mm
    # higher so the plate still lands flush.
    grab_dz = 0.004 if (args.grasp_mode == "side" and src_enclosed) else 0.0

    # Grasp orientation.
    #  side: horizontal pinch — approach axis along insert_v, jaws vertical.
    #  top/center: gripper-down, optionally yawed (--grasp-yaw-deg).
    if args.grasp_mode == "side":
        grasp_q = side_grasp_q(src_v)          # acquire orientation
        deposit_q = side_grasp_q(insert_v)     # deposit orientation (same here;
                                               # decks share world orientation)
        node.get_logger().info(
            "Side grasp: gripper horizontal along insert axis, jaws vertical, "
            f"pinch height deck+{TAB_PINCH_Y*1000:.0f}mm")
    else:
        grasp_q = _quat_mul(_rz_quat(args.grasp_yaw_deg), DOWN_Q)
        deposit_q = grasp_q

    # ============================================================
    # PHASE: open gripper  (OMPL joint goal; trivial)
    # side mode opens only to SIDE_JAW_GAP_MM so the lower jaw never dips
    # into the table while sliding around the raised paddle.
    # ============================================================
    if args.grasp_mode == "side":
        # Narrower opening when diving into the enclosed bay (see acquire
        # comment): 30mm clears the shelf under the open lower jaw tip.
        open_val = grip_mm_to_joint(
            ENCLOSED_JAW_GAP_MM if src_enclosed else SIDE_JAW_GAP_MM)
    else:
        open_val = GRIPPER_OPEN
    if args.no_gripper:
        node.get_logger().warning(
            "--no-gripper: SKIPPING 'open gripper' phase (hardware arm-only)")
    else:
        if not _run_phase(node, phases, "open gripper", "OMPL",
                          lambda: plan_gripper(node, mg, open_val),
                          target_joint=open_val):
            fail("open gripper failed")

    if args.grasp_mode == "side":
        # ============================================================
        # SIDE PICK — two shapes:
        #  open deck (table): approach BEHIND the paddle at pinch height,
        #      short horizontal advance so the paddle enters the open jaws.
        #  enclosed deck (pxrd): stage OUTSIDE the slot (two-hop), then a
        #      long horizontal insert with empty jaws until the paddle sits
        #      between the pads — the reverse of the deposit slide.
        # ============================================================
        vx, vy = src_v
        pad = PAD_OFFSET_FROM_TCP
        pz = sz + TAB_PINCH_Y
        adv_x, adv_y = sx - vx * pad, sy - vy * pad
        if src_enclosed:
            app_dist = args.insert_depth
        else:
            app_dist = SIDE_STANDOFF
        app_x, app_y = adv_x - vx * app_dist, adv_y - vy * app_dist

        if src_enclosed:
            if not _run_phase(node, phases, f"stage outside {args.source}", "OMPL+IK",
                              lambda: plan_side_stage(
                                  node, mg, ik_client, get_scene_client,
                                  app_x, app_y, pz, grasp_q,
                                  vel=args.vel, planning_time=args.transit_time,
                                  planning_attempts=args.num_attempts,
                                  planner_id=args.planner, exec_client=exec_client,
                                  hint=STAGE_HINTS.get(args.source)),
                              target=f"({app_x:+.3f}, {app_y:+.3f}, {pz:+.3f}) horizontal",
                              method="3-hop staged, branch-pinned",
                              note=f"{app_dist*1000:.0f}mm outside the opening, "
                                   f"empty jaws at pinch height"):
                fail("stage-outside (acquire) failed")
        else:
            # Seed the pick IK with THIS deck's known-good family (the one
            # that is flat-ladder-connectable to the bay family). Seeding
            # with a far-away pose's config lets the distance metric pick
            # the wrong elbow sign.
            pick_hint = STAGE_HINTS.get(args.source)
            if not _run_phase(node, phases, f"approach behind {args.source} tab", "OMPL+IK",
                              lambda: plan_arm_to_pose_via_ik(
                                  node, mg, ik_client, get_scene_client,
                                  "base_link", "gripper_grasp_link",
                                  app_x, app_y, pz,
                                  grasp_q, vel=args.vel,
                                  planning_time=args.planning_time,
                                  planner_id=args.planner,
                                  planning_attempts=args.num_attempts,
                                  exec_client=exec_client,
                                  hint=pick_hint),
                              target=f"({app_x:+.3f}, {app_y:+.3f}, {pz:+.3f}) horizontal",
                              note=f"{SIDE_STANDOFF*1000:.0f}mm behind paddle, "
                                   f"pinch height deck+{TAB_PINCH_Y*1000:.0f}mm"
                                   + (", hint-seeded to carry family" if pick_hint else ""),
                              ompl=f"{args.planner}, {args.planning_time:.0f}s × {args.num_attempts}"):
                fail("side approach failed")

        # Enclosed acquire — the AG-145's open-jaw envelope only clears the
        # structure under the v2 paddle at pinch+4mm and above (measured:
        # slide at +4 clean, any descend below +2 collides). So: 30mm jaws,
        # slide in 4mm HIGH, and PINCH THERE — no seat-down. A force gripper
        # clamps the 20mm paddle full-face with a 4mm vertical offset; the
        # rigid TF attach keeps the sim honest, and the deposit compensates
        # by releasing 4mm higher (grab_dz). A +10mm taller riser in CAD
        # would make this centered and comfortable.
        adv_z = pz + grab_dz
        if not _run_phase(node, phases, f"advance to pinch at {args.source}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              adv_x, adv_y, adv_z,
                              grasp_q, vel=args.vel, max_step=0.005),
                          slide=f"{app_dist*1000:.0f}mm forward along "
                                f"({vx:+.2f}, {vy:+.2f})"
                                + (f" (+{grab_dz*1000:.0f}mm high pinch)" if grab_dz else ""),
                          max_step="5 mm"):
            fail("advance failed")
    else:
        # ============================================================
        # PHASE: approach above source + vertical descend (top/center)
        # ============================================================
        if not _run_phase(node, phases, f"approach above {args.source}", "OMPL+IK",
                          lambda: plan_arm_to_pose_via_ik(
                              node, mg, ik_client, get_scene_client,
                              "base_link", "gripper_grasp_link",
                              sx, sy, sz + args.hover,
                              grasp_q, vel=args.vel,
                              planning_time=args.planning_time,
                              planner_id=args.planner,
                              planning_attempts=args.num_attempts,
                              exec_client=exec_client),
                          target=f"({sx:+.3f}, {sy:+.3f}, {sz+args.hover:+.3f}) gripper-down",
                          method="IK-seeded joint goal (avoids long-way swings)",
                          ompl=f"{args.planner}, {args.planning_time:.0f}s × {args.num_attempts}",
                          budget=f"≤{MAX_TRAVEL_PER_OMPL_PHASE:.1f}rad total, "
                                 f"≤{MAX_SINGLE_JOINT_TRAVEL:.1f}rad per joint"):
            fail("approach failed")

        if not _run_phase(node, phases, f"descend to grasp at {args.source}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              sx, sy, sz + pick_offset,
                              grasp_q, vel=args.vel, max_step=0.005),
                          from_z=f"{sz+args.hover:+.3f}",
                          to_z=f"{sz+pick_offset:+.3f}",
                          pick_lift=f"+{args.pick_lift*1000:.0f}mm above place height",
                          max_step="5 mm"):
            fail("descend failed")

    # ============================================================
    # PHASE: close gripper  (OMPL joint goal)
    # ============================================================
    if args.no_gripper:
        node.get_logger().warning(
            "--no-gripper: SKIPPING 'close gripper' phase (hardware arm-only)")
    else:
        if not _run_phase(node, phases, "close gripper", "OMPL",
                          lambda: plan_gripper(node, mg, args.grip_value),
                          target_joint=args.grip_value):
            fail("close gripper failed")

    # ============================================================
    # PHASE: attach plate to gripper  (scene-diff)
    # ============================================================
    if not args.no_plate:
        if not _run_phase(node, phases, "attach plate to gripper", "scene-diff",
                          lambda: (attach_plate_to_gripper(
                              node, scene_client, plate_parent,
                              buf=buf, source_deck=args.source,
                              seat_tab=(args.grasp_mode == "top")), "ok"),
                          from_parent=plate_parent,
                          to_parent="gripper_grasp_link"):
            fail("attach-to-gripper failed")
        plate_parent = "gripper_grasp_link"

    # ============================================================
    # PHASE: extract the held plate from the source.
    #  side/open (table): vertical lift to carry height — the later stage
    #      move becomes a level swing, which OMPL connects reliably.
    #  side/enclosed (pxrd): micro-lift off the shelf, then slide back OUT
    #      through the slot carrying the plate (reverse of the deposit).
    #  top/center: vertical lift to hover.
    # ============================================================
    if args.grasp_mode == "side" and src_enclosed:
        vx, vy = src_v
        adv_x = sx - vx * PAD_OFFSET_FROM_TCP
        adv_y = sy - vy * PAD_OFFSET_FROM_TCP
        out_x = adv_x - vx * args.insert_depth
        out_y = adv_y - vy * args.insert_depth
        z_carry = sz + TAB_PINCH_Y + args.place_lift + args.pre_release_lift
        if not _run_phase(node, phases, "micro-lift off shelf", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              adv_x, adv_y, z_carry,
                              grasp_q, vel=args.vel, max_step=0.005),
                          from_z=f"{sz+TAB_PINCH_Y:+.3f}",
                          to_z=f"{z_carry:+.3f}",
                          max_step="5 mm"):
            fail("micro-lift failed")
        if not _run_phase(node, phases, f"retract out of {args.source} (carrying)", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              out_x, out_y, z_carry,
                              grasp_q, vel=args.vel, max_step=0.005),
                          slide=f"{args.insert_depth*1000:.0f}mm back along "
                                f"({-vx:+.2f}, {-vy:+.2f}), wells slide out",
                          max_step="5 mm"):
            fail("carry-out failed")
    else:
        if args.grasp_mode == "side":
            lift_x = sx - src_v[0] * PAD_OFFSET_FROM_TCP
            lift_y = sy - src_v[1] * PAD_OFFSET_FROM_TCP
            lift_from = sz + TAB_PINCH_Y
            # Lift straight up to the TARGET's insert height: the subsequent
            # stage move becomes a level swing (no simultaneous climb).
            lift_to = max(sz + TAB_PINCH_Y + 0.080,
                          tz + TAB_PINCH_Y + args.place_lift + args.pre_release_lift)
        else:
            lift_x, lift_y = sx, sy
            lift_from = sz + pick_offset
            lift_to = sz + args.hover
        if not _run_phase(node, phases, f"lift above {args.source}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              lift_x, lift_y, lift_to,
                              grasp_q, vel=args.vel, max_step=0.005),
                          from_z=f"{lift_from:+.3f}",
                          to_z=f"{lift_to:+.3f}",
                          max_step="5 mm"):
            fail("lift failed")

    # ============================================================
    # PICK-ONLY: stop here, holding the plate. The plate stays ATTACHED in
    # the move_group planning scene, so you can now teach the reach-in path
    # incrementally with `ros2 run pxrd_cell goto_joints --joints=...` —
    # each move carries the plate with full collision checking.
    # ============================================================
    if args.pick_only:
        _print_summary(node, phases)
        node.get_logger().info(
            "PICK-ONLY done — plate held & attached. Teach next configs with "
            "goto_joints; the plate moves with the gripper in the scene.")
        rclpy.shutdown()
        return

    # ============================================================
    # PHASE: via-point at home  (with plate held)
    # ------------------------------------------------------------
    # Industrial-canonical "via point" pattern: instead of trying to plan
    # one big motion from above-source straight to above-target, fold the
    # arm back to the home/ready pose first. Two short, clean motions are
    # much easier to plan smoothly than one long swing through awkward
    # joint configs.
    # Gripper stays pointing down throughout, plate held horizontal.
    # ============================================================
    # First always fold back to HOME — a known neutral config. This is the
    # SAME starting point you use when posing a staging pose in rviz, so the
    # next move (home -> staging) follows the simple path you saw there,
    # instead of an awkward direct route from the lifted-pick config.
    # SIDE mode skips this: HOME points the gripper down, which would aim the
    # side-pinched plate straight down (long lever near the pedestal). The
    # side carry goes lift -> stage directly, keeping the plate horizontal.
    if args.grasp_mode != "side":
        if not _run_phase(node, phases, "via home (carrying plate)", "OMPL",
                          lambda: plan_arm_to_joints(
                              node, mg, HOME_JOINTS, vel=args.vel,
                              planning_time=args.planning_time,
                              planning_attempts=args.num_attempts,
                              planner_id=args.planner,
                              exec_client=exec_client),
                          via=("home: " + ", ".join(
                              f"{k}={v:+.2f}" for k, v in HOME_JOINTS.items())),
                          note="neutral config; same start as when the staging pose was found"):
            fail("via-home failed")

    # Then, if a custom staging pose was given (--via-joints), go there from
    # home to pre-orient the arm toward the target opening before reaching in.
    if args.via_joints.strip():
        if not _run_phase(node, phases, "via staging pose (carrying plate)", "OMPL",
                          lambda: plan_arm_to_joints(
                              node, mg, via_joints, vel=args.vel,
                              planning_time=args.planning_time,
                              planning_attempts=args.num_attempts,
                              planner_id=args.planner,
                              exec_client=exec_client),
                          via=("staging: " + ", ".join(
                              f"{k}={v:+.2f}" for k, v in via_joints.items())),
                          note="from HOME -> staging pose (clean, like in rviz)"):
            fail("via staging pose failed")

    if args.grasp_mode == "side" and not tgt_enclosed and insert_v is not None:
        # ============================================================
        # SIDE DEPOSIT on an OPEN deck (table): carry over the drop point
        # at carry height (two-hop), vertical descend until the plate
        # rests on the deck, release, back off horizontally.
        # ============================================================
        vx, vy = insert_v
        t_adv_x = tx - vx * PAD_OFFSET_FROM_TCP
        t_adv_y = ty - vy * PAD_OFFSET_FROM_TCP
        t_pz = tz + TAB_PINCH_Y + grab_dz   # release higher by the pinch offset
        t_carry = t_pz + 0.080

        # FLAT CARRY first: acquire and deposit share one orientation (both
        # decks' tabs point the same way), so the whole carry can be one
        # straight orientation-locked Cartesian segment — the plate stays
        # level by construction. The tilt-gated ladder is the fallback.
        qd = abs(grasp_q.x * deposit_q.x + grasp_q.y * deposit_q.y
                 + grasp_q.z * deposit_q.z + grasp_q.w * deposit_q.w)
        glided = (not args.no_glide and qd > 0.9999
                  and attempt_glide(f"flat glide above {args.target}",
                                    t_adv_x, t_adv_y, t_carry, deposit_q))
        if not glided:
            if not _run_phase(node, phases, f"carry above {args.target}", "OMPL+IK",
                              lambda: plan_side_stage(
                                  node, mg, ik_client, get_scene_client,
                                  t_adv_x, t_adv_y, t_carry, deposit_q,
                                  vel=args.vel, planning_time=args.transit_time,
                                  planning_attempts=args.num_attempts,
                                  planner_id=args.planner, exec_client=exec_client,
                                  # Seed the carry target IK at the deck's
                                  # known flat-ladder-compatible family (same
                                  # elbow/j6 signs as the bay carry family).
                                  hint=STAGE_HINTS.get(args.target),
                                  j1_first=src_enclosed,
                                  tilt_gate=carry_gate,
                                  source=args.source, target=args.target),
                              target=f"({t_adv_x:+.3f}, {t_adv_y:+.3f}, {t_carry:+.3f}) horizontal",
                              method="staged ladder, current-family-seeded, "
                                     "j1-first from bay, tilt-gated"):
                fail("carry-above failed")
        if not _run_phase(node, phases, f"descend to place at {args.target}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              t_adv_x, t_adv_y, t_pz,
                              deposit_q, vel=args.vel, max_step=0.005),
                          from_z=f"{t_carry:+.3f}",
                          to_z=f"{t_pz:+.3f}",
                          note="plate bottom lands on the deck",
                          max_step="5 mm"):
            fail("place descend failed")
    elif args.grasp_mode in ("top", "side") and insert_v is not None:
        # ============================================================
        # INSERTION PLACE: through-the-opening slide.
        # Hover-above-then-descend can't work on an enclosed target — the
        # wrist would have to be above the sample, INSIDE the enclosure.
        # Instead: stage outside the opening at insert height, slide the
        # plate straight in along the tab->wells axis, short set-down.
        #  top : TCP above the tab (wrist outside, vertical stack)
        #  side: TCP behind the paddle (whole chain horizontal, through
        #        the front slot — the mode the PXRD bay geometry demands)
        # ============================================================
        vx, vy = insert_v
        if args.grasp_mode == "side":
            # TCP rides PAD_OFFSET behind the paddle along the insert axis
            in_x = tx - vx * PAD_OFFSET_FROM_TCP
            in_y = ty - vy * PAD_OFFSET_FROM_TCP
            z_set = tz + TAB_PINCH_Y + args.place_lift
        else:
            in_x, in_y = tx, ty
            z_set = tz + place_offset
        stage_x = in_x - vx * args.insert_depth
        stage_y = in_y - vy * args.insert_depth
        z_ins = z_set + args.pre_release_lift   # slide slightly high

        glided = False
        if args.grasp_mode == "side" and not args.no_glide:
            # FLAT CARRY first (see the deposit-side comment): straight
            # orientation-locked segment from the post-lift pose to the stage
            # point outside the opening. Plate stays level by construction.
            qd = abs(grasp_q.x * deposit_q.x + grasp_q.y * deposit_q.y
                     + grasp_q.z * deposit_q.z + grasp_q.w * deposit_q.w)
            if qd > 0.9999:
                glided = attempt_glide(
                    f"flat glide to stage outside {args.target}",
                    stage_x, stage_y, z_ins, deposit_q)
        if glided:
            stage_planner = None
        elif args.grasp_mode == "side":
            stage_planner = lambda: plan_side_stage(
                node, mg, ik_client, get_scene_client,
                stage_x, stage_y, z_ins, deposit_q,
                vel=args.vel, planning_time=args.transit_time,
                planning_attempts=args.num_attempts,
                planner_id=args.planner, exec_client=exec_client,
                hint=STAGE_HINTS.get(args.target),
                tilt_gate=carry_gate,
                source=args.source, target=args.target)
            stage_method = "taught replay if available, else gated ladder"
        else:
            stage_planner = lambda: plan_arm_to_pose_via_ik(
                node, mg, ik_client, get_scene_client,
                "base_link", "gripper_grasp_link",
                stage_x, stage_y, z_ins,
                grasp_q, vel=args.vel,
                planning_time=args.transit_time,
                planner_id=args.planner,
                planning_attempts=args.num_attempts,
                exec_client=exec_client,
                keep_orientation_along_path=args.keep_plate_flat,
                path_tilt_tol=args.path_tilt_tol)
            stage_method = "IK-seeded joint goal"
        if stage_planner is not None:
            if not _run_phase(node, phases, f"stage outside {args.target}", "OMPL+IK",
                              stage_planner,
                              target=f"({stage_x:+.3f}, {stage_y:+.3f}, {z_ins:+.3f}) "
                                     + ("horizontal" if args.grasp_mode == "side" else "gripper-down"),
                              method=stage_method,
                              note=f"{args.insert_depth*1000:.0f}mm outside the "
                                   f"opening, at insert height (free space)",
                              ompl=f"{args.planner}, {args.transit_time:.0f}s × {args.num_attempts}"):
                fail("stage-outside failed")


        if not _run_phase(node, phases, f"insert into {args.target}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              in_x, in_y, z_ins,
                              grasp_q, vel=args.vel, max_step=0.005),
                          slide=f"{args.insert_depth*1000:.0f}mm along "
                                f"({vx:+.2f}, {vy:+.2f}), constant z={z_ins:+.3f}",
                          note="wells slide in over the sample; wrist stays outside",
                          max_step="5 mm"):
            fail("insert failed — check fraction: how far in it got before "
                 "hitting the enclosure")

        if not _run_phase(node, phases, "set down", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              in_x, in_y, z_set,
                              grasp_q, vel=args.vel, max_step=0.005),
                          from_z=f"{z_ins:+.3f}",
                          to_z=f"{z_set:+.3f}",
                          max_step="5 mm"):
            fail("set-down failed")
    else:
        # ============================================================
        # LEGACY (center grasp): transit above target, vertical descend.
        # ============================================================
        if not _run_phase(node, phases, f"transit above {args.target}", "OMPL+IK",
                          lambda: plan_arm_to_pose_via_ik(
                              node, mg, ik_client, get_scene_client,
                              "base_link", "gripper_grasp_link",
                              tx, ty, tz + args.hover,
                              grasp_q, vel=args.vel,
                              planning_time=args.transit_time,
                              planner_id=args.planner,
                              planning_attempts=args.num_attempts,
                              exec_client=exec_client,
                              keep_orientation_along_path=args.keep_plate_flat,
                              path_tilt_tol=args.path_tilt_tol),
                          target=f"({tx:+.3f}, {ty:+.3f}, {tz+args.hover:+.3f}) gripper-down",
                          method="IK-seeded joint goal"
                                 + (" + plate-flat path" if args.keep_plate_flat else " (no long swing)"),
                          ompl=f"{args.planner}, {args.transit_time:.0f}s × {args.num_attempts}",
                          budget=f"≤{MAX_TRAVEL_PER_OMPL_PHASE:.1f}rad total, "
                                 f"≤{MAX_SINGLE_JOINT_TRAVEL:.1f}rad per joint"):
            fail("transit failed")

        if not _run_phase(node, phases, f"descend to place at {args.target}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              tx, ty, tz + place_offset,
                              grasp_q, vel=args.vel, max_step=0.005),
                          from_z=f"{tz+args.hover:+.3f}",
                          to_z=f"{tz+place_offset:+.3f}",
                          max_step="5 mm"):
            fail("place descend failed")

        if not _run_phase(node, phases, "pre-release lift", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              tx, ty, tz + place_offset + args.pre_release_lift,
                              grasp_q, vel=args.vel, max_step=0.005),
                          from_z=f"{tz+place_offset:+.3f}",
                          to_z=f"{tz+place_offset+args.pre_release_lift:+.3f}",
                          max_step="5 mm"):
            fail("pre-release lift failed")

    # ============================================================
    # PHASE: open gripper (release)  (OMPL joint goal)
    # side mode opens only to SIDE_JAW_GAP_MM — a full 145mm swing would
    # drive the lower jaw into the bay floor / table.
    # ============================================================
    release_val = (grip_mm_to_joint(SIDE_JAW_GAP_MM)
                   if args.grasp_mode == "side" else GRIPPER_OPEN)
    if args.no_gripper:
        node.get_logger().warning(
            "--no-gripper: SKIPPING 'open gripper (release)' phase")
    else:
        if not _run_phase(node, phases, "open gripper (release)", "OMPL",
                          lambda: plan_gripper(node, mg, release_val),
                          target_joint=release_val):
            fail("release failed")

    # ============================================================
    # PHASE: re-attach plate to target deck  (scene-diff)
    # ============================================================
    if not args.no_plate:
        if not _run_phase(node, phases, f"re-attach plate to {args.target}", "scene-diff",
                          lambda: (attach_plate_to_deck(
                              node, scene_client, args.target, plate_parent),
                              "ok"),
                          from_parent=plate_parent,
                          to_parent=args.target):
            fail("re-attach failed")
        plate_parent = args.target

    if args.grasp_mode == "side" and not tgt_enclosed and insert_v is not None:
        # ============================================================
        # SIDE BACK-OFF on an open deck: slide the open jaws horizontally
        # off the paddle at pinch height (SIDE_STANDOFF is plenty).
        # ============================================================
        vx, vy = insert_v
        t_adv_x = tx - vx * PAD_OFFSET_FROM_TCP
        t_adv_y = ty - vy * PAD_OFFSET_FROM_TCP
        t_pz = tz + TAB_PINCH_Y + grab_dz
        if not _run_phase(node, phases, f"back off from {args.target}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              t_adv_x - vx * SIDE_STANDOFF,
                              t_adv_y - vy * SIDE_STANDOFF, t_pz,
                              deposit_q, vel=args.vel, max_step=0.005),
                          slide=f"{SIDE_STANDOFF*1000:.0f}mm back along "
                                f"({-vx:+.2f}, {-vy:+.2f})",
                          max_step="5 mm"):
            fail("back-off failed")
    elif args.grasp_mode in ("top", "side") and insert_v is not None:
        # ============================================================
        # INSERTION RETREAT: slide back OUT along the insert axis
        # (reverse of the way in), never entering the enclosure.
        #  top : lift fingertips off the tab first, then retract.
        #  side: retract straight back at set-down height — the open jaws
        #        slide off the paddle horizontally, like letting go of a
        #        tray. No lift (the paddle sits between the jaws).
        # ============================================================
        vx, vy = insert_v
        if args.grasp_mode == "side":
            in_x = tx - vx * PAD_OFFSET_FROM_TCP
            in_y = ty - vy * PAD_OFFSET_FROM_TCP
            z_set = tz + TAB_PINCH_Y + args.place_lift
            z_ret = z_set                     # no lift-off in side mode
        else:
            in_x, in_y = tx, ty
            z_set = tz + place_offset
            z_ret = z_set + args.pre_release_lift
        stage_x = in_x - vx * args.insert_depth
        stage_y = in_y - vy * args.insert_depth
        if args.grasp_mode == "top":
            if not _run_phase(node, phases, "lift off tab", "Cartesian",
                              lambda: cartesian_move(
                                  node, cart_client, exec_client,
                                  "base_link", "gripper_grasp_link",
                                  in_x, in_y, z_ret,
                                  grasp_q, vel=args.vel, max_step=0.005),
                              from_z=f"{z_set:+.3f}",
                              to_z=f"{z_ret:+.3f}",
                              max_step="5 mm"):
                fail("lift-off failed")
        if not _run_phase(node, phases, f"retract out of {args.target}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              stage_x, stage_y, z_ret,
                              grasp_q, vel=args.vel, max_step=0.005),
                          slide=f"{args.insert_depth*1000:.0f}mm back along "
                                f"({-vx:+.2f}, {-vy:+.2f})",
                          max_step="5 mm"):
            fail("retract failed")
    else:
        # ============================================================
        # PHASE: retreat  (Cartesian — straight vertical)
        # ============================================================
        if not _run_phase(node, phases, f"retreat above {args.target}", "Cartesian",
                          lambda: cartesian_move(
                              node, cart_client, exec_client,
                              "base_link", "gripper_grasp_link",
                              tx, ty, tz + args.hover,
                              grasp_q, vel=args.vel, max_step=0.005),
                          from_z=f"{tz+place_offset:+.3f}",
                          to_z=f"{tz+args.hover:+.3f}",
                          max_step="5 mm"):
            fail("retreat failed")

    # ============================================================
    # PHASE: exit the instrument  (reverse out: staging -> home)
    # The gripper is empty now. Pull straight back out through the SAME
    # staging pose used to reach in, then fold to HOME — so the arm doesn't
    # end deep in the enclosure with the elbow up against the ceiling.
    # ============================================================
    if not args.no_return_home:
        if args.via_joints.strip():
            if not _run_phase(node, phases, "exit via staging pose", "OMPL",
                              lambda: plan_arm_to_joints(node, mg, via_joints,
                                  vel=args.vel, planning_time=args.planning_time,
                                  planning_attempts=args.num_attempts,
                                  planner_id=args.planner, exec_client=exec_client),
                              via=("staging: " + ", ".join(
                                  f"{k}={v:+.2f}" for k, v in via_joints.items())),
                              note="reverse out of the enclosure (empty gripper)"):
                fail("exit via staging failed")
        # After an enclosed-target deposit the retract ends right at the bay
        # mouth, where home plans sometimes fail instantly (near-grazing
        # start state). Best-effort: pull back a further SIDE_STANDOFF along
        # the corridor first. Non-fatal — reach may run out here.
        if args.grasp_mode == "side" and tgt_enclosed and insert_v is not None:
            vx, vy = insert_v
            tf_now = lookup_tf(node, buf, "base_link", "gripper_grasp_link")
            gx0, gy0, gz0 = (tf_now.transform.translation.x,
                             tf_now.transform.translation.y,
                             tf_now.transform.translation.z)
            ok_pull, _, _, info_pull = cartesian_move(
                node, cart_client, exec_client,
                "base_link", "gripper_grasp_link",
                gx0 - vx * SIDE_STANDOFF, gy0 - vy * SIDE_STANDOFF, gz0,
                deposit_q, vel=args.vel, max_step=0.005)
            node.get_logger().info(
                f"   clearance pull ({SIDE_STANDOFF*1000:.0f}mm): "
                f"{'ok' if ok_pull else 'skipped — ' + str(info_pull)}")
        if not _run_phase(node, phases, "return to home", "OMPL",
                          lambda: plan_arm_to_joints(node, mg, HOME_JOINTS,
                              vel=args.vel,
                              planning_time=max(args.planning_time, 15.0),
                              planning_attempts=args.num_attempts,
                              planner_id=args.planner, exec_client=exec_client),
                          target_joints=", ".join(
                              f"{k}={v:+.2f}" for k, v in HOME_JOINTS.items()),
                          note="neutral config, clear of the instrument"):
            fail("return to home failed")

    _print_summary(node, phases)
    node.get_logger().info("DONE")
    rclpy.shutdown()


if __name__ == "__main__":
    main()

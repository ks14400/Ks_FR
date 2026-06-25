"""
SDK Executor — Python bridge between MoveIt2 and the real Fairino robot.

Bypasses the buggy fairino_hardware C++ plugin by:
  1. Subscribing to MoveIt2's FollowJointTrajectory action
  2. Publishing /joint_states at 50Hz by polling the SDK
  3. Executing each trajectory point via SDK MoveJ or ServoJ

This lets you use rviz2's Plan/Execute workflow on the real robot
without the checksum/protocol issues in the C++ plugin.

Cell-agnostic: the same bridge serves any cell (unchained, pxrd, ...). It
knows nothing about decks/scenes — it only forwards joint trajectories and
gripper commands to the robot. Launch it via fr_test_cell/launch/bridge.launch.py.

Usage:
    ros2 run fr_bridge sdk_executor --ros-args \
        -p robot_ip:=192.168.58.2 \
        -p controller_name:=fairino16_controller
"""
import math
import os
import sys
import time
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from builtin_interfaces.msg import Time

# Import Fairino SDK
_sdk_path = os.path.expanduser("~/Ks_FR/linux")
if _sdk_path not in sys.path:
    sys.path.insert(0, _sdk_path)
from fairino import Robot


JOINT_NAMES = ["j1", "j2", "j3", "j4", "j5", "j6"]

# ============================================================
# Gripper (DH AG-145 on the FR16 tool RS-485 port)
# ============================================================
# The gripper is commanded through the SAME Robot.RPC connection as the arm,
# via SDK MoveGripper/GetGripperMotionDone. MoveIt plans the gripper as a
# joint ("gripper_finger1_joint", 0.0=open .. -0.65=closed per URDF); we map
# that to the SDK's percentage scale (100=open .. 0=closed).
GRIPPER_JOINT_NAME = "gripper_finger1_joint"
GRIPPER_JOINT_CLOSED_RAD = -0.65   # URDF lower limit = fully closed
GRIPPER_STROKE_MM = 145.0          # jaw opening at 100% (must match pick_place)
GRIPPER_INDEX = 1
GRIPPER_COMPANY_DAHUAN = 4
GRIPPER_DEVICE_PGI140 = 0


def gripper_rad_to_pct(joint_rad):
    """URDF joint value (0.0 open .. -0.65 closed) → SDK pct (100 open .. 0).

    Empirically verified (AG-145, SDK V2.1.6, Robot V3.8.6): command and
    steady-state feedback use the SAME scale (100=open, 0=closed). Caveat:
    GetGripperCurPosition transiently reads 0 right after a MoveGripper
    command is issued — completion checks must debounce (see
    _wait_gripper_done)."""
    pct = (1.0 - joint_rad / GRIPPER_JOINT_CLOSED_RAD) * 100.0
    return max(0.0, min(100.0, pct))


def gripper_pct_to_rad(pct):
    """SDK pct (100 open .. 0 closed) → URDF joint value (0.0 .. -0.65)."""
    return (1.0 - pct / 100.0) * GRIPPER_JOINT_CLOSED_RAD

# ============================================================
# Trajectory budget gate — defense-in-depth at the bridge layer
# ============================================================
# These are the same kind of checks pick_place.py applies BEFORE sending.
# Replicating them here means ANY trajectory arriving via the action server
# (from any source, including future tools or accidents) gets filtered.
# Tuned slightly more generous than pick_place's caps so legitimate plans
# don't get double-rejected.
BRIDGE_MAX_TOTAL_TRAVEL_RAD = 8.0   # sum of |Δjoint| across whole trajectory
BRIDGE_MAX_SINGLE_JOINT_RAD = 4.0   # any one joint's total travel
BRIDGE_MAX_STEP_RAD         = 0.15  # max |Δjoint| between consecutive waypoints


def _trajectory_stats(points, joint_order):
    """Compute joint-travel stats from a JointTrajectory.points list.
    Returns dict with total_travel, max_single_joint_travel, max_step, worst_joint."""
    if len(points) < 2:
        return {"total_travel": 0.0, "max_single_joint_travel": 0.0,
                "max_step": 0.0, "worst_joint": "-"}
    n_joints = len(points[0].positions)
    per_joint = [0.0] * n_joints
    max_step = 0.0
    for i in range(1, len(points)):
        for j in range(n_joints):
            d = abs(points[i].positions[j] - points[i - 1].positions[j])
            per_joint[j] += d
            if d > max_step:
                max_step = d
    worst_idx = max(range(n_joints), key=lambda i: per_joint[i])
    worst_name = (joint_order[worst_idx]
                  if worst_idx < len(joint_order) else f"joint{worst_idx}")
    return {
        "total_travel": sum(per_joint),
        "max_single_joint_travel": per_joint[worst_idx],
        "max_step": max_step,
        "worst_joint": worst_name,
    }


class SDKExecutorNode(Node):
    def __init__(self):
        super().__init__("sdk_executor")

        self.declare_parameter("robot_ip", "192.168.58.2")
        self.declare_parameter("controller_name", "fairino16_controller")
        self.declare_parameter("joint_state_rate_hz", 50.0)
        self.declare_parameter("movej_vel_pct", 10.0)  # 10% of max velocity
        self.declare_parameter("movej_acc_pct", 20.0)
        self.declare_parameter("use_servoj", False)  # False=MoveJ (safer), True=ServoJ (faster)
        # Gripper (AG-145 via tool RS-485)
        self.declare_parameter("gripper_enable", True)
        self.declare_parameter("gripper_controller_name", "gripper_controller")
        self.declare_parameter("gripper_vel_pct", 30.0)
        self.declare_parameter("gripper_force_pct", 20.0)
        self.declare_parameter("gripper_timeout_ms", 5000)

        self.robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        controller_name = self.get_parameter("controller_name").get_parameter_value().string_value
        self.js_rate = self.get_parameter("joint_state_rate_hz").get_parameter_value().double_value
        self.movej_vel = self.get_parameter("movej_vel_pct").get_parameter_value().double_value
        self.movej_acc = self.get_parameter("movej_acc_pct").get_parameter_value().double_value
        self.use_servoj = self.get_parameter("use_servoj").get_parameter_value().bool_value
        self.gripper_enable = self.get_parameter("gripper_enable").get_parameter_value().bool_value
        gripper_controller_name = self.get_parameter(
            "gripper_controller_name").get_parameter_value().string_value
        self.gripper_vel = self.get_parameter("gripper_vel_pct").get_parameter_value().double_value
        self.gripper_force = self.get_parameter("gripper_force_pct").get_parameter_value().double_value
        self.gripper_timeout_ms = self.get_parameter(
            "gripper_timeout_ms").get_parameter_value().integer_value

        self.get_logger().info(f"Connecting to robot at {self.robot_ip}...")
        self.robot = Robot.RPC(self.robot_ip)

        # Verify connection. On timeout, GetSDKVersion() returns a plain int
        # error code instead of the expected (code, version_str) tuple.
        ver = self.robot.GetSDKVersion()
        if not isinstance(ver, (tuple, list)) or len(ver) < 2 or ver[0] != 0:
            self.get_logger().fatal(
                f"Cannot connect to robot at {self.robot_ip}. "
                f"GetSDKVersion returned: {ver!r}. "
                f"Likely causes: (1) robot powered off; (2) network unreachable "
                f"(ping {self.robot_ip}); (3) stale SDK session — power-cycle robot."
            )
            raise RuntimeError("Robot connection failed")
        self.get_logger().info(f"Connected. {ver[1]}")

        # Reset errors and enable
        err = self.robot.GetRobotErrorCode()
        if err[0] == 0 and err[1] != [0, 0]:
            self.get_logger().warn(f"Robot has errors {err[1]}, resetting...")
            self.robot.ResetAllError()
            time.sleep(0.5)
        self.robot.RobotEnable(1)
        time.sleep(0.5)

        self._robot_lock = threading.Lock()  # serialize SDK calls

        # /joint_states publisher
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        self._js_timer = self.create_timer(
            1.0 / self.js_rate, self._publish_joint_states
        )

        # FollowJointTrajectory action server
        # Namespace must match what MoveIt2 expects from moveit_controllers.yaml
        action_ns = f"/{controller_name}/follow_joint_trajectory"
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            action_ns,
            self._execute_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
        )
        self.get_logger().info(f"Action server ready at: {action_ns}")
        self.get_logger().info(
            f"Execution mode: {'ServoJ (streaming)' if self.use_servoj else 'MoveJ (per-waypoint)'}"
        )

        # /stop_motion service — emergency halt independent of MoveIt
        # The 'stop' CLI tool calls this for guaranteed hardware halt.
        # Returns immediately after issuing SDK StopMotion().
        self._stop_service = self.create_service(
            Trigger, "/stop_motion", self._stop_motion_callback,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("Service ready at: /stop_motion")

        # ───────────── Gripper (AG-145) ─────────────
        # Activation ritual + a second FollowJointTrajectory action server so
        # MoveIt's gripper_controller goals execute on the real gripper.
        # Cached state for /joint_states (polled at low rate to limit RPC load).
        self.gripper_ok = False
        self._gripper_joint_rad = 0.0   # last known position, URDF radians
        self._gripper_poll_counter = 0
        if self.gripper_enable:
            self.gripper_ok = self._init_gripper()
            gripper_ns = f"/{gripper_controller_name}/follow_joint_trajectory"
            self._gripper_action_server = ActionServer(
                self,
                FollowJointTrajectory,
                gripper_ns,
                self._execute_gripper_callback,
                callback_group=ReentrantCallbackGroup(),
                goal_callback=self._gripper_goal_callback,
                cancel_callback=self._cancel_callback,
            )
            self.get_logger().info(f"Gripper action server ready at: {gripper_ns}")
        else:
            self.get_logger().info("Gripper disabled (gripper_enable=false)")

    # ───────────── Gripper init ritual ─────────────
    def _init_gripper(self):
        """Configure + activate the AG-145 on the tool RS-485 port.
        Mirrors the proven sequence from scripts/test_gripper.py.
        Returns True if the gripper activated, False otherwise (arm still works)."""
        try:
            with self._robot_lock:
                # RS-485: port=7, 115200 8N1, per AG-145 wiring
                self.robot.SetAxleCommunicationParam(7, 8, 1, 0, 100, 3, 1)
                self.robot.SetGripperConfig(GRIPPER_COMPANY_DAHUAN,
                                            GRIPPER_DEVICE_PGI140)
            time.sleep(0.5)
            with self._robot_lock:
                self.robot.ActGripper(GRIPPER_INDEX, 0)   # reset
            time.sleep(1.5)
            with self._robot_lock:
                err = self.robot.ActGripper(GRIPPER_INDEX, 1)  # activate
            time.sleep(2.0)
            if err != 0:
                self.get_logger().error(
                    f"Gripper activation failed (ActGripper err={err}). "
                    "Arm remains usable; gripper goals will be rejected. "
                    "Check gripper power/wiring, then restart the bridge.")
                return False
            self.get_logger().info("Gripper activated (AG-145 on tool RS-485)")
            return True
        except Exception as e:
            self.get_logger().error(
                f"Gripper init raised: {e}. Arm remains usable; "
                "gripper goals will be rejected.")
            return False

    # ───────────── Joint State Publisher ─────────────
    def _publish_joint_states(self):
        with self._robot_lock:
            ret = self.robot.GetActualJointPosDegree(1)
        if ret[0] != 0:
            return
        joints_deg = ret[1]

        # Poll the gripper position at a reduced rate (~1/10th of joint rate)
        # to keep RPC load low; cache between polls. Published every cycle so
        # MoveIt's start-state validation always has a fresh-enough value.
        if self.gripper_ok:
            self._gripper_poll_counter += 1
            if self._gripper_poll_counter >= 10:
                self._gripper_poll_counter = 0
                with self._robot_lock:
                    g = self.robot.GetGripperCurPosition()
                # Expected (err, [fault, pos_pct]) or (err, fault, pos_pct)
                try:
                    if isinstance(g, (tuple, list)) and g[0] == 0:
                        payload = g[1]
                        if isinstance(payload, (tuple, list)):
                            pos_pct = payload[-1]
                        else:
                            pos_pct = g[-1]
                        self._gripper_joint_rad = gripper_pct_to_rad(
                            float(pos_pct))
                except (IndexError, TypeError, ValueError):
                    pass  # keep last cached value

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = [math.radians(j) for j in joints_deg]
        if self.gripper_ok:
            msg.name.append(GRIPPER_JOINT_NAME)
            msg.position.append(self._gripper_joint_rad)
        self._js_pub.publish(msg)

    # ───────────── Emergency Stop Service ─────────────
    def _stop_motion_callback(self, request, response):
        """Hard-halt the real robot. Called by the 'stop' CLI / pause script.
        Independent of MoveIt — works even if MoveIt is wedged."""
        self.get_logger().warning(">>> /stop_motion called — halting robot <<<")
        try:
            with self._robot_lock:
                self.robot.StopMotion()
            response.success = True
            response.message = "SDK StopMotion() succeeded"
        except Exception as e:
            response.success = False
            response.message = f"StopMotion() raised: {e}"
            self.get_logger().error(response.message)
        return response

    # ───────────── Action Server Callbacks ─────────────
    def _goal_callback(self, goal_request):
        traj = goal_request.trajectory
        n_pts = len(traj.points)
        self.get_logger().info(
            f"Trajectory goal received: {n_pts} points"
        )
        # Budget gate — reject obviously unsafe trajectories before motion.
        s = _trajectory_stats(traj.points, traj.joint_names)
        if s["total_travel"] > BRIDGE_MAX_TOTAL_TRAVEL_RAD:
            self.get_logger().error(
                f"REJECT goal: total joint travel {s['total_travel']:.2f} rad > "
                f"budget {BRIDGE_MAX_TOTAL_TRAVEL_RAD:.2f} rad")
            return GoalResponse.REJECT
        if s["max_single_joint_travel"] > BRIDGE_MAX_SINGLE_JOINT_RAD:
            self.get_logger().error(
                f"REJECT goal: {s['worst_joint']} travels "
                f"{s['max_single_joint_travel']:.2f} rad > budget "
                f"{BRIDGE_MAX_SINGLE_JOINT_RAD:.2f} rad (one joint swings too far)")
            return GoalResponse.REJECT
        if s["max_step"] > BRIDGE_MAX_STEP_RAD:
            self.get_logger().error(
                f"REJECT goal: max step {s['max_step']:.3f} rad > "
                f"budget {BRIDGE_MAX_STEP_RAD:.3f} rad (path too jerky)")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"  budget OK: travel={s['total_travel']:.2f}rad "
            f"({s['worst_joint']}={s['max_single_joint_travel']:.2f}), "
            f"maxStep={s['max_step']:.3f}rad")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().warn("Trajectory cancel requested")
        with self._robot_lock:
            self.robot.StopMotion()
        return CancelResponse.ACCEPT

    # ───────────── Gripper Action Server ─────────────
    def _gripper_goal_callback(self, goal_request):
        if not self.gripper_ok:
            self.get_logger().error(
                "REJECT gripper goal: gripper not activated "
                "(init failed at bridge startup — restart bridge after fixing)")
            return GoalResponse.REJECT
        traj = goal_request.trajectory
        if GRIPPER_JOINT_NAME not in traj.joint_names:
            self.get_logger().error(
                f"REJECT gripper goal: '{GRIPPER_JOINT_NAME}' not in "
                f"trajectory joints {list(traj.joint_names)}")
            return GoalResponse.REJECT
        self.get_logger().info(
            f"Gripper goal received: {len(traj.points)} points")
        return GoalResponse.ACCEPT

    def _execute_gripper_callback(self, goal_handle: ServerGoalHandle):
        """Execute a gripper trajectory: take the FINAL waypoint's target and
        send one MoveGripper command. Intermediate waypoints are irrelevant —
        the AG-145 does its own internal motion profile."""
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory
        idx = list(traj.joint_names).index(GRIPPER_JOINT_NAME)
        target_rad = traj.points[-1].positions[idx]
        target_pct = gripper_rad_to_pct(target_rad)

        # Read force/vel LIVE each grip so they can be tuned with
        #   ros2 param set /sdk_executor gripper_force_pct 35
        # without relaunching the bridge.
        vel = self.get_parameter("gripper_vel_pct").get_parameter_value().double_value
        force = self.get_parameter("gripper_force_pct").get_parameter_value().double_value

        target_mm = target_pct / 100.0 * GRIPPER_STROKE_MM
        self.get_logger().info(
            f"Gripper: joint={target_rad:+.3f} rad → {target_pct:.0f}% "
            f"≈ {target_mm:.0f}mm gap "
            f"(vel={vel:.0f}%, force={force:.0f}%)")

        with self._robot_lock:
            err = self.robot.MoveGripper(
                GRIPPER_INDEX, int(round(target_pct)),
                int(vel), int(force),
                int(self.gripper_timeout_ms), 0,
                0,   # PARALLEL
                0, 0, 0)
        if err != 0:
            self.get_logger().error(f"MoveGripper failed: code {err}")
            result.error_code = result.INVALID_GOAL
            goal_handle.abort()
            return result

        if not self._wait_gripper_done(
                target_pct, timeout_s=self.gripper_timeout_ms / 1000.0 + 2.0):
            # Timed out — report failure so pick_place can retry
            result.error_code = result.GOAL_TOLERANCE_VIOLATED
            goal_handle.abort()
            return result

        # Update cached joint value immediately so /joint_states is fresh
        self._gripper_joint_rad = target_rad
        self.get_logger().info("Gripper motion complete")
        result.error_code = result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def _wait_gripper_done(self, target_pct, timeout_s=7.0, tol_pct=5.0,
                           min_wait_s=0.6, n_confirm=3):
        """Wait for the gripper to reach target_pct by polling POSITION
        feedback (same scale as the command: 100=open, 0=closed).

        Two firmware quirks this guards against (empirically observed):
          1. GetGripperCurPosition transiently reads 0 right after a
             MoveGripper command — so we ignore all readings before
             min_wait_s and require n_confirm consecutive in-tolerance
             readings before declaring success.
          2. GetGripperMotionDone flag semantics are unreliable — logged
             once for diagnostics, never used for the verdict.

        When gripping an object the jaws stall before the commanded
        position; stall (position unchanged ~1s after min_wait_s) also
        counts as success."""
        deadline = time.time() + timeout_s
        logged_raw = False
        last_pos = None
        stable_since = None
        confirm_count = 0
        start = time.time()
        while time.time() < deadline:
            with self._robot_lock:
                rc = self.robot.GetGripperMotionDone()
                gp = self.robot.GetGripperCurPosition()
            if not logged_raw:
                self.get_logger().info(
                    f"  raw GetGripperMotionDone={rc!r}  "
                    f"GetGripperCurPosition={gp!r}")
                logged_raw = True
            # Extract current position pct (defensive across formats)
            pos = None
            try:
                if isinstance(gp, (tuple, list)) and gp[0] == 0:
                    payload = gp[1]
                    pos = float(payload[-1]) if isinstance(
                        payload, (tuple, list)) else float(gp[-1])
            except (IndexError, TypeError, ValueError):
                pass
            elapsed = time.time() - start
            if pos is not None and elapsed >= min_wait_s:
                if abs(pos - target_pct) <= tol_pct:
                    confirm_count += 1
                    if confirm_count >= n_confirm:
                        self._gripper_joint_rad = gripper_pct_to_rad(pos)
                        self.get_logger().info(
                            f"  gripper at {pos:.0f}% "
                            f"≈ {pos/100.0*GRIPPER_STROKE_MM:.0f}mm "
                            f"(target {target_pct:.0f}%) after {elapsed:.1f}s")
                        return True
                else:
                    confirm_count = 0
                    # Stall detection: position stopped changing (object grip)
                    if last_pos is not None and abs(pos - last_pos) < 0.5:
                        if stable_since is None:
                            stable_since = time.time()
                        elif time.time() - stable_since > 1.0:
                            self.get_logger().info(
                                f"  gripper stalled at {pos:.0f}% "
                                f"≈ {pos/100.0*GRIPPER_STROKE_MM:.0f}mm "
                                f"(target {target_pct:.0f}%) — gripped object, "
                                "treating as success")
                            self._gripper_joint_rad = gripper_pct_to_rad(pos)
                            return True
                    else:
                        stable_since = None
                last_pos = pos
            time.sleep(0.1)
        self.get_logger().error(
            f"Gripper did not reach {target_pct:.0f}% within {timeout_s:.1f}s "
            f"(last position: {last_pos})")
        return False

    def _execute_callback(self, goal_handle: ServerGoalHandle):
        """Execute the trajectory on the real robot via SDK."""
        traj = goal_handle.request.trajectory
        points = traj.points
        joint_order = traj.joint_names

        self.get_logger().info(f"Executing {len(points)} trajectory points")

        if self.use_servoj:
            result = self._execute_servoj(goal_handle, points, joint_order)
        else:
            result = self._execute_movej(goal_handle, points, joint_order)

        return result

    def _reorder_joints(self, joint_order, positions_rad):
        """Reorder joint positions according to our JOINT_NAMES (j1..j6)."""
        name_to_val = dict(zip(joint_order, positions_rad))
        return [name_to_val[jn] for jn in JOINT_NAMES]

    def _execute_movej(self, goal_handle, points, joint_order):
        """Execute by sending segmented MoveJ commands along the MoveIt2 trajectory.

        We iterate through trajectory waypoints (downsampled) and MoveJ to each
        in sequence. This follows MoveIt2's collision-free path closely, instead
        of taking a direct joint-space shortcut that might hit obstacles.
        """
        result = FollowJointTrajectory.Result()

        # Downsample trajectory to ~10 segments (plus endpoints)
        # Too few = might miss obstacle avoidance curves
        # Too many = slow execution due to MoveJ startup overhead
        n_total = len(points)
        n_segments = min(n_total, max(10, n_total // 10))
        step = max(1, n_total // n_segments)

        waypoint_indices = list(range(0, n_total, step))
        if waypoint_indices[-1] != n_total - 1:
            waypoint_indices.append(n_total - 1)  # always include final point

        self.get_logger().info(
            f"Executing {len(waypoint_indices)} waypoints from {n_total}-point trajectory"
        )

        for seg_idx, pt_idx in enumerate(waypoint_indices):
            if goal_handle.is_cancel_requested:
                self.get_logger().warn("Trajectory execution cancelled")
                with self._robot_lock:
                    self.robot.StopMotion()
                result.error_code = result.INVALID_GOAL
                goal_handle.canceled()
                return result

            pt = points[pt_idx]
            positions_rad = list(pt.positions)
            joints_rad = self._reorder_joints(joint_order, positions_rad)
            joints_deg = [math.degrees(r) for r in joints_rad]

            # Joint sanity check
            if any(abs(jd) > 350 for jd in joints_deg):
                self.get_logger().error(f"Unreasonable joint target: {joints_deg}")
                result.error_code = result.PATH_TOLERANCE_VIOLATED
                goal_handle.abort()
                return result

            is_final = (seg_idx == len(waypoint_indices) - 1)
            self.get_logger().info(
                f"  [{seg_idx+1}/{len(waypoint_indices)}] MoveJ to "
                f"{[f'{d:.1f}' for d in joints_deg]}°"
                + (" (final)" if is_final else "")
            )

            with self._robot_lock:
                # MoveJ with blend radius (ovl) to chain smoothly between segments
                # ovl=100.0 means full-speed blend. Use lower for final point.
                ret = self.robot.MoveJ(
                    joints_deg,
                    0, 0,  # tool, user
                    [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    self.movej_vel,
                    self.movej_acc,
                    100.0,  # ovl (blend)
                )

            err_code = ret[0] if isinstance(ret, tuple) else ret
            if err_code != 0:
                self.get_logger().error(f"MoveJ segment {seg_idx+1} failed: code {err_code}")
                result.error_code = result.INVALID_GOAL
                goal_handle.abort()
                return result

            # Wait for this segment to complete before sending next
            if not self._wait_for_motion_complete(joints_deg, timeout_s=30.0, tol_deg=1.0):
                self.get_logger().warn(f"Segment {seg_idx+1} didn't settle cleanly")

        self.get_logger().info("Trajectory execution complete")
        result.error_code = result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def _execute_servoj(self, goal_handle, points, joint_order):
        """Execute by streaming points via ServoJ at trajectory rate.
        Follows MoveIt2's exact path including collision-avoidance curves.
        """
        result = FollowJointTrajectory.Result()

        n = len(points)
        self.get_logger().info(f"ServoJ streaming {n} trajectory points")

        # Compute the actual step duration between consecutive points
        # MoveIt2 usually spaces points at ~10-50ms
        times = []
        for pt in points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            times.append(t)

        total_duration = times[-1] if times else 0.0
        self.get_logger().info(f"Planned total duration: {total_duration:.2f}s")

        with self._robot_lock:
            self.robot.ServoMoveStart()

        start_wall = time.time()
        try:
            for i, pt in enumerate(points):
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn("ServoJ cancelled")
                    result.error_code = result.INVALID_GOAL
                    goal_handle.canceled()
                    return result

                # Reorder to j1..j6 and convert rad → deg
                positions_rad = list(pt.positions)
                joints_rad = self._reorder_joints(joint_order, positions_rad)
                joints_deg = [math.degrees(r) for r in joints_rad]

                # Sanity check
                if any(abs(jd) > 350 for jd in joints_deg):
                    self.get_logger().error(f"Unreasonable joint: {joints_deg}")
                    result.error_code = result.PATH_TOLERANCE_VIOLATED
                    goal_handle.abort()
                    return result

                # Calculate dt for this segment
                if i == 0:
                    dt = 0.02
                else:
                    dt = times[i] - times[i-1]
                dt = max(dt, 0.008)  # minimum servo period

                with self._robot_lock:
                    # ServoJ(joint_pos, axisPos, acc=0, vel=0, cmdT=0.008, filterT=0, gain=0, id=0)
                    self.robot.ServoJ(
                        joints_deg,
                        [0.0] * 6,  # axisPos
                        0.0, 0.0,   # acc, vel (0 = auto)
                        dt,         # cmdT — time between commands
                        0.05,       # filterT (light smoothing)
                        0.0,        # gain (0 = default)
                        0,          # id
                    )

                # Wait to match trajectory timing
                wall_elapsed = time.time() - start_wall
                target_wall = times[i]
                sleep_time = target_wall - wall_elapsed
                if sleep_time > 0.001:
                    time.sleep(sleep_time)

        finally:
            with self._robot_lock:
                self.robot.ServoMoveEnd()

        self.get_logger().info(
            f"ServoJ streaming completed in {time.time()-start_wall:.2f}s "
            f"(planned {total_duration:.2f}s)"
        )
        result.error_code = result.SUCCESSFUL
        goal_handle.succeed()
        return result

    def _wait_for_motion_complete(self, target_joints_deg, timeout_s=60.0, tol_deg=0.5):
        """Poll until robot reaches target or timeout."""
        start = time.time()
        while time.time() - start < timeout_s:
            with self._robot_lock:
                ret = self.robot.GetActualJointPosDegree(1)
            if ret[0] == 0:
                current = ret[1]
                deltas = [abs(c - t) for c, t in zip(current, target_joints_deg)]
                if max(deltas) < tol_deg:
                    return True
            time.sleep(0.05)
        self.get_logger().warn(f"Motion did not settle within {timeout_s}s")
        return False

    def destroy_node(self):
        try:
            self.robot.CloseRPC()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SDKExecutorNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

"""
PXRD bridge launch — ONLY the sdk_executor that talks to the real FR10.

Mirrors unchained_cell/launch/bridge.launch.py (the hardware-proven
pattern); the fr_bridge package is cell-agnostic. This launch loads no
URDF, MoveIt, or rviz. Its sole job is the SDK bridge that:

  - Receives FollowJointTrajectory goals from any orchestrator
  - Streams them to the real FR10 via the Fairino SDK (ServoJ or MoveJ)
  - Publishes /joint_states from real encoders at 50 Hz
  - Exposes /stop_motion service for emergency halt

DANGER — once this launch is running, ANY node that sends a
FollowJointTrajectory goal can move the real robot. Keep the e-stop in
hand, pendant override low (25%), and movej_vel_pct <= 5 near the
instrument (repo rule).

Usage (three terminals, domain 43 each — source ros2_ws/env_pxrd.sh):
    # T1 — bridge to the real robot (slow defaults for first tests)
    ./run_pxrd.sh bridge                 # robot_ip:=... to override

    # T2 — scene + MoveIt against the real robot
    ./run_pxrd.sh hw-scene

    # T3 — the same pick_place as sim, plus --hardware
    ./run_pxrd.sh load --plate v3 --hardware
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    robot_model = LaunchConfiguration("robot_model").perform(context)
    use_servoj = LaunchConfiguration("use_servoj").perform(context)
    movej_vel = LaunchConfiguration("movej_vel_pct").perform(context)
    movej_acc = LaunchConfiguration("movej_acc_pct").perform(context)
    js_rate = LaunchConfiguration("joint_state_rate_hz").perform(context)
    gripper_vel = LaunchConfiguration("gripper_vel_pct").perform(context)
    gripper_force = LaunchConfiguration("gripper_force_pct").perform(context)

    robot_num = robot_model[2:]  # "10" from "fr10"
    controller_name = f"fairino{robot_num}_controller"

    sdk_executor = Node(
        package="fr_bridge",
        executable="sdk_executor",
        name="sdk_executor",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"controller_name": controller_name},
            {"joint_state_rate_hz": float(js_rate)},
            {"movej_vel_pct": float(movej_vel)},
            {"movej_acc_pct": float(movej_acc)},
            {"use_servoj": use_servoj.lower() == "true"},
            {"gripper_vel_pct": float(gripper_vel)},
            {"gripper_force_pct": float(gripper_force)},
        ],
    )

    return [sdk_executor]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value="192.168.58.2",
                              description="IP of the real FR10 controller "
                                          "(Fairino factory default; VERIFY on "
                                          "the pendant before first use)."),
        DeclareLaunchArgument("robot_model", default_value="fr10",
                              description="Robot model: fr10, fr16, fr20. "
                                          "Determines the controller name suffix."),
        # ─── First-test defaults: VERY SLOW ───
        DeclareLaunchArgument("use_servoj", default_value="false",
                              description="true: ServoJ streams the exact MoveIt "
                                          "trajectory (production). false: MoveJ "
                                          "per-waypoint (safer for first tests)."),
        DeclareLaunchArgument("movej_vel_pct", default_value="3.0",
                              description="MoveJ velocity %% of robot max (3.0 for "
                                          "first hardware tests; repo rule: <=5 "
                                          "near the instrument)."),
        DeclareLaunchArgument("movej_acc_pct", default_value="5.0",
                              description="MoveJ acceleration %% of robot max."),
        DeclareLaunchArgument("joint_state_rate_hz", default_value="50.0",
                              description="Joint-state poll/publish rate."),
        DeclareLaunchArgument("gripper_vel_pct", default_value="30.0",
                              description="AG-145 jaw speed (%% of max)."),
        DeclareLaunchArgument("gripper_force_pct", default_value="20.0",
                              description="AG-145 grip FORCE (%% of max) — the "
                                          "real grasp knob; the gripper stalls on "
                                          "the paddle at this force."),
        OpaqueFunction(function=launch_setup),
    ])

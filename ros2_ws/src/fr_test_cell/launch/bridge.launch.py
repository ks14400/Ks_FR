"""
Bridge launch — ONLY the sdk_executor that talks to the real FR16.

This launch is intentionally minimal. It does not load any URDF, MoveIt,
or rviz. Its sole job is to run the SDK bridge that:

  - Receives FollowJointTrajectory goals from any orchestrator
  - Streams them to the real FR16 via Fairino SDK (ServoJ or MoveJ)
  - Publishes /joint_states from real encoders at 50 Hz
  - Exposes /stop_motion service for emergency halt

Why separate from the scene launch? So the same bridge works with ANY sim
or orchestration on top — pick_place, replay scripts, rviz teleop, etc.

DANGER — once this launch is running, ANY node that sends a
FollowJointTrajectory goal can move the real robot. Keep e-stop in hand.

Usage:
    # Terminal 1 — bridge to real robot (slow defaults for first tests)
    ros2 launch fr_test_cell bridge.launch.py robot_ip:=192.168.58.2

    # Terminal 2 — your scene/orchestration
    ros2 launch fr_test_cell unchained_full.launch.py control_mode:=hardware \\
        table_x:=-0.2 table_y:=-1.2 table_h:=0.9

    # Terminal 3 — pick_place exactly like in sim
    ros2 run fr_test_cell pick_place --source deck_9_10_pos1 --target table_top \\
        --vel 0.05
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

    robot_num = robot_model[2:]  # "16" from "fr16"
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
                              description="IP address of the real FR16 controller"),
        DeclareLaunchArgument("robot_model", default_value="fr16",
                              description="Robot model: fr10, fr16, fr20. "
                                          "Determines the controller name suffix."),
        # ─── First-test defaults: VERY SLOW ───
        # use_servoj=False means MoveJ per-waypoint (easier to see and stop
        # between segments). For production use ServoJ (streams the exact
        # MoveIt path).
        DeclareLaunchArgument("use_servoj", default_value="false",
                              description="true: ServoJ streams the exact MoveIt "
                                          "trajectory (production). false: MoveJ "
                                          "per-waypoint (safer for first tests)."),
        DeclareLaunchArgument("movej_vel_pct", default_value="3.0",
                              description="MoveJ velocity %% of robot max (default "
                                          "3.0 for first hardware tests; raise once "
                                          "you trust the motion)."),
        DeclareLaunchArgument("movej_acc_pct", default_value="5.0",
                              description="MoveJ acceleration %% of robot max."),
        DeclareLaunchArgument("joint_state_rate_hz", default_value="50.0",
                              description="Rate at which to poll the robot for "
                                          "joint states and publish /joint_states"),
        DeclareLaunchArgument("gripper_vel_pct", default_value="30.0",
                              description="AG-145 closing/opening speed (%% of max)."),
        DeclareLaunchArgument("gripper_force_pct", default_value="20.0",
                              description="AG-145 grip FORCE (%% of max). This is "
                                          "the meaningful grasp knob — the gripper "
                                          "is force-controlled and stalls on the "
                                          "object at this force. Lower = gentler "
                                          "(fragile samples), higher = firmer hold."),
        OpaqueFunction(function=launch_setup),
    ])

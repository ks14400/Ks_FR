"""
Launch MoveIt2 + rviz2 + SDK executor bridge to control the REAL FR16.

Architecture:
  rviz2 → MoveIt2 (mock plan mode) → FollowJointTrajectory action
    → sdk_executor (our Python bridge) → SDK MoveJ → Real FR16

This avoids the buggy fairino_hardware C++ plugin by using the Python SDK
directly. The SDK executor also publishes /joint_states at 50Hz from the
real robot, so rviz2 always shows the actual arm position.

DANGER: Executing a motion in rviz2 moves the REAL ROBOT.
        Keep e-stop in hand. Start with small motions.

Usage:
    ros2 launch fr_test_cell fr_test_hardware_bridge.launch.py robot_model:=fr16
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


ROBOT_CONFIGS = {
    "fr10": {"num": "10", "pkg": "fairino10_v6_moveit2_config", "robot_name": "fairino10_v6_robot"},
    "fr16": {"num": "16", "pkg": "fairino16_v6_moveit2_config", "robot_name": "fairino16_v6_robot"},
    "fr20": {"num": "20", "pkg": "fairino20_v6_moveit2_config", "robot_name": "fairino20_v6_robot"},
}


def launch_setup(context, *args, **kwargs):
    robot_model = LaunchConfiguration("robot_model").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)

    if robot_model not in ROBOT_CONFIGS:
        raise ValueError(f"Unknown robot_model '{robot_model}'. Use: fr10, fr16, fr20")

    rc = ROBOT_CONFIGS[robot_model]
    robot_num = rc["num"]
    moveit_pkg = rc["pkg"]
    robot_name = rc["robot_name"]
    controller_name = f"fairino{robot_num}_controller"

    test_cell_share = get_package_share_directory("fr_test_cell")
    moveit_share = get_package_share_directory(moveit_pkg)

    ped_x     = float(LaunchConfiguration("pedestal_x").perform(context))
    ped_y     = float(LaunchConfiguration("pedestal_y").perform(context))
    ped_z     = float(LaunchConfiguration("pedestal_z").perform(context))
    ped_roll  = float(LaunchConfiguration("pedestal_roll").perform(context))
    ped_pitch = float(LaunchConfiguration("pedestal_pitch").perform(context))
    ped_yaw   = float(LaunchConfiguration("pedestal_yaw").perform(context))

    test_urdf = os.path.join(test_cell_share, "urdf", "fr_test_cell.urdf.xacro")

    # Use 'mock' control mode — we don't actually use the mock controller,
    # we use our SDK executor. But mock mode avoids the C++ plugin entirely.
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_pkg)
        .robot_description(
            file_path=test_urdf,
            mappings={
                "robot_model": robot_model,
                "control_mode": "mock",
                "pedestal_x": str(ped_x),
                "pedestal_y": str(ped_y),
                "pedestal_z": str(ped_z),
                "pedestal_roll": str(ped_roll),
                "pedestal_pitch": str(ped_pitch),
                "pedestal_yaw": str(ped_yaw),
            },
        )
        .robot_description_semantic(file_path=f"config/fairino{robot_num}_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── Robot state publisher — uses /joint_states from sdk_executor ──
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── SDK executor — our Python bridge to the real robot ──
    # Publishes /joint_states at 50Hz from real robot
    # Provides FollowJointTrajectory action at /{controller_name}/follow_joint_trajectory
    sdk_executor = Node(
        package="fr_bridge",
        executable="sdk_executor",
        name="sdk_executor",
        output="screen",
        parameters=[
            {"robot_ip": robot_ip},
            {"controller_name": controller_name},
            {"joint_state_rate_hz": 50.0},
            {"movej_vel_pct": 10.0},
            {"movej_acc_pct": 20.0},
            {"use_servoj": True},   # ServoJ follows MoveIt2's exact trajectory
        ],
    )

    # ── MoveIt2 move_group ──
    # Note: trajectory_execution_manager will connect to the action server
    # published by sdk_executor (matching name from moveit_controllers.yaml)
    # Give execution more time than MoveIt2's default (to accommodate
    # SDK communication latency and robot settling time).
    execution_tolerance = {
        "trajectory_execution.allowed_execution_duration_scaling": 10.0,
        "trajectory_execution.allowed_goal_duration_margin": 30.0,
        "trajectory_execution.allowed_start_tolerance": 0.05,  # 0.05 rad ~= 3°
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), execution_tolerance],
    )

    # ── rviz2 ──
    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "world"],
    )

    return [
        static_tf,
        sdk_executor,       # Starts FIRST so it's ready before MoveIt2 queries it
        rsp,
        move_group,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="fr16",
                              description="Robot model: fr10, fr16, or fr20"),
        DeclareLaunchArgument("robot_ip", default_value="192.168.58.2",
                              description="Real robot IP address"),
        DeclareLaunchArgument("pedestal_x",     default_value="-0.2025"),
        DeclareLaunchArgument("pedestal_y",     default_value="0.2025"),
        DeclareLaunchArgument("pedestal_z",     default_value="-0.585"),
        DeclareLaunchArgument("pedestal_roll",  default_value="1.5708"),
        DeclareLaunchArgument("pedestal_pitch", default_value="0"),
        DeclareLaunchArgument("pedestal_yaw",   default_value="0"),
        OpaqueFunction(function=launch_setup),
    ])

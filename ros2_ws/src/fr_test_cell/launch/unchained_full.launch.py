"""
Unchained build with FR16 + MoveIt2 motion planning.

Default (mock mode): rviz with MotionPlanning panel, mock controllers.
  - Drag end-effector marker → click Plan → click Execute (in simulation only)
  - Robot turns red on collision (e.g., if you try to drive arm into Unchained)

control_mode:=gazebo for physics simulation
control_mode:=hardware for real robot (uses SDK executor — see other launch)

Usage:
    ros2 launch fr_test_cell unchained_full.launch.py
    ros2 launch fr_test_cell unchained_full.launch.py robot_mount_yaw_deg:=90
"""
import os
import math
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
    ExecuteProcess, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro


def launch_setup(context, *args, **kwargs):
    test_cell_share = get_package_share_directory("fr_test_cell")
    fairino_desc_share = get_package_share_directory("fairino_description")

    robot_model = LaunchConfiguration("robot_model").perform(context)
    control_mode = LaunchConfiguration("control_mode").perform(context)
    device_height = LaunchConfiguration("device_height").perform(context)
    yaw_deg = LaunchConfiguration("robot_mount_yaw_deg").perform(context)
    mount_x = LaunchConfiguration("robot_mount_x").perform(context)
    mount_y = LaunchConfiguration("robot_mount_y").perform(context)
    mount_z = LaunchConfiguration("robot_mount_z").perform(context)
    table_x = LaunchConfiguration("table_x").perform(context)
    table_y = LaunchConfiguration("table_y").perform(context)
    table_z = LaunchConfiguration("table_z").perform(context)
    table_w = LaunchConfiguration("table_w").perform(context)
    table_d = LaunchConfiguration("table_d").perform(context)
    table_h = LaunchConfiguration("table_h").perform(context)
    include_table = LaunchConfiguration("include_table").perform(context)

    robot_num = robot_model[2:]
    moveit_pkg = f"fairino{robot_num}_v6_moveit2_config"

    # ── MoveIt2 config (uses our combined URDF) ──
    test_urdf_xacro = os.path.join(test_cell_share, "urdf", "unchained_full.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder(
            f"fairino{robot_num}_v6_robot",
            package_name=moveit_pkg,
        )
        .robot_description(
            file_path=test_urdf_xacro,
            mappings={
                "robot_model": robot_model,
                "control_mode": control_mode,
                "device_height": device_height,
                "robot_mount_yaw_deg": yaw_deg,
                "robot_mount_x": mount_x,
                "robot_mount_y": mount_y,
                "robot_mount_z": mount_z,
                "table_x": table_x,
                "table_y": table_y,
                "table_z": table_z,
                "table_w": table_w,
                "table_d": table_d,
                "table_h": table_h,
                "include_table": include_table,
                # Override the upstream all-zeros initial_positions with our
                # ready pose so the arm comes up folded, not extended.
                "initial_positions_file":
                    os.path.join(test_cell_share, "config",
                                 "initial_positions.yaml"),
            },
        )
        .robot_description_semantic(
            file_path=os.path.join(test_cell_share, "config", "unchained_full.srdf"))
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path=os.path.join(test_cell_share, "config",
                                   "unchained_moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ── robot_state_publisher ──
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ── ros2_control_node — provides controller_manager for mock/hardware ──
    moveit_share = get_package_share_directory(moveit_pkg)
    ros2_controllers_yaml = os.path.join(
        test_cell_share, "config", "unchained_ros2_controllers.yaml")
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_yaml],
        output="screen",
    )

    # ── Spawn controllers ──
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="log",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[f"fairino{robot_num}_controller",
                   "--controller-manager", "/controller_manager"],
        output="log",
    )
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller",
                   "--controller-manager", "/controller_manager"],
        output="log",
    )

    # ── MoveIt2 move_group ──
    # In hardware mode, give the trajectory_execution_manager more headroom
    # for SDK communication latency and real-robot settling. The defaults
    # are tuned for the mock controller's near-zero latency.
    extra_mg_params = {}
    if control_mode == "hardware":
        extra_mg_params = {
            # The bridge executes MoveJ at a fixed SDK speed (movej_vel_pct),
            # ignoring the trajectory's planned timing — so real execution can
            # take far longer than the planned traj_dur. The allowance is
            # traj_dur * scaling + margin; it must cover the worst case or
            # MoveIt reports TIMED_OUT (-6) on motions that are actually
            # completing fine on the robot.
            "trajectory_execution.allowed_execution_duration_scaling": 40.0,
            "trajectory_execution.allowed_goal_duration_margin": 60.0,
            "trajectory_execution.allowed_start_tolerance": 0.05,  # ~3°
        }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), extra_mg_params],
    )

    # ── rviz2 with MoveIt2 motion planning panel ──
    # Use our local config (Fixed Frame = "world" so the ground plane sits at
    # the actual floor, not at the robot base). In hardware mode use the
    # variant where the planned path renders bright green & translucent,
    # so it's never confused with the REAL arm (rendered with default colors
    # from /joint_states published by the bridge).
    if control_mode == "hardware":
        rviz_config = os.path.join(
            test_cell_share, "config", "unchained_full_hardware.rviz")
    else:
        rviz_config = os.path.join(
            test_cell_share, "config", "unchained_full.rviz")
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
        name="map_to_world",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "world"],
    )

    nodes = [
        static_tf,
        rsp,
        ros2_control_node,
        joint_state_broadcaster,
        arm_controller,
        gripper_controller,
        move_group,
        rviz_node,
    ]

    # In hardware mode: skip mock controllers entirely. The bridge launch
    # (bridge.launch.py) runs sdk_executor in its own process which provides
    # /joint_states from real encoders and the FollowJointTrajectory action.
    # MoveIt2's trajectory_execution_manager connects to that action just
    # like it would to the mock one.
    if control_mode == "hardware":
        nodes = [n for n in nodes if n not in (
            ros2_control_node,
            joint_state_broadcaster,
            arm_controller,
            gripper_controller,
        )]

    # Optional: Gazebo for physics visualization
    if control_mode == "gazebo":
        existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
        gazebo_resource_path = os.pathsep.join(filter(None, [
            existing_path,
            test_cell_share,
            os.path.dirname(test_cell_share),
            fairino_desc_share,
            os.path.dirname(fairino_desc_share),
        ]))
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory("ros_gz_sim"),
                             "launch", "gz_sim.launch.py")
            ]),
            launch_arguments={"gz_args": "empty.sdf -r"}.items(),
        )
        # In gazebo mode, ros2_control_node is replaced by Gazebo's gz_ros2_control
        # so we remove it from the list and let Gazebo handle controllers
        nodes.remove(ros2_control_node)
        nodes.extend([
            SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=gazebo_resource_path),
            SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gazebo_resource_path),
            gazebo,
        ])

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="fr16",
                              description="Robot model: fr10, fr16, fr20"),
        DeclareLaunchArgument("control_mode", default_value="mock",
                              description="mock (default) | gazebo | hardware"),
        DeclareLaunchArgument("device_height", default_value="0.629675",
                              description="Height (m) of Unchained CAD origin above floor"),
        DeclareLaunchArgument("robot_mount_yaw_deg", default_value="-45",
                              description="Robot mounting yaw in degrees around "
                                          "vertical axis. -45 matches the real "
                                          "FR16 mounting (connector on the right, "
                                          "parallel to the Unchained)."),
        DeclareLaunchArgument("robot_mount_x", default_value="-0.113",
                              description="Robot mount X offset in pedestal frame (m)"),
        DeclareLaunchArgument("robot_mount_y", default_value="0.0",
                              description="Robot mount Y offset in pedestal frame (m)"),
        DeclareLaunchArgument("robot_mount_z", default_value="0.392",
                              description="Robot mount Z offset in pedestal frame (m)"),
        DeclareLaunchArgument("include_table", default_value="true",
                              description="Include the lab loading table in the scene"),
        DeclareLaunchArgument("table_x", default_value="0.7",
                              description="Table X position in world (m)"),
        DeclareLaunchArgument("table_y", default_value="-1.3",
                              description="Table Y position in world (m)"),
        DeclareLaunchArgument("table_z", default_value="0.0",
                              description="Table Z position in world (m); top of table is at this + table_h"),
        DeclareLaunchArgument("table_w", default_value="0.30",
                              description="Table width (X dimension, m)"),
        DeclareLaunchArgument("table_d", default_value="0.30",
                              description="Table depth (Y dimension, m)"),
        DeclareLaunchArgument("table_h", default_value="0.70",
                              description="Table height (Z dimension, m)"),
        OpaqueFunction(function=launch_setup),
    ])

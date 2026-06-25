"""
Launch MoveIt2 + rviz2 connected to the REAL Fairino robot (hardware mode).

Uses the fairino_hardware plugin to communicate with the robot via TCP/IP.
No Gazebo. The robot IP is defined at compile time in
    fairino_hardware/include/fairino_hardware/data_type_def.h
    (set to 192.168.58.2 during Phase 2 setup)

DANGER: Executing a motion in rviz2 moves the REAL ROBOT.
        Always validate in simulation first and keep the e-stop in hand.

Usage:
    ros2 launch fr_test_cell fr_test_hardware.launch.py robot_model:=fr16
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
    if robot_model not in ROBOT_CONFIGS:
        raise ValueError(f"Unknown robot_model '{robot_model}'. Use: fr10, fr16, fr20")

    rc = ROBOT_CONFIGS[robot_model]
    robot_num = rc["num"]
    moveit_pkg = rc["pkg"]
    robot_name = rc["robot_name"]
    controller_name = f"fairino{robot_num}_controller"

    test_cell_share = get_package_share_directory("fr_test_cell")
    moveit_share = get_package_share_directory(moveit_pkg)

    # Pedestal pose (for collision avoidance — robot must not hit the pedestal)
    ped_x     = float(LaunchConfiguration("pedestal_x").perform(context))
    ped_y     = float(LaunchConfiguration("pedestal_y").perform(context))
    ped_z     = float(LaunchConfiguration("pedestal_z").perform(context))
    ped_roll  = float(LaunchConfiguration("pedestal_roll").perform(context))
    ped_pitch = float(LaunchConfiguration("pedestal_pitch").perform(context))
    ped_yaw   = float(LaunchConfiguration("pedestal_yaw").perform(context))

    # ══════════════════════════════════════════════════════════
    # URDF: Combined scene with hardware control plugin
    # ══════════════════════════════════════════════════════════
    test_urdf = os.path.join(test_cell_share, "urdf", "fr_test_cell.urdf.xacro")

    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_pkg)
        .robot_description(
            file_path=test_urdf,
            mappings={
                "robot_model": robot_model,
                "control_mode": "hardware",      # ← real robot
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

    # ══════════════════════════════════════════════════════════
    # robot_state_publisher
    # ══════════════════════════════════════════════════════════
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # ══════════════════════════════════════════════════════════
    # ros2_control_node — loads fairino_hardware plugin,
    # connects to real robot via TCP/IP
    # ══════════════════════════════════════════════════════════
    ros2_controllers_yaml = os.path.join(
        moveit_share, "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_yaml],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    robot_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[controller_name, "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # ══════════════════════════════════════════════════════════
    # MoveIt2 move_group
    # ══════════════════════════════════════════════════════════
    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # ══════════════════════════════════════════════════════════
    # rviz2
    # ══════════════════════════════════════════════════════════
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

    # Print safety banner
    banner = Node(
        package="demo_nodes_cpp",
        executable="talker",
        name="safety_banner",
        arguments=["--ros-args"],
        output="log",
        condition=None,  # no-op placeholder (no banner in ROS2 native launch)
    ) if False else None

    nodes = [
        static_tf,
        rsp,
        ros2_control_node,
        joint_state_broadcaster,
        robot_controller,
        move_group,
        rviz_node,
    ]
    return [n for n in nodes if n is not None]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="fr16",
                              description="Robot model: fr10, fr16, or fr20"),
        # Pedestal for collision checking — same defaults as sim for MoveIt frame
        DeclareLaunchArgument("pedestal_x",     default_value="-0.2025"),
        DeclareLaunchArgument("pedestal_y",     default_value="0.2025"),
        DeclareLaunchArgument("pedestal_z",     default_value="-0.585",
                              description="Pedestal Z in MoveIt frame (below base)"),
        DeclareLaunchArgument("pedestal_roll",  default_value="1.5708"),
        DeclareLaunchArgument("pedestal_pitch", default_value="0"),
        DeclareLaunchArgument("pedestal_yaw",   default_value="0"),
        OpaqueFunction(function=launch_setup),
    ])

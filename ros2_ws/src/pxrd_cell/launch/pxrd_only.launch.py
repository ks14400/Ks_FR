"""
Visualize the PXRD (SmartLab) instrument as ground truth in rviz2.

Step 1 of the PXRD cell build — confirm orientation / scale / origin before
adding the pedestal, arm, and decks.

Usage:
    ros2 launch pxrd_cell pxrd_only.launch.py
    ros2 launch pxrd_cell pxrd_only.launch.py device_height:=1.30
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory("pxrd_cell")
    device_height = LaunchConfiguration("device_height").perform(context)
    include_arm = LaunchConfiguration("include_arm").perform(context)
    mount_yaw = LaunchConfiguration("robot_mount_yaw_deg").perform(context)

    xacro_path = os.path.join(share, "urdf", "pxrd_only.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_path,
        mappings={
            "device_height": device_height,
            "include_arm": include_arm,
            "robot_mount_yaw_deg": mount_yaw,
        },
    ).toxml()
    robot_description = {"robot_description": robot_description_xml}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_world",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "world"],
    )

    rviz_config = os.path.join(share, "config", "pxrd_only.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
    )

    nodes = [static_tf, rsp, rviz_node]

    # The arm's 6 revolute joints need a /joint_states source to render.
    # The GUI gives sliders to pose the arm and eyeball reach to the instrument.
    if include_arm.lower() == "true":
        nodes.append(Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="log",
        ))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "device_height", default_value="1.21",
            description="Height (m) of the SmartLab CAD origin above the floor. "
                        "Default puts the lowest geometry on the floor."),
        DeclareLaunchArgument(
            "include_arm", default_value="true",
            description="Mount the FR10 arm on the pedestal."),
        DeclareLaunchArgument(
            "robot_mount_yaw_deg", default_value="0",
            description="Arm mounting yaw on the pedestal (deg). Adjust to match "
                        "the physical bolt orientation after viewing in rviz."),
        OpaqueFunction(function=launch_setup),
    ])

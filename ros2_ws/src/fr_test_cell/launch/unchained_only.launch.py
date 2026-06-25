"""
Show the Unchained Junior in Gazebo + rviz2.
STL placed AS-IS — SolidWorks frame = world frame.

Usage:
    ros2 launch fr_test_cell unchained_only.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
    ExecuteProcess, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def launch_setup(context, *args, **kwargs):
    test_cell_share = get_package_share_directory("fr_test_cell")

    # Read launch args
    dx = LaunchConfiguration("deck_offset_x").perform(context)
    dy = LaunchConfiguration("deck_offset_y").perform(context)
    dz = LaunchConfiguration("deck_offset_z").perform(context)
    device_height = LaunchConfiguration("device_height").perform(context)
    include_pedestal = LaunchConfiguration("include_pedestal").perform(context)

    # Process xacro
    xacro_path = os.path.join(test_cell_share, "urdf", "unchained_only.urdf.xacro")
    robot_description_xml = xacro.process_file(
        xacro_path,
        mappings={
            "deck_offset_x": dx,
            "deck_offset_y": dy,
            "deck_offset_z": dz,
            "device_height": device_height,
            "include_pedestal": include_pedestal,
        },
    ).toxml()

    # Write to temp file for rviz2 RobotModel "File" source
    urdf_temp_path = "/tmp/unchained_only_processed.urdf"
    with open(urdf_temp_path, "w") as f:
        f.write(robot_description_xml)

    robot_description = {"robot_description": robot_description_xml}

    existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gazebo_resource_path = os.pathsep.join(filter(None, [
        existing_path,
        test_cell_share,
        os.path.dirname(test_cell_share),
    ]))

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")
        ]),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    visual_mesh = os.path.join(test_cell_share, "meshes", "unchained_junior_visual.stl")
    collision_mesh = os.path.join(test_cell_share, "meshes", "unchained_junior_collision.stl")

    device_sdf = (
        f'<?xml version=\\"1.0\\"?>'
        f'<sdf version=\\"1.8\\">'
        f'<model name=\\"unchained_junior\\"><static>true</static>'
        f'<link name=\\"device_link\\">'
        f'<inertial><mass>50</mass></inertial>'
        f'<visual name=\\"v\\"><geometry><mesh>'
        f'<uri>file://{visual_mesh}</uri>'
        f'<scale>0.001 0.001 0.001</scale>'
        f'</mesh></geometry>'
        f'<material><ambient>0.6 0.6 0.65 1</ambient>'
        f'<diffuse>0.6 0.6 0.65 1</diffuse></material></visual>'
        f'<collision name=\\"c\\"><geometry><mesh>'
        f'<uri>file://{collision_mesh}</uri>'
        f'<scale>0.001 0.001 0.001</scale>'
        f'</mesh></geometry></collision>'
        f'</link></model></sdf>'
    )

    # 90° around X (qx=0.7071, qw=0.7071) to align CAD Y-up to world Z-up
    spawn_device = ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", "/world/empty/create",
            "--reqtype", "ignition.msgs.EntityFactory",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "10000",
            "--req",
            f'sdf: "{device_sdf}" '
            f'pose: {{position: {{x: 0, y: 0, z: {device_height}}} '
            f'orientation: {{x: 0.7071068, y: 0, z: 0, w: 0.7071068}}}} '
            f'name: "unchained_junior"',
        ],
        output="screen",
    )

    # ── Pedestal in Gazebo ──
    # Pedestal in CAD frame: (73.98, 315.33, 1265.27) mm
    # In world frame after Unchained's 90° X rotation + lift:
    #   world_x = cad_x = 0.07398
    #   world_y = -cad_z = -1.26527
    #   world_z = cad_y + device_height = 0.31533 + 0.629675 = 0.945
    # Pedestal also gets the same 90° X rotation (since it's in CAD frame)
    pedestal_visual = os.path.join(test_cell_share, "meshes", "vention_pedestal_visual.stl")
    pedestal_collision = os.path.join(test_cell_share, "meshes", "vention_pedestal_collision.stl")

    pedestal_sdf = (
        f'<?xml version=\\"1.0\\"?>'
        f'<sdf version=\\"1.8\\">'
        f'<model name=\\"pedestal\\"><static>true</static>'
        f'<link name=\\"pedestal_link\\">'
        f'<inertial><mass>40</mass></inertial>'
        f'<visual name=\\"v\\"><geometry><mesh>'
        f'<uri>file://{pedestal_visual}</uri>'
        f'<scale>0.001 0.001 0.001</scale>'
        f'</mesh></geometry>'
        f'<material><ambient>0.75 0.75 0.80 1</ambient>'
        f'<diffuse>0.75 0.75 0.80 1</diffuse></material></visual>'
        f'<collision name=\\"c\\"><geometry><mesh>'
        f'<uri>file://{pedestal_collision}</uri>'
        f'<scale>0.001 0.001 0.001</scale>'
        f'</mesh></geometry></collision>'
        f'</link></model></sdf>'
    )

    spawn_pedestal = ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", "/world/empty/create",
            "--reqtype", "ignition.msgs.EntityFactory",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "10000",
            "--req",
            f'sdf: "{pedestal_sdf}" '
            f'pose: {{position: {{x: 0.07398, y: -1.26527, z: 0.945}} '
            f'orientation: {{x: 0.7071068, y: 0, z: 0, w: 0.7071068}}}} '
            f'name: "pedestal"',
        ],
        output="screen",
    )

    rviz_config = os.path.join(test_cell_share, "config", "unchained_only.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[robot_description],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_world",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "world"],
    )

    nodes = [
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=gazebo_resource_path),
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gazebo_resource_path),
        static_tf,
        rsp,
        gazebo,
        spawn_device,
        rviz_node,
    ]
    if include_pedestal.lower() == "true":
        nodes.insert(6, spawn_pedestal)  # before rviz_node
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("deck_offset_x", default_value="0.0",
                              description="Offset (m) applied to all deck X positions"),
        DeclareLaunchArgument("deck_offset_y", default_value="0.0",
                              description="Offset (m) applied to all deck Y positions"),
        DeclareLaunchArgument("deck_offset_z", default_value="0.0",
                              description="Offset (m) applied to all deck Z positions"),
        DeclareLaunchArgument("device_height", default_value="0.629675",
                              description="Height (m) of device CAD origin above floor"),
        DeclareLaunchArgument("include_pedestal", default_value="true",
                              description="Include the Vention pedestal"),
        OpaqueFunction(function=launch_setup),
    ])

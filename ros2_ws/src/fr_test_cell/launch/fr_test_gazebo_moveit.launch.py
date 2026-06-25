"""
Launch Gazebo + MoveIt2 + rviz2 for the test cell (FR robot on pedestal).

Gazebo: Robot + pedestal as separate SDF models (same pattern as PXRD cell)
MoveIt2: Combined URDF with pedestal as static collision object

Usage:
    ros2 launch fr_test_cell fr_test_gazebo_moveit.launch.py robot_model:=fr16
    ros2 launch fr_test_cell fr_test_gazebo_moveit.launch.py robot_model:=fr10
    ros2 launch fr_test_cell fr_test_gazebo_moveit.launch.py robot_model:=fr20
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, ExecuteProcess, OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
import xacro
import math


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
    fairino_desc_share = get_package_share_directory("fairino_description")
    moveit_share = get_package_share_directory(moveit_pkg)

    # Pedestal pose
    ped_x     = float(LaunchConfiguration("pedestal_x").perform(context))
    ped_y     = float(LaunchConfiguration("pedestal_y").perform(context))
    ped_z     = float(LaunchConfiguration("pedestal_z").perform(context))
    ped_roll  = float(LaunchConfiguration("pedestal_roll").perform(context))
    ped_pitch = float(LaunchConfiguration("pedestal_pitch").perform(context))
    ped_yaw   = float(LaunchConfiguration("pedestal_yaw").perform(context))

    # === Robot position in DEVICE frame ===
    # User specifies where the robot's "world" link (mounting plate, base_link)
    # sits inside the device's Coordinate System 1 (= SolidWorks origin).
    # All coordinates are in the device's NATIVE SolidWorks frame (Y-up).
    rid_x    = float(LaunchConfiguration("robot_in_device_x").perform(context))
    rid_y    = float(LaunchConfiguration("robot_in_device_y").perform(context))
    rid_z    = float(LaunchConfiguration("robot_in_device_z").perform(context))
    rid_yaw  = float(LaunchConfiguration("robot_in_device_yaw").perform(context))
    include_device = LaunchConfiguration("include_unchained_junior").perform(context).lower() == "true"

    # SolidWorks default convention: Y-up.
    # ROS REP-103 convention:        Z-up.
    # We apply a +90° rotation around X to the device link so that
    # device-frame Y axis (up in SW) aligns with world Z axis (up in ROS).
    #
    # With this rotation, a point (a, b, c) in device frame appears at
    # (a, -c, b) in world frame.
    # Robot is at world (0,0,0) per URDF constraint.
    # User says robot is at (rid_x, rid_y, rid_z) in device frame.
    # So: device_origin_world + (rid_x, -rid_z, rid_y) = (0, 0, 0)
    # ⇒  device_origin_world = (-rid_x, rid_z, -rid_y)
    dev_x = -rid_x
    dev_y =  rid_z
    dev_z = -rid_y
    dev_roll  = math.pi / 2.0   # 90° around X (Y-up → Z-up)
    dev_pitch = 0.0
    dev_yaw   = rid_yaw          # yaw around Y in SW = yaw around Z in ROS

    # ── Gazebo resource paths ──
    existing_path = os.environ.get("IGN_GAZEBO_RESOURCE_PATH", "")
    gazebo_resource_path = os.pathsep.join(filter(None, [
        existing_path,
        fairino_desc_share,
        os.path.dirname(fairino_desc_share),
        test_cell_share,
        os.path.dirname(test_cell_share),
    ]))

    # ══════════════════════════════════════════════════════════
    # GAZEBO: Robot alone (from original Devonics URDF)
    # ══════════════════════════════════════════════════════════
    robot_xacro = os.path.join(moveit_share, "config", f"fairino{robot_num}_v6_robot.urdf.xacro")
    robot_urdf_gazebo = xacro.process_file(
        robot_xacro,
        mappings={"control_system": "gazebo"},
    ).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"),
                         "launch", "gz_sim.launch.py")
        ]),
        launch_arguments={"gz_args": "empty.sdf -r"}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    sim_time = {"use_sim_time": True}

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_urdf_gazebo}, sim_time],
    )

    # Spawn robot ON TOP of pedestal (pedestal is 585mm tall)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_model,
            "-z", "0.585",
        ],
    )

    # ══════════════════════════════════════════════════════════
    # GAZEBO: Pedestal as separate static SDF (same pattern as PXRD)
    # ══════════════════════════════════════════════════════════
    pedestal_mesh = os.path.join(test_cell_share, "meshes", "pedestal_585mm.stl")

    # RPY → quaternion
    cr, sr = math.cos(ped_roll / 2), math.sin(ped_roll / 2)
    cp, sp = math.cos(ped_pitch / 2), math.sin(ped_pitch / 2)
    cy, sy = math.cos(ped_yaw / 2), math.sin(ped_yaw / 2)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    pedestal_sdf = (
        f'<?xml version=\\"1.0\\"?>'
        f'<sdf version=\\"1.8\\">'
        f'<model name=\\"pedestal\\"><static>true</static>'
        f'<link name=\\"pedestal_link\\">'
        f'<inertial><mass>30</mass></inertial>'
        f'<visual name=\\"v\\"><geometry><mesh>'
        f'<uri>file://{pedestal_mesh}</uri>'
        f'<scale>0.001 0.001 0.001</scale>'
        f'</mesh></geometry>'
        f'<material><ambient>0.75 0.75 0.80 1</ambient>'
        f'<diffuse>0.75 0.75 0.80 1</diffuse></material></visual>'
        f'<collision name=\\"c\\"><geometry><mesh>'
        f'<uri>file://{pedestal_mesh}</uri>'
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
            f'pose: {{position: {{x: {ped_x}, y: {ped_y}, z: {ped_z}}} '
            f'orientation: {{x: {qx:.6f}, y: {qy:.6f}, z: {qz:.6f}, w: {qw:.6f}}}}} '
            f'name: "pedestal"',
        ],
        output="screen",
    )

    # ══════════════════════════════════════════════════════════
    # GAZEBO: Unchained Junior device (separate static SDF)
    # ══════════════════════════════════════════════════════════
    device_visual_mesh = os.path.join(test_cell_share, "meshes", "unchained_junior_visual.stl")
    device_collision_mesh = os.path.join(test_cell_share, "meshes", "unchained_junior_collision.stl")

    # In Gazebo, place device 585mm higher than MoveIt frame (since pedestal
    # raises the robot 585mm in Gazebo but base_link stays at z=0 in TF)
    gz_dev_z = dev_z + 0.585

    cr, sr = math.cos(dev_roll / 2), math.sin(dev_roll / 2)
    cp, sp = math.cos(dev_pitch / 2), math.sin(dev_pitch / 2)
    cy, sy = math.cos(dev_yaw / 2), math.sin(dev_yaw / 2)
    dqw = cr * cp * cy + sr * sp * sy
    dqx = sr * cp * cy - cr * sp * sy
    dqy = cr * sp * cy + sr * cp * sy
    dqz = cr * cp * sy - sr * sp * cy

    device_sdf = (
        f'<?xml version=\\"1.0\\"?>'
        f'<sdf version=\\"1.8\\">'
        f'<model name=\\"unchained_junior\\"><static>true</static>'
        f'<link name=\\"device_link\\">'
        f'<inertial><mass>50</mass></inertial>'
        f'<visual name=\\"v\\"><geometry><mesh>'
        f'<uri>file://{device_visual_mesh}</uri>'
        f'<scale>0.001 0.001 0.001</scale>'
        f'</mesh></geometry>'
        f'<material><ambient>0.6 0.6 0.65 1</ambient>'
        f'<diffuse>0.6 0.6 0.65 1</diffuse></material></visual>'
        f'<collision name=\\"c\\"><geometry><mesh>'
        f'<uri>file://{device_collision_mesh}</uri>'
        f'<scale>0.001 0.001 0.001</scale>'
        f'</mesh></geometry></collision>'
        f'</link></model></sdf>'
    )

    spawn_device = ExecuteProcess(
        cmd=[
            "ign", "service",
            "-s", "/world/empty/create",
            "--reqtype", "ignition.msgs.EntityFactory",
            "--reptype", "ignition.msgs.Boolean",
            "--timeout", "10000",
            "--req",
            f'sdf: "{device_sdf}" '
            f'pose: {{position: {{x: {dev_x}, y: {dev_y}, z: {gz_dev_z}}} '
            f'orientation: {{x: {dqx:.6f}, y: {dqy:.6f}, z: {dqz:.6f}, w: {dqw:.6f}}}}} '
            f'name: "unchained_junior"',
        ],
        output="screen",
    )

    # ══════════════════════════════════════════════════════════
    # Gazebo controllers
    # ══════════════════════════════════════════════════════════
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
    # MOVEIT2: Combined URDF (robot + pedestal collision object)
    # ══════════════════════════════════════════════════════════
    test_urdf = os.path.join(test_cell_share, "urdf", "fr_test_cell.urdf.xacro")

    # For MoveIt2 URDF: robot base_link is at world origin, so pedestal must be
    # 585mm BELOW (z=-0.585). Gazebo uses different coords (pedestal on ground,
    # robot above) but that doesn't affect planning since TF and Gazebo can differ.
    moveit_ped_z = ped_z - 0.585  # shift down 585mm for MoveIt frame
    moveit_config = (
        MoveItConfigsBuilder(robot_name, package_name=moveit_pkg)
        .robot_description(
            file_path=test_urdf,
            mappings={
                "robot_model": robot_model,
                "control_mode": "mock",
                "pedestal_x": str(ped_x),
                "pedestal_y": str(ped_y),
                "pedestal_z": str(moveit_ped_z),
                "pedestal_roll": str(ped_roll),
                "pedestal_pitch": str(ped_pitch),
                "pedestal_yaw": str(ped_yaw),
                "device_x": str(dev_x),
                "device_y": str(dev_y),
                "device_z": str(dev_z),
                "device_roll": str(dev_roll),
                "device_pitch": str(dev_pitch),
                "device_yaw": str(dev_yaw),
                "include_unchained_junior": "true" if include_device else "false",
            },
        )
        .robot_description_semantic(file_path=f"config/fairino{robot_num}_v6_robot.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), sim_time],
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
            sim_time,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "map", "--child-frame-id", "world"],
        parameters=[sim_time],
    )

    nodes = [
        SetEnvironmentVariable(name="IGN_GAZEBO_RESOURCE_PATH", value=gazebo_resource_path),
        SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value=gazebo_resource_path),
        static_tf,
        gazebo,
        clock_bridge,
        rsp,
        spawn_robot,
        spawn_pedestal,
        joint_state_broadcaster,
        robot_controller,
        move_group,
        rviz_node,
    ]
    if include_device:
        nodes.insert(8, spawn_device)  # after spawn_pedestal
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_model", default_value="fr16",
                              description="Robot model: fr10, fr16, or fr20"),
        DeclareLaunchArgument("pedestal_x",     default_value="-0.2025"),
        DeclareLaunchArgument("pedestal_y",     default_value="0.2025"),
        DeclareLaunchArgument("pedestal_z",     default_value="0"),
        DeclareLaunchArgument("pedestal_roll",  default_value="1.5708"),
        DeclareLaunchArgument("pedestal_pitch", default_value="0"),
        DeclareLaunchArgument("pedestal_yaw",   default_value="0"),
        # === Robot position INSIDE the Unchained Junior device frame ===
        # Specify where the robot's mounting plate (base_link) sits in the
        # device's Coordinate System 1 (NATIVE SolidWorks Y-up frame, in METERS).
        # The launch automatically converts SW Y-up → ROS Z-up.
        #
        # Y axis is "up" in the device's CAD. The device's CAD origin is
        # 908.225mm above the floor (per user). The robot's mounting plate is
        # 585mm above the floor (top of 585mm pedestal). So the robot mount
        # is 323.225mm BELOW the device origin in the device's Y axis.
        # Hence the default robot_in_device_y = -0.323.
        #
        # X and Z defaults are placeholders — adjust for your physical layout.
        DeclareLaunchArgument("robot_in_device_x",   default_value="-0.8"),
        DeclareLaunchArgument("robot_in_device_y",   default_value="-0.323",
                              description="Robot mount Y offset in device frame (Y=up in SolidWorks)"),
        DeclareLaunchArgument("robot_in_device_z",   default_value="0.0"),
        DeclareLaunchArgument("robot_in_device_yaw", default_value="0.0",
                              description="Yaw rotation about device Y (= world Z) axis"),
        DeclareLaunchArgument("include_unchained_junior", default_value="true",
                              description="Include the Unchained Junior in the scene"),
        OpaqueFunction(function=launch_setup),
    ])

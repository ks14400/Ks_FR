from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

packagename = "fairino5_v6_moveit2_config"


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("fairino_resource",package_name=packagename).to_dict()

    # MTC Demo node
    pick_place_demo = Node(
        package="fairino_mtc_demo",
        executable="mtc_node",
        output="screen",
        #prefix=['gdb -ex run --args'],
        parameters=[
            moveit_config,
        ],
    )

    return LaunchDescription([pick_place_demo])

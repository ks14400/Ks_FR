#!/usr/bin/env bash
# Kill ALL ROS2 nodes/processes for this project — run this before every
# launch, especially when switching between sim (mock) and hardware (bridge).
#
# The #1 cause of "arm starts at 000000" / "go_home fails -4" is a stale
# sdk_executor (the hardware bridge) lingering from a previous session: it
# hijacks the controller action or pollutes /joint_states. This clears it.
#
# Usage:  ~/Ks_FR/ros2_ws/clean_sim.sh
set -u

echo "Killing project ROS2 processes..."
for pat in \
    sdk_executor \
    move_group \
    rviz2 \
    ros2_control_node \
    controller_manager \
    spawner \
    robot_state_publisher \
    static_transform_publisher \
    unchained_full \
    bridge.launch \
    "fr_test_cell" \
    ros2launch ; do
    pkill -9 -f "$pat" 2>/dev/null && echo "  killed: $pat"
done

sleep 3

echo "Remaining nodes (should be empty):"
ros2 node list 2>/dev/null || true
echo "Done. Safe to launch."

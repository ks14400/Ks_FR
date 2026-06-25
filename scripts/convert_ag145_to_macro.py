#!/usr/bin/env python3
"""
Convert dh_gripper_ros AG-145 URDF into a clean xacro macro for our package.

Steps:
  1. Read the upstream AG-145 URDF
  2. Strip the world link and world_fixed joint
  3. Add prefix to all link and joint names
  4. Update mesh paths to use package://fr_test_cell/meshes/dh_ag145/
  5. Wrap in <xacro:macro name="dh_ag145_gripper" params="prefix parent *origin grasp_link_offset:=0.143">
  6. Add the parent joint using ${parent} and the *origin block
  7. Add a grasp_link (TCP) at the user-specified offset
"""
import re
import sys

UPSTREAM_URDF = (
    "/home/aimatx_nuc3/Ks_FR/dh_gripper_ros-f59f9c2f4bc8eb116448b1d798791424bf64e337/"
    "dh_gripper_ros-f59f9c2f4bc8eb116448b1d798791424bf64e337/"
    "dh_robotics_ag145_gripper/dh_robotics_ag145_description/urdf/dh_robotics_ag145.urdf"
)
OUTPUT = "/home/aimatx_nuc3/ros2_ws/src/fr_test_cell/urdf/dh_ag145_macro.xacro"


def main():
    with open(UPSTREAM_URDF) as f:
        content = f.read()

    # Strip the <link name="world"/> definition
    content = re.sub(r'\s*<link\s+name="world"\s*/>\s*\n', "\n", content, count=1)
    # Strip the world_fixed joint
    content = re.sub(
        r'\s*<joint\s+name="world_fixed"[^>]*>.*?</joint>\s*\n',
        "\n", content, count=1, flags=re.DOTALL,
    )

    # Update mesh paths
    content = content.replace(
        'package://dh_robotics_ag145_description/meshes/visual/',
        'package://fr_test_cell/meshes/dh_ag145/',
    )

    # Add prefix to all link and joint names (matching name attribute)
    # We use string replacement for the most common link/joint names
    link_names = [
        "base_link",
        "finger1_knuckle_link", "finger1_finger_link",
        "finger1_inner_knuckle_link", "finger1_finger_tip_link",
        "finger2_knuckle_link", "finger2_finger_link",
        "finger2_inner_knuckle_link", "finger2_finger_tip_link",
    ]
    joint_names = [
        "finger1_joint", "finger1_finger_joint",
        "finger1_inner_knuckle_joint", "finger1_finger_tip_joint",
        "finger2_joint", "finger2_finger_joint",
        "finger2_inner_knuckle_joint", "finger2_finger_tip_joint",
    ]

    # Replace name="X" with name="${prefix}X" and link references too
    for name in link_names + joint_names:
        # name="foo" → name="${prefix}foo"
        content = re.sub(
            rf'(\bname=)"{re.escape(name)}"',
            rf'\1"${{prefix}}{name}"',
            content,
        )
        # parent link="foo" → parent link="${prefix}foo"
        content = re.sub(
            rf'(\b(?:parent|child)\s+link=)"{re.escape(name)}"',
            rf'\1"${{prefix}}{name}"',
            content,
        )
        # mimic joint="foo" → mimic joint="${prefix}foo"
        content = re.sub(
            rf'(\bmimic\s+joint=)"{re.escape(name)}"',
            rf'\1"${{prefix}}{name}"',
            content,
        )

    # Extract just the body content (everything between <robot> tags)
    # The upstream wraps in <robot name="..."> ... </robot>
    body_match = re.search(r'<robot\s+[^>]*>(.*?)</robot>', content, re.DOTALL)
    if not body_match:
        print("ERROR: Could not find <robot> body", file=sys.stderr)
        sys.exit(1)
    body = body_match.group(1)

    # Build the macro
    macro = f'''<?xml version="1.0"?>
<!--
  DH AG-145 gripper macro.
  Adapted from upstream dh_gripper_ros AG-145 URDF.

  Usage:
    <xacro:include filename="$(find fr_test_cell)/urdf/dh_ag145_macro.xacro"/>
    <xacro:dh_ag145_gripper parent="wrist3_link" prefix="gripper_">
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </xacro:dh_ag145_gripper>

  Parameters:
    parent             - link to attach the gripper to
    prefix             - name prefix for all gripper links/joints (e.g. "gripper_")
    *origin            - block defining xyz/rpy of gripper base relative to parent
    grasp_link_offset  - Z offset (m) from gripper base_link to grasp_link (TCP).
                         Default 0.143 m = 5mm + ~138mm gripper Z extent.
-->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="dh_ag145_gripper"
               params="prefix parent *origin grasp_link_offset:=0.143">

    <!-- Gripper body (from upstream AG-145 URDF) -->
{body}

    <!-- Attach gripper base_link to the parent (e.g. wrist3_link) -->
    <joint name="${{prefix}}gripper_base_joint" type="fixed">
      <parent link="${{parent}}"/>
      <child link="${{prefix}}base_link"/>
      <xacro:insert_block name="origin"/>
    </joint>

    <!-- TCP (grasp_link) — centered between fingers, at grasp_link_offset from base -->
    <link name="${{prefix}}grasp_link"/>
    <joint name="${{prefix}}grasp_joint" type="fixed">
      <parent link="${{prefix}}base_link"/>
      <child link="${{prefix}}grasp_link"/>
      <origin xyz="0 0 ${{grasp_link_offset}}" rpy="0 0 0"/>
    </joint>

  </xacro:macro>

</robot>
'''

    with open(OUTPUT, "w") as f:
        f.write(macro)
    print(f"Wrote: {OUTPUT}")
    print(f"  Body lines: {body.count(chr(10))}")


if __name__ == "__main__":
    main()

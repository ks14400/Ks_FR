# Adding Test Objects to the Test Cell

To test collision avoidance or pick-and-place with custom objects, add links
and joints to `urdf/fr_test_cell.urdf.xacro` before the closing `</robot>` tag.

## Example: Red Box

```xml
<link name="test_box">
  <visual>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry><box size="0.1 0.1 0.1"/></geometry>
    <material name="red"><color rgba="1 0 0 1"/></material>
  </visual>
  <collision>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <geometry><box size="0.1 0.1 0.1"/></geometry>
  </collision>
  <inertial>
    <mass value="1.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<joint name="test_box_joint" type="fixed">
  <parent link="world"/>
  <child link="test_box"/>
  <origin xyz="0.6 0 0" rpy="0 0 0"/>   <!-- 600mm forward of robot -->
</joint>
```

## Coordinate System

- `world` is at the **top of the pedestal** (robot mounting surface)
- The floor is at `z = -0.585` (below world)
- `+X` = forward from the robot's starting orientation
- `+Y` = left
- `+Z` = up

## Example: Cylinder

```xml
<link name="test_cylinder">
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry><cylinder radius="0.05" length="0.2"/></geometry>
    <material name="blue"><color rgba="0 0 1 1"/></material>
  </visual>
  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0"/>
    <geometry><cylinder radius="0.05" length="0.2"/></geometry>
  </collision>
  <inertial>
    <mass value="0.5"/>
    <inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/>
  </inertial>
</link>

<joint name="test_cylinder_joint" type="fixed">
  <parent link="world"/>
  <child link="test_cylinder"/>
  <origin xyz="0.5 0.2 0" rpy="0 0 0"/>
</joint>
```

## Example: Mesh Object (e.g., from CAD)

```xml
<link name="test_mesh">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://fr_test_cell/meshes/my_object.stl"
            scale="0.001 0.001 0.001"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://fr_test_cell/meshes/my_object.stl"
            scale="0.001 0.001 0.001"/>
    </geometry>
  </collision>
  <inertial>
    <mass value="2.0"/>
    <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
  </inertial>
</link>

<joint name="test_mesh_joint" type="fixed">
  <parent link="world"/>
  <child link="test_mesh"/>
  <origin xyz="0.5 0 0" rpy="0 0 0"/>
</joint>
```

For meshes, place STL files in `fr_test_cell/meshes/` and update `setup.py`
to install them:
```python
(os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
```

## After Adding

```bash
cd ~/ros2_ws
colcon build --packages-select fr_test_cell
source install/setup.bash
ros2 launch fr_test_cell fr_test_gazebo_moveit.launch.py
```

MoveIt2 will automatically include the new objects in collision checking.

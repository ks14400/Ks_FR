# Phase 3 — PXRD Cell Scene with Collision Meshes

Add the PXRD instrument, sample holders, and any fixtures to the simulation as
collision objects. MoveIt2 will then plan paths that avoid these objects automatically.
This is the core value of the ROS2 approach — collision-free planning against real
CAD geometry.

---

## Why This Phase

Without collision meshes, MoveIt2 plans paths straight through the PXRD instrument.
With them, every path is guaranteed to avoid the instrument, sample holders, table,
and any other objects in the cell.

---

## Step 1: Convert CAD to Collision Meshes

STEP files are the canonical CAD source. Convert them to DAE meshes for URDF.

```bash
# Install FreeCAD for STEP → DAE conversion
sudo snap install freecad

# Convert to low-poly collision mesh (~10% polygons — faster planning)
python3 tools/cad_to_urdf/convert_step.py \
    --input ~/cad/pxrd_instrument.step \
    --output ~/ros2_ws/src/pxrd_cell/meshes/pxrd_instrument_collision.dae \
    --simplify 0.1

# Optional: full-resolution visual mesh (for Gazebo display)
python3 tools/cad_to_urdf/convert_step.py \
    --input ~/cad/pxrd_instrument.step \
    --output ~/ros2_ws/src/pxrd_cell/meshes/pxrd_instrument_visual.dae
```

Fairino CAD models: `https://www.fairino.com/DOWNLOAD2`
3D STEP models available: FRCobots V5.0/V6.0, control boxes, workstations, envelope diagrams.

---

## Step 2: Create the `pxrd_cell` Package

```bash
cd ~/ros2_ws/src
ros2 pkg create pxrd_cell --build-type ament_python
```

### Package Structure

```
~/ros2_ws/src/pxrd_cell/
├── pxrd_cell/
│   ├── __init__.py
│   ├── job_runner.py              # Phase 4
│   ├── waypoint.py                # Phase 4
│   ├── gripper_interface.py       # Phase 4
│   └── pxrd_trigger.py            # Phase 4
├── urdf/
│   └── pxrd_cell.urdf.xacro       # Robot + PXRD instrument + fixtures
├── meshes/
│   ├── pxrd_instrument_collision.dae
│   ├── pxrd_instrument_visual.dae
│   ├── sample_holder_collision.dae
│   └── table_collision.dae
├── config/
│   └── pxrd_planning_scene.yaml    # Object poses in world frame
├── launch/
│   └── pxrd_moveit.launch.py       # MoveIt2 with full cell scene
├── jobs/                            # Job YAML files (Phase 4)
├── setup.py
└── package.xml
```

---

## Step 3: Write the Cell URDF

`urdf/pxrd_cell.urdf.xacro` includes the Fairino robot URDF and adds static objects.
**Never edit** `fairino_description/` — include it via xacro.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="pxrd_cell">

  <!-- Include Fairino robot -->
  <xacro:include filename="$(find fairino_description)/urdf/fairino16_v6.urdf.xacro"/>

  <!-- PXRD Instrument — positioned relative to robot base -->
  <link name="pxrd_instrument">
    <visual>
      <geometry>
        <mesh filename="package://pxrd_cell/meshes/pxrd_instrument_visual.dae"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <mesh filename="package://pxrd_cell/meshes/pxrd_instrument_collision.dae"/>
      </geometry>
    </collision>
  </link>

  <joint name="pxrd_instrument_joint" type="fixed">
    <parent link="base_link"/>
    <child link="pxrd_instrument"/>
    <!-- Measure actual position: X, Y, Z in meters -->
    <origin xyz="0.0 0.5 0.0" rpy="0 0 0"/>
  </joint>

  <!-- Add more objects: table, sample holders, fixtures... -->

</robot>
```

---

## Step 4: Configure the Planning Scene

`config/pxrd_planning_scene.yaml` defines object positions for the MoveIt2 planning
scene. Measure from the physical cell layout.

```yaml
pxrd_instrument:
  mesh: package://pxrd_cell/meshes/pxrd_instrument_collision.dae
  pose:
    position: {x: 0.0, y: 0.5, z: 0.0}     # meters, robot base frame
    orientation: {r: 0.0, p: 0.0, y: 0.0}   # radians

sample_holder:
  mesh: package://pxrd_cell/meshes/sample_holder_collision.dae
  pose:
    position: {x: 0.3, y: -0.2, z: 0.0}
    orientation: {r: 0.0, p: 0.0, y: 0.0}
```

---

## Step 5: Write the Launch File

`launch/pxrd_moveit.launch.py` launches MoveIt2 with the full cell scene loaded.
It should:
- Load the `pxrd_cell` URDF (robot + all collision objects)
- Start the MoveIt2 move_group node
- Load the planning scene from YAML
- Launch rviz2 with the planning panel

---

## Step 6: Build and Verify

```bash
colcon build --symlink-install --packages-select pxrd_cell
source install/setup.bash

# Launch MoveIt2 with the cell scene
ros2 launch pxrd_cell pxrd_moveit.launch.py
```

### Verify in rviz2:
1. PXRD instrument mesh is visible in the planning scene
2. Set a target pose near the instrument
3. Click **Plan** — the path should route around the instrument
4. Move the target to inside the instrument — planning should **fail** (collision)
5. Check that joint limits match the Fairino hardware spec

---

## Phase 3 Completion Checklist

- [ ] PXRD instrument STEP file converted to collision DAE
- [ ] `pxrd_cell` package created with URDF, meshes, config, launch
- [ ] Cell URDF includes robot + PXRD instrument via xacro
- [ ] Physical positions measured and encoded in URDF/config
- [ ] `pxrd_moveit.launch.py` loads MoveIt2 with collision scene
- [ ] rviz2 shows instrument; plans avoid it
- [ ] Plans that would collide are correctly rejected
- [ ] Joint limits cross-checked against Fairino hardware spec

---

## Rules

- **Always load collision meshes before any motion plan**
- Collision meshes: ~10% polygon count. Visual meshes: full resolution.
- Never edit `fairino_description/` URDFs — include via xacro
- Re-run CAD conversion if source STEP files change
- STEP is canonical — never edit DAE/STL directly

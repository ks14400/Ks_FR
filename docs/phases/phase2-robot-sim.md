# Phase 2 — Fairino Robot in Gazebo + MoveIt2 Planning

Clone the Devonics and Fairino repos, build them, and get the FR16/FR20 robot model
visible in Gazebo and controllable via MoveIt2. By the end of this phase you can
plan and execute motions on a simulated robot — no physical hardware needed.

---

## Why This Phase

Devonics (`ros2_fr_gazebo`) and Fairino (`frcobot_ros2`) provide everything we need:
- Robot URDF with visual and collision meshes
- Gazebo Fortress simulation (digital twin + mirror modes)
- MoveIt2 configuration with planning groups and controllers
- `fairino_hardware` ros2_control plugin for real robot connection later

We clone these repos, build them, and verify the simulation works before adding
our PXRD cell scene in Phase 3.

---

## Step 1: Create Workspace and Clone Repos

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_control_ws/install/setup.bash

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Fairino official — URDF, MoveIt2 configs, hardware plugin
git clone https://github.com/FAIR-INNOVATION/frcobot_ros2
cd frcobot_ros2 && git checkout <tag matching firmware 3.8.x> && cd ..

# Devonics — Gazebo simulation environment
git clone https://github.com/Devonics-Inc/ros2_fr_gazebo
```

### What Each Repo Gives Us

**frcobot_ros2** (Fairino official):
- `fairino_msgs/` — custom ROS2 message types
- `fairino_hardware/` — C++ ros2_control plugin (`FairinoHardwareInterface`)
  - Compiled output: `libfairino_hardware.so`
  - Plugin registered as `fairino_hardware/FairinoHardwareInterface`
  - Supports position mode (default) and torque mode
- `fairino_description/` — URDF + meshes for FR3, FR5, FR10, FR16, FR20, FR30
- `fairino{N}_v6_moveit2_config/` — pre-built MoveIt2 config per robot model
- `fairino_mtc_demo/` — MoveIt Task Constructor pick-and-place sample

**ros2_fr_gazebo** (Devonics):
- `fairino_gazebo_config/` — Gazebo Fortress worlds, launch files, `rt_state_data.py`
- Three operational modes:
  1. **Digital twin** — fully simulated, no hardware needed
  2. **Mirror** — Gazebo mirrors real/SimMachine robot state
  3. **MoveIt2 demo** — rviz2 only, no Gazebo

---

## Step 2: Fix IP Address Configuration

Devonics defaults to `192.168.55.2`. Fairino uses `192.168.58.2`. Update to match
your robot/SimMachine IP in these three files:

```
src/ros2_fr_gazebo/fairino_hardware/include/fairino_hardware/data_types_def.h
src/ros2_fr_gazebo/fairino_gazebo_config/launch/rt_state_data.py
src/ros2_fr_gazebo/fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp
```

---

## Step 3: Build (Order Matters!)

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build in this order — fairino_msgs must come first
colcon build --packages-select fairino_msgs
source install/setup.bash

colcon build --packages-select fairino_hardware
source install/setup.bash

colcon build --packages-select fairino_description
source install/setup.bash

# Build MoveIt2 config for your robot model (FR16 example)
colcon build --packages-select fairino16_v6_moveit2_config
source install/setup.bash

# Add to bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 4: Launch Digital Twin (No Hardware Needed)

This is the mode we'll use for development — a fully simulated robot.

```bash
ros2 launch fairino16_v6_moveit2_config digital_fr16_gazebo_sim.launch.py
```

You should see:
- Gazebo window with the FR16 robot
- rviz2 with MoveIt2 motion planning panel

### Test Motion Planning

In rviz2:
1. Drag the blue sphere at the robot's end-effector to set a target pose
2. Use the red/green/blue rings to change orientation
3. Click **Plan** to compute a trajectory
4. Click **Execute** to move the simulated robot
5. Click **Plan & Execute** to do both in one step

---

## Step 5: Test MoveIt2 Demo (rviz2 only)

```bash
ros2 launch fairino16_v6_moveit2_config demo.launch.py
```

### Hardware Plugin Configuration

In the ros2_control xacro file for your model:
- `use_fake_hardware:=true` → mock mode (no robot connection, for simulation)
- `use_fake_hardware:=false` → uses `fairino_hardware/FairinoHardwareInterface` (real robot)

The `robot_control_mode` parameter:
- `0` → position mode (default, exports position command interface)
- `1` → torque mode (exports effort command interface, requires firmware ≥ v3.8.3)

---

## Step 6: Test MTC Demo (Optional but Recommended)

The official `fairino_mtc_demo` package is a working pick-and-place cycle — good
reference for our Phase 4 application code.

```bash
# Terminal 1: Launch rviz2 with MTC interface
ros2 launch fairino_mtc_demo mtc_demo_env.launch.py

# Terminal 2: Execute the pick-and-place motion
ros2 launch fairino_mtc_demo mtc_demo_app.launch.py
```

Select your robot model in `mtc_demo_env.launch.py` lines 9-11.

---

## ROS2 Control Architecture

```
MoveIt2 planner
    ↓ (JointTrajectory action)
moveit_controllers.yaml
    ↓
JointTrajectoryController (ros2_controllers.yaml)
    ↓
ros2_control controller manager
    ↓
FairinoHardwareInterface plugin  ←or→  mock_components/GenericSystem
    ↓                                        ↓
TCP/IP to real robot                    simulated joints
```

- Joint names: `j1, j2, j3, j4, j5, j6` (base to end-effector)
- Controller names in `moveit_controllers.yaml` **must match** `ros2_controllers.yaml`
- Joint data type (position/velocity/effort) must match the plugin mode

---

## ROS2 Topics and Services

```bash
# View robot state feedback
ros2 topic echo /nonrt_state_data

# Start command server (for sending SDK-like commands via ROS2)
ros2 run fairino_hardware ros2_cmd_server

# Send commands via rqt service caller
rqt
# Plugins -> Service -> Service Caller -> /fairino_remote_command_service
# Input format: FunctionName(arg1,arg2,...)
```

---

## Phase 2 Completion Checklist

- [ ] `frcobot_ros2` and `ros2_fr_gazebo` cloned with correct firmware tags
- [ ] IP addresses updated in all three config files
- [ ] `colcon build` succeeds (fairino_msgs → fairino_hardware → description → moveit2_config)
- [ ] Digital twin launches: robot visible in Gazebo
- [ ] MoveIt2 demo launches: can plan and execute in rviz2
- [ ] Robot moves in simulation when executing a MoveIt2 plan
- [ ] (Optional) MTC demo runs pick-and-place cycle

---

## Known Issues

- **Build order matters**: fairino_msgs → fairino_hardware → fairino_description → moveit2_config
- If Gazebo shows entity but no model:
  `export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/ros2_ws/install/fairino_description/share`
- If "cannot find SimJointPublisher.py": create empty `__pycache__/` in launch dir and rebuild
- Controller names must match between `moveit_controllers.yaml` and `ros2_controllers.yaml`
- If robot doesn't move after Plan & Execute: check `use_fake_hardware` setting
- Verify `libfairino_hardware.so` exists at `ros2_ws/install/fairino_hardware/lib/`
- After `demo.launch.py`, if launches.py errors: remove line 203
  (`default_value=moveit_config.move_group_capabilities["capabilities"]`)
- Checksum validation failures with SimMachine: see [frcobot_ros2#12](https://github.com/FAIR-INNOVATION/frcobot_ros2/issues/12)

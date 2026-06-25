# Setup Guide — Fairino FR16 PXRD Pick-and-Place System

Complete setup from bare Ubuntu 22.04 to a working FR16 simulation with MoveIt2
motion planning. Covers Phase 1 (toolchain) and Phase 2 (robot in simulation).

**Tested on:** Ubuntu 22.04.5 LTS, x86_64, April 2026

---

## Prerequisites

- Ubuntu 22.04 LTS (Jammy Jellyfish), x86_64
- ~15 GB free disk space
- sudo access
- Internet connection

---

## Phase 1 — ROS2 Toolchain Installation

### 1.1 Locale

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

### 1.2 ROS2 Humble

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools
```

Add to `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
```

Verify:
```bash
source ~/.bashrc
ros2 topic list
# Should show /rosout and /parameter_events
```

### 1.3 MoveIt2

```bash
sudo apt install -y ros-humble-moveit
```

Verify:
```bash
ros2 pkg list | grep moveit | wc -l
# Should show 20+ packages
```

### 1.4 Gazebo Fortress

```bash
sudo apt-get install -y lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update && sudo apt-get install -y ignition-fortress
```

Verify:
```bash
ign gazebo --version
# Should show version 6.x
```

### 1.5 ros2_control + Gazebo Bridge

```bash
sudo apt install -y ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install -y ros-humble-ign-ros2-control ros-humble-ros-gz-sim
```

> **Note:** The phase doc recommends building ros2_control from source. We use apt
> instead because the source CI repos target newer ROS2 distros (Rolling/Jazzy) and
> fail to build on Humble. The apt packages (v2.53.1) are stable and complete for
> Humble. Switch to source build only if you hit a specific missing feature.

### 1.6 Build Tools

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init   # skip if "already initialized"
rosdep update
```

### 1.7 Phase 1 Verification

```bash
source /opt/ros/humble/setup.bash
ros2 topic list                                    # /rosout, /parameter_events
ros2 pkg list | grep moveit | wc -l               # 20+
ign gazebo --version                               # 6.x
ros2 pkg list | grep ros2_control                  # ros2_control listed
which colcon vcs rosdep                            # all found
```

---

## Phase 2 — FR16 Robot in Gazebo + MoveIt2

### 2.1 Key Decision: Use ros2_fr_gazebo (Not frcobot_ros2)

The Devonics `ros2_fr_gazebo` repo is **self-contained**. It bundles:
- `fairino_msgs` — custom ROS2 messages
- `fairino_hardware` — C++ ros2_control plugin + libfairino SDK
- `fairino_description` — URDF + meshes for all robot models
- `fairino{N}_v6_moveit2_config` — MoveIt2 configs (FR3, 5, 10, 16, 20, 30)
- `fairino_gazebo_config` — Gazebo launch files, digital twin, mirror mode

The standalone `frcobot_ros2` repo does NOT include Gazebo/digital twin launch files.

**Do NOT clone both repos** — they contain duplicate package names and colcon will
get confused.

### 2.2 Create Workspace and Clone

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Devonics-Inc/ros2_fr_gazebo.git
```

Stay on `main` branch. No v3.8.6-specific tag exists. Firmware version matching
is only critical for Phase 5 (real hardware). The digital twin does not connect to
hardware.

### 2.3 Verify Repo Layout

```bash
ls ~/ros2_ws/src/ros2_fr_gazebo/src/
```

Should contain:
```
fairino_msgs/
fairino_hardware/
fairino_description/
fairino_gazebo_config/
fairino3_v6_moveit2_config/
fairino5_v6_moveit2_config/
fairino10_v6_moveit2_config/
fairino16_v6_moveit2_config/
fairino20_v6_moveit2_config/
fairino30_v6_moveit2_config/
```

Check for the FR16 digital twin launch file:
```bash
find ~/ros2_ws/src -name "*fr16*gazebo*" -o -name "*digit*fr16*"
```

> **Note:** The filename may have a typo: `digitial_fr16_gazebo_sim.launch.py`
> instead of `digital_fr16_gazebo_sim.launch.py`.

### 2.4 Update IP Addresses

Change `192.168.55.2` to `192.168.58.2` in these 3 files:

**File 1:** `src/ros2_fr_gazebo/src/fairino_hardware/include/fairino_hardware/data_type_def.h`
```c
// Change line ~17:
#define CONTROLLER_IP "192.168.58.2"
```

**File 2:** `src/ros2_fr_gazebo/src/fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp`
```cpp
// Change line ~15:
#define CONTROLLER_IP_ADDRESS "192.168.58.2"
```

**File 3:** `src/ros2_fr_gazebo/src/fairino_gazebo_config/launch/rt_state_data.py`
```python
# Change line ~16:
ROBOT_IP = "192.168.58.2"
```

> Not strictly needed for digital twin (no hardware connection), but prepares for
> Phase 5 mirror mode and real hardware.

### 2.5 Apply Known Workarounds

**SimJointPublisher.py fix:**
```bash
mkdir -p ~/ros2_ws/src/ros2_fr_gazebo/src/fairino_gazebo_config/launch/__pycache__
```

### 2.6 Install Dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2.7 Build (Order Matters!)

```bash
cd ~/ros2_ws

colcon build --packages-select fairino_msgs
source install/setup.bash

colcon build --packages-select fairino_hardware
source install/setup.bash

colcon build --packages-select fairino_description
source install/setup.bash

colcon build --packages-select fairino16_v6_moveit2_config
source install/setup.bash

colcon build --packages-select fairino_gazebo_config
source install/setup.bash
```

Verify build artifacts:
```bash
ls ~/ros2_ws/install/fairino_hardware/lib/libfairino_hardware.so
ls ~/ros2_ws/install/fairino_description/share/fairino_description/meshes/fairino16_v6/
```

### 2.8 Configure Environment

Add to `~/.bashrc` (after the ROS2 Humble source line):
```bash
source ~/ros2_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/ros2_ws/install/fairino_description/share
```

Then: `source ~/.bashrc`

> Remove any stale `source ~/ros2_control_ws/install/setup.bash` lines if present.

### 2.9 Launch Digital Twin (No Hardware Needed)

```bash
ros2 launch fairino16_v6_moveit2_config digital_fr16_gazebo_sim.launch.py
```

Expected:
- Gazebo Fortress window with FR16 robot visible
- rviz2 with MoveIt2 Motion Planning panel
- No red errors in terminal

**Troubleshooting:**
- Robot invisible in Gazebo → check `IGN_GAZEBO_RESOURCE_PATH`
- Controller errors → verify `ros-humble-ign-ros2-control` is installed
- Controller name mismatch → `moveit_controllers.yaml` must match `ros2_controllers.yaml`

### 2.10 Test Motion Planning

In rviz2:
1. Drag the blue sphere (end-effector marker) to a target pose
2. Use red/green/blue rings to change orientation
3. Click **Plan** — green trajectory should appear
4. Click **Execute** — robot moves in Gazebo
5. Click **Plan & Execute** — both in one step

CLI verification:
```bash
ros2 topic echo /joint_states --once          # 6 joint positions
ros2 control list_controllers                  # fairino16_controller active
```

### 2.11 Launch MoveIt2 Demo (rviz2 Only)

```bash
ros2 launch fairino16_v6_moveit2_config demo.launch.py
```

Uses `mock_components/GenericSystem` — no Gazebo, robot moves in rviz2 only.

**Known issue:** May error on `launches.py` line 203:
```
KeyError: 'capabilities'
```
**Fix:** Edit the system file:
```bash
sudo nano /opt/ros/humble/lib/python3.10/site-packages/moveit_configs_utils/launches.py
```
Comment out or remove line 203:
```python
# default_value=moveit_config.move_group_capabilities["capabilities"]
```

### 2.12 (Optional) MTC Demo

Reference pick-and-place demo using MoveIt Task Constructor:

```bash
# Install MTC
sudo apt install -y ros-humble-moveit-task-constructor-core

# Copy demo into workspace
cp -r /path/to/Ks_FR/fairino_mtc_demo ~/ros2_ws/src/
```

Update launch files for FR16 — edit both files in `fairino_mtc_demo/launch/`:

`mtc_demo_env.launch.py` (lines 9-11):
```python
robotname = "fairino16_v6_robot"
packagename = "fairino16_v6_moveit2_config"
ros2controllername = "fairino16_controller"
```

`mtc_demo_app.launch.py` (line 5):
```python
packagename = "fairino16_v6_moveit2_config"
```

Build and run:
```bash
cd ~/ros2_ws
colcon build --packages-select fairino_mtc_demo
source install/setup.bash

# Terminal 1: Environment (rviz2 + controllers)
ros2 launch fairino_mtc_demo mtc_demo_env.launch.py

# Terminal 2: Application (pick-and-place cycle)
ros2 launch fairino_mtc_demo mtc_demo_app.launch.py
```

### 2.13 Phase 2 Verification Checklist

- [ ] `~/ros2_ws/install/` contains fairino_msgs, fairino_hardware, fairino_description,
      fairino16_v6_moveit2_config, fairino_gazebo_config
- [ ] IP `192.168.58.2` in all 3 config files
- [ ] `libfairino_hardware.so` exists in install dir
- [ ] FR16 meshes installed
- [ ] `IGN_GAZEBO_RESOURCE_PATH` set in bashrc
- [ ] Digital twin launches: robot visible in Gazebo
- [ ] MoveIt2 Plan & Execute moves robot in Gazebo
- [ ] `ros2 topic echo /joint_states --once` returns 6 joints
- [ ] `ros2 control list_controllers` shows fairino16_controller active
- [ ] (Optional) MTC demo runs pick-and-place cycle

---

## Final ~/.bashrc Lines

After completing both phases, your bashrc should include:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:~/ros2_ws/install/fairino_description/share
```

---

## Quick Reference — Ports and IPs

| Service | Port | Protocol |
|---|---|---|
| SDK commands | 20003 | XML-RPC |
| SDK state feedback | 20004 | UDP (~10 Hz) |
| ros2_control commands | 8080 | TCP/IP |
| ros2_control status | 8083 | TCP/IP (~10 Hz) |
| Default robot IP | 192.168.58.2 | — |

---

## Troubleshooting

| Problem | Fix |
|---|---|
| `ign: command not found` | Gazebo Fortress not installed. Run 1.4. |
| `launches.py` line 203 KeyError | Comment out line 203 in system `launches.py`. See 2.11. |
| Robot invisible in Gazebo | Set `IGN_GAZEBO_RESOURCE_PATH`. See 2.8. |
| "cannot find SimJointPublisher.py" | Create `__pycache__/` in gazebo launch dir. See 2.5. |
| ros2_control source build fails | Use apt install instead. See 1.5. |
| colcon build order errors | Build fairino_msgs first, then fairino_hardware, then rest. See 2.7. |
| Controller name mismatch | `moveit_controllers.yaml` names must match `ros2_controllers.yaml` |
| `AMENT_TRACE_SETUP_FILES: unbound variable` | Don't use `set -u` before sourcing ROS2 setup.bash |

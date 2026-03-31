# Phase 1 — ROS2 + MoveIt2 + Gazebo Installation

Install the full ROS2 toolchain needed for simulation-based pick-and-place development.
By the end of this phase you have ROS2 Humble, MoveIt2, Gazebo Fortress, and
ros2_control all working on your machine.

---

## Why This Phase

Before we can simulate the robot or plan collision-free paths, we need the ROS2
ecosystem installed. This phase is purely environment setup — no robot code yet.

---

## System Requirements

- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- x86_64 architecture
- ~10 GB disk space for ROS2 + Gazebo + MoveIt2

---

## Step 1: Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repo
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools

# Source it
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Verify: `ros2 topic list` should return `/rosout` and `/parameter_events`.

---

## Step 2: Install ros2_control (Source Build)

The official manual recommends **source build** — the apt install may leave packages
missing, which causes silent failures later.

```bash
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_control_ws/src
cd ~/ros2_control_ws/
wget https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.$ROS_DISTRO.repos
vcs import src < ros_controls.$ROS_DISTRO.repos

rosdep update --rosdistro=$ROS_DISTRO
sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y

. /opt/ros/${ROS_DISTRO}/setup.sh
colcon build --symlink-install

echo "source ~/ros2_control_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 3: Install MoveIt2

```bash
sudo apt install ros-humble-moveit
```

Verify: `ros2 pkg list | grep moveit` should show multiple moveit packages.

---

## Step 4: Install Gazebo Fortress

```bash
sudo apt-get install lsb-release gnupg
sudo curl https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update && sudo apt-get install ignition-fortress
```

---

## Step 5: Install Additional Dependencies

```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init   # skip if already initialized
rosdep update
```

---

## Phase 1 Completion Checklist

- [ ] Ubuntu 22.04 LTS
- [ ] `ros2 topic list` works
- [ ] `ros2_control_ws` built from source, sourced in bashrc
- [ ] `ros2 pkg list | grep moveit` shows packages
- [ ] `ign gazebo` launches Gazebo Fortress
- [ ] `colcon build` available

---

## Known Issues

- If `vcs import` fails: install `python3-vcstool` via apt
- If rosdep init says "already initialized": that's fine, just run `rosdep update`
- ros2_control apt install may miss packages — always use source build

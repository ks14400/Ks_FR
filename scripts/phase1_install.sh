#!/usr/bin/env bash
# Phase 1 — ROS2 Humble + MoveIt2 + Gazebo Fortress + ros2_control
# Run with: bash scripts/phase1_install.sh
# Requires sudo privileges.
set -eo pipefail

echo "========================================="
echo "Phase 1: ROS2 Toolchain Installation"
echo "========================================="

# ── Step 1: Locale ──────────────────────────
echo "[1/7] Setting locale..."
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# ── Step 2: ROS2 Humble apt repo ────────────
echo "[2/7] Adding ROS2 Humble repository..."
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ── Step 3: Install ROS2 Humble Desktop ─────
echo "[3/7] Installing ROS2 Humble Desktop (this takes a while)..."
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-humble-desktop ros-dev-tools

# Source ROS2
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi
source /opt/ros/humble/setup.bash

# ── Step 4: Install MoveIt2 ─────────────────
echo "[4/7] Installing MoveIt2..."
sudo apt install -y ros-humble-moveit

# ── Step 5: Install Gazebo Fortress ──────────
echo "[5/7] Installing Gazebo Fortress..."
sudo apt-get install -y lsb-release gnupg
sudo curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
    --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
    https://packages.osrfoundation.org/gazebo/ubuntu-stable \
    $(lsb_release -cs) main" | \
    sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update && sudo apt-get install -y ignition-fortress

# ── Step 6: Install build tools ──────────────
echo "[6/7] Installing build tools (colcon, rosdep, vcstool)..."
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init 2>/dev/null || echo "rosdep already initialized"
rosdep update

# ── Step 7: Build ros2_control from source ───
echo "[7/7] Building ros2_control from source..."
source /opt/ros/humble/setup.bash

mkdir -p ~/ros2_control_ws/src
cd ~/ros2_control_ws/
wget -q https://raw.githubusercontent.com/ros-controls/ros2_control_ci/master/ros_controls.${ROS_DISTRO}.repos
vcs import src < ros_controls.${ROS_DISTRO}.repos

sudo apt-get update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

if ! grep -q "source ~/ros2_control_ws/install/setup.bash" ~/.bashrc; then
    echo "source ~/ros2_control_ws/install/setup.bash" >> ~/.bashrc
fi

# ── Verification ─────────────────────────────
echo ""
echo "========================================="
echo "Verification"
echo "========================================="
source /opt/ros/humble/setup.bash
source ~/ros2_control_ws/install/setup.bash

echo -n "ROS2:       "; ros2 --version 2>/dev/null && echo "OK" || echo "FAILED"
echo -n "MoveIt2:    "; ros2 pkg list 2>/dev/null | grep -q moveit && echo "OK" || echo "FAILED"
echo -n "Gazebo:     "; ign gazebo --version 2>/dev/null | head -1 && echo "OK" || echo "FAILED"
echo -n "colcon:     "; which colcon >/dev/null 2>&1 && echo "OK" || echo "FAILED"
echo -n "ros2_ctrl:  "; [ -d ~/ros2_control_ws/install ] && echo "OK" || echo "FAILED"

echo ""
echo "========================================="
echo "Phase 1 complete! Open a new terminal or run:"
echo "  source ~/.bashrc"
echo "========================================="

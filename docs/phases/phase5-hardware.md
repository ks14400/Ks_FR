# Phase 5 — Hardware Commissioning (Real Robot)

Connect the simulation workflow to the real FR16/FR20. The code doesn't change —
only the IP address and the `use_fake_hardware` flag.

---

## Pre-Commissioning Checklist

**All of these must pass in simulation before touching the real robot:**

- [ ] Full pick-place-trigger sequence completes in Gazebo
- [ ] No collision warnings during any planned path
- [ ] Gripper open/close works in simulation
- [ ] PXRD trigger fires correctly
- [ ] Speed limits enforced (FR16 ≤ 1000 mm/s, approach ≤ 50 mm/s)
- [ ] Joint limits cross-checked against Fairino hardware spec

---

## Step 1: Verify Robot Connection

Use the Python SDK (this repo) to confirm basic connectivity:

```bash
python3 -c "from fairino import Robot; r=Robot.RPC('<ROBOT_IP>'); print(r.GetRobotState())"
```

Check firmware version matches v3.8.x:
```bash
python3 -c "from fairino import Robot; r=Robot.RPC('<ROBOT_IP>'); print(r.GetSoftwareVersion())"
```

---

## Step 2: Update IP Configuration

1. Update robot IP in your `pxrd_cell` config (env var or config file)

2. Update IP in the Devonics/Fairino hardware interface files:
   ```
   src/fairino_hardware/include/fairino_hardware/data_types_def.h
   src/fairino_gazebo_config/launch/rt_state_data.py
   src/fairino_hardware/include/fairino_hardware/fairino_hardware_interface.hpp
   ```

3. Set `use_fake_hardware:=false` in the ros2_control xacro file

4. Rebuild:
   ```bash
   colcon build --symlink-install --packages-select fairino_hardware
   source install/setup.bash
   ```

---

## Step 3: First Run — Slow Speed

Create a test YAML with very low speeds:

```yaml
job_name: hardware_test_slow
robot_model: fr16
speed_linear_mms: 10       # very slow
speed_joint_pct: 5          # very slow
# ... same waypoints as simulation
```

```bash
# Terminal 1: MoveIt2 with cell scene (now connected to real robot)
ros2 launch pxrd_cell pxrd_moveit.launch.py

# Terminal 2: Gazebo mirror (visualize what robot is doing)
ros2 launch fairino16_v6_moveit2_config fr16_gazebo_mirror.launch.py

# Terminal 3: Run slow test job
ros2 run pxrd_cell job_runner --ros-args -p job_file:=~/jobs/hardware_test_slow.yaml
```

---

## Safety

- Keep the **e-stop within reach** at all times
- Run Gazebo **mirror mode** to see what the robot will do before it does it
- Test each waypoint individually before running full sequence
- FR16 max TCP speed: 1 m/s. FR20: 2 m/s. **Never exceed.**
- If anything looks wrong: e-stop first, diagnose second

---

## Hardware Reference

| Item | Details |
|---|---|
| Robot arm | FR16 (16 kg, 1034 mm, ±0.03 mm) or FR20 (20 kg, 1854 mm) |
| Controller I/O | 16 DI, 16 DO, 2 AI, 2 AO |
| Head (end-effector) I/O | 2 DI, 2 DO, 1 AI, 1 AO |
| SDK ports | 20003 (XML-RPC), 20004 (state feedback) |
| ros2_control ports | 8080 (commands), 8083 (status ~10 Hz) |
| Default IP | `192.168.58.2` |
| Firmware | v3.8.6 (no EtherCAT) |

---

## SDK Quick Reference (for direct debugging)

When you need to bypass ROS2 and talk to the robot directly:

```python
from fairino import Robot
r = Robot.RPC('<ROBOT_IP>')

r.GetRobotState()                # Connection check
r.GetActualTCPPose()             # Current tool pose [x,y,z,rx,ry,rz]
r.GetActualJointPosDegree()      # Current joints [j1..j6] degrees
r.GetRobotErrorCode()            # Error diagnosis
r.ResetAllError()                # Clear errors
r.RobotEnable(1)                 # Enable robot
r.SetSpeed(10)                   # Set global speed %

# Gripper
r.SetGripperConfig(4, 0, 0, 2)
r.ActGripper(1, 1)
r.MoveGripper(1, 90, 50, 50, 30000, 0, 0, 0, 0, 0)  # Open
r.MoveGripper(1, 10, 50, 50, 30000, 0, 0, 0, 0, 0)  # Close

r.CloseRPC()
```

---

## Firmware Upgrade Protocol

If firmware needs upgrading:
1. Upgrade SDK tag and ROS2 package tag together
2. Rebuild: `colcon build --symlink-install`
3. Retest full sequence in simulation
4. Upgrade via WebApp: System Settings → System Upgrade → Upload Package

---

## Phase 5 Completion Checklist

- [ ] Robot connection verified via SDK
- [ ] Firmware version confirmed as 3.8.x
- [ ] IP configured in all files, `use_fake_hardware:=false`
- [ ] Gazebo mirror tracks real robot joint states
- [ ] Slow-speed test run completes safely
- [ ] Full-speed job sequence completes
- [ ] Gripper confirmed working on hardware
- [ ] PXRD trigger confirmed working
- [ ] Production job YAMLs finalized

# MirrorSLAM

A ROS2 (Jazzy) system that drives a TurtleBot3 through an unknown space,
follows walls using lidar, and uses a UR3e arm to physically probe surfaces.
If the arm's motion fails (indicating a hard/reflective surface like a mirror),
the location is stored permanently and injected into the Nav2 costmap so the
robot avoids it on future passes.

## Architecture

| Package | Role |
|---|---|
| `gap_explorer` | TurtleBot3 brain — wall detection, Nav2 client, state machine |
| `ur3` | UR3e action server — receives `ProbeArm` goals, executes home→poke→home |
| `gap_explorer_interfaces` | Shared `ProbeArm.action` definition |
| `mirror_slam_bringup` | Nav2 params, maps, launch files |

### State machine (gap_explorer)
`COLLECT` → `NAV` → `SETTLE` → `FOLLOW` → `PROBE` → `COLLECT`

The robot collects lidar scans, fits wall segments by PCA, navigates to the
nearest unvisited wall, follows it to the endpoint, then triggers the arm probe.
A failed poke stores the wall as a detected mirror and marks it in the Nav2
costmap permanently.

## Dependencies

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-turtlebot3-gazebo \
    ros-jazzy-ur-robot-driver ros-jazzy-ur-moveit-config \
    ros-jazzy-moveit ros-jazzy-rtabmap-ros
```

Set your TurtleBot3 model:
```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select gap_explorer_interfaces ur3 gap_explorer mirror_slam_bringup
source install/setup.bash
```

---

## Launch — Simulation (fake hardware)

Open each section in a new terminal. Source your workspace in each one:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Terminal 1 — UR3e driver (mock hardware)
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=0.0.0.0 \
    use_mock_hardware:=true \
    launch_rviz:=false
```

### Terminal 2 — MoveIt (provides /move_action for the probe server)
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    use_mock_hardware:=true \
    launch_rviz:=true
```
Wait for `"You can start planning now!"` before continuing.

### Terminal 3 — TurtleBot3 simulation + Nav2 + SLAM
```bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
    world:=$HOME/ros2_ws/src/Mirror_SLAM/mirror_slam_bringup/worlds/simpleroom.sdf.xacro \
    x_pose:=2.0 \
    y_pose:=0.0 \
    z_pose:=0.1 \
    slam:=True \
    headless:=False
```

### Terminal 4 — Arm probe server + gap explorer
```bash
ros2 launch gap_explorer gap_explorer.launch.py
```

> **Note:** If running the gap explorer standalone (outside the launch file),
> pass `use_sim_time` manually:
> ```bash
> ros2 run gap_explorer gap_explorer --ros-args -p use_sim_time:=true
> ```

---

## Launch — Real Hardware

### Terminal 1 — UR3e driver
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    robot_ip:=10.3.13.64 \
    launch_rviz:=false
```
Then start the `external_control` URCap program from the teach pendant.

### Terminal 2 — MoveIt
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    robot_ip:=10.3.13.64 \
    launch_rviz:=true
```

### Terminal 3 — Unitree L1 Lidar
```bash
sudo chmod 666 /dev/ttyUSB0
ros2 launch unitree_lidar_ros2 launch.py
```

### Terminal 4 — SLAM (RTAB-Map)
```bash
ros2 launch rtabmap_examples lidar3d_assemble.launch.py \
    lidar_topic:=/unilidar/cloud \
    imu_topic:=/unilidar/imu \
    frame_id:=base_link
```

### Terminal 5 — Arm probe server + gap explorer
```bash
ros2 launch gap_explorer gap_explorer.launch.py
```

---

## Manually trigger a probe (testing)

```bash
ros2 action send_goal --feedback /probe_arm \
    gap_explorer_interfaces/action/ProbeArm \
    "{extend_distance_m: 0.20, timeout_sec: 8.0}"
```

## Useful commands

```bash
# Check all controllers are active
ros2 control list_controllers

# Check Nav2 is running
ros2 action list | grep navigate

# Watch mirror detections live
ros2 topic echo /detected_mirrors

# Visualize in RViz (gap debug markers + mirror overlays)
ros2 run rviz2 rviz2
```

---

## MoveIt & NavStack assignment: 
f391ca97a8ce2d8ae10699f42f9265be00e98ad4

# MirrorSLAM

A ROS2 system that drives a TurtleBot through an unknown space, follows walls
using lidar, and uses a UR3e arm to physically probe surfaces. If the arm's
motion fails (indicating a mirror or hard surface), the location is injected
into the Nav2 costmap so the robot avoids it on future passes.

## Architecture

| Package | Role |
|---|---|
| `gap_explorer` | TurtleBot brain — wall detection, Nav2 client, 5-state machine |
| `ur3` | UR3e action server — receives `ProbeArm` goals, executes home→poke→home |
| `gap_explorer_interfaces` | Shared `ProbeArm.action` definition |
| `mirror_slam_bringup` | Nav2 params, world files, launch files |
| `scripts/` | Bridge scripts and TurtleBot setup automation |

### State machine (gap_explorer)

`COLLECT` → `NAV` → `SETTLE` → `FOLLOW` → `PROBE` → `COLLECT`

### Network architecture

```
┌─────────────────────────────────┐         ┌──────────────────────────┐
│           LAPTOP (Jazzy)        │         │   TURTLEBOT (Foxy)       │
│                                 │  WiFi   │                          │
│  rosbridge :9090  ◄─────────────┼────────►│  cmd_vel_bridge.py       │
│  gap_explorer                   │         │  lidar_bridge.py         │
│  SLAM / Nav2                    │         │  Unitree L1 lidar        │
│  MoveIt + UR3e (sim or real)    │         │  Kobuki base             │
│  RViz                           │         │                          │
└─────────────────────────────────┘         └──────────────────────────┘

Laptop → /commands/velocity → TurtleBot (drive commands)
TurtleBot → /unilidar/cloud, /unilidar/imu → Laptop (lidar data)
```

### Machines

| Machine | IP | ROS2 |
|---|---|---|
| Laptop | 10.5.15.42 | Jazzy |
| turtle-one | 10.5.12.184 | Foxy |
| UR3e arm | 10.3.4.10 | — |

---

## Build (laptop)

```bash
cd ~/ros2_ws
colcon build --packages-select \
    gap_explorer_interfaces ur3 gap_explorer mirror_slam_bringup
source install/setup.bash
```

---

## New TurtleBot Setup

One-time setup for a fresh TurtleBot. This installs the lidar package,
bridge scripts, udev rules, and disables lid-close suspend.

### From the laptop:

```bash
# Get your laptop IP
ip addr show wlp3s0 | grep "inet "

# Copy the setup script to the TurtleBot
scp ~/ros2_ws/src/Mirror_SLAM/scripts/setup_turtlebot.sh <user>@<TB_IP>:~/

# SSH in and run it
ssh <user>@<TB_IP>
chmod +x ~/setup_turtlebot.sh
./setup_turtlebot.sh <LAPTOP_IP>
```

For turtle-one specifically:

```bash
scp ~/ros2_ws/src/Mirror_SLAM/scripts/setup_turtlebot.sh turtle-one@10.5.12.184:~/
ssh turtle-one@10.5.12.184
chmod +x ~/setup_turtlebot.sh
./setup_turtlebot.sh 10.5.15.42
```

The script handles everything: lidar build, bridge scripts, serial symlinks,
sleep disable. After it finishes, the TurtleBot is ready.

### If the laptop IP changes:

Re-run the setup script with the new IP, or manually edit both files:

```bash
nano ~/ros_bridge/cmd_vel_bridge.py   # change LAPTOP_IP
nano ~/ros_bridge/lidar_bridge.py     # change LAPTOP_IP
```

---

# Launch Sequence

## Step 1 — Laptop: start rosbridge

```bash
# Laptop Terminal 1
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Leave running. All bridges share this single WebSocket on port 9090.

---

## Step 2 — TurtleBot: start all robot-side processes (4 SSH sessions)

```bash
# SSH Terminal 1 — Unitree lidar
ssh turtle-one@10.5.12.184
source /opt/ros/foxy/setup.bash
source ~/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

```bash
# SSH Terminal 2 — Lidar bridge (TB → laptop)
ssh turtle-one@10.5.12.184
source /opt/ros/foxy/setup.bash
source ~/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
python3 ~/ros_bridge/lidar_bridge.py
```

```bash
# SSH Terminal 3 — Cmd vel bridge (laptop → TB)
ssh turtle-one@10.5.12.184
source /opt/ros/foxy/setup.bash
python3 ~/ros_bridge/cmd_vel_bridge.py
```

```bash
# SSH Terminal 4 — Kobuki base
ssh turtle-one@10.5.12.184
source /opt/ros/foxy/setup.bash
source ~/kobuki_ws_2/install/setup.bash
ros2 launch kobuki_node kobuki_node-launch.py device_port:=/dev/kobuki
```

---

## Step 3 — Laptop: verify bridge

```bash
# Laptop Terminal 2
ros2 topic list | grep -E "unilidar|commands"
```

Expected:

```
/commands/velocity
/unilidar/cloud
/unilidar/imu
```

Quick drive test:

```bash
ros2 topic pub --once /commands/velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.05}, angular: {z: 0.0}}"
```

The Kobuki should twitch forward.

---

## Step 4 — Laptop: start UR3e arm

Choose **one** of the two options below.

### Terminal 3.1 — Simulated arm (mock hardware)

```bash
# Laptop Terminal 3
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e use_mock_hardware:=true robot_ip:=0.0.0.0 \
    launch_rviz:=false
```

### Terminal 3.2 — Real arm

```bash
# Laptop Terminal 3
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e robot_ip:=10.3.4.10 launch_rviz:=false
```

Then start the External Control URCap from the teach pendant.

---

## Step 5 — Laptop: start MoveIt

Same for both sim and real:

```bash
# Laptop Terminal 4
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e launch_rviz:=true \
    warehouse_sqlite_path:=$HOME/.ros/warehouse_ros.db
```

Wait for `"You can start planning now!"`.

---

## Step 6 — Laptop: start probe server

```bash
# Laptop Terminal 5
cd ~/ros2_ws && source install/setup.bash
ros2 run ur3 probe_server
```

Wait for `"probe_arm action server ready."`.

Optional — test the arm manually:

```bash
ros2 action send_goal --feedback /probe_arm \
    gap_explorer_interfaces/action/ProbeArm "{}"
```

Expected feedback: `homing` → `poking` → `returning` → `done`

---

## Step 7 — Laptop: start SLAM / Nav2

```bash
# Laptop Terminal 6
cd ~/ros2_ws && source install/setup.bash
ros2 launch mirror_slam_bringup mirror_slam_full.launch.py
```

Wait for Nav2 to come up. Verify:

```bash
ros2 lifecycle get /bt_navigator       # should be: active [3]
ros2 lifecycle get /controller_server   # should be: active [3]
```

Test with a manual Nav2 goal in RViz before starting autonomy.

---

## Step 8 — Laptop: start gap_explorer

```bash
# Laptop Terminal 7
cd ~/ros2_ws && source install/setup.bash
ros2 run gap_explorer gap_explorer --ros-args -p scan_topic:=/unilidar/scan
```

Expected log sequence:

```
COLLECT → NAV → SETTLE → FOLLOW → PROBE → COLLECT
```

The robot will autonomously explore, follow walls, and probe surfaces.

---

## Velocity topic relay

The Kobuki listens on `/commands/velocity`. If gap_explorer or Nav2 publishes
on `/cmd_vel`, relay it:

```bash
ros2 run topic_tools relay /cmd_vel /commands/velocity
```

If your launch file already includes this relay, do not start a duplicate.

---

## Shutdown order

Stop in reverse:

1. gap_explorer
2. SLAM / Nav2
3. Probe server
4. MoveIt
5. UR3e driver
6. Kobuki base (SSH)
7. Command bridge (SSH)
8. Lidar bridge (SSH)
9. Unitree lidar (SSH)
10. rosbridge (laptop)

Use `Ctrl-C` in each terminal.

Nuclear reset:

```bash
# Laptop
pkill -f ros2; pkill -f rosbridge; pkill -f gazebo; pkill -f gz
ros2 daemon stop && ros2 daemon start
```

---

## Useful commands

```bash
# View TF tree
ros2 run tf2_tools view_frames

# Check controllers
ros2 control list_controllers

# Monitor lidar rate
ros2 topic hz /unilidar/cloud

# Check Nav2 status
ros2 action list | grep navigate

# Watch mirror detections
ros2 topic echo /detected_mirrors

# Check probe action server
ros2 action list | grep probe_arm

# Debug velocity flow
ros2 topic echo /cmd_vel
ros2 topic echo /commands/velocity
```

---

## Troubleshooting

**Rosbridge "Address already in use":**
`pkill -f rosbridge` then relaunch.

**No lidar topics on laptop:**
Check: (1) rosbridge running on laptop, (2) lidar node running on TB,
(3) lidar_bridge.py running on TB, (4) correct laptop IP in bridge script.

**Kobuki doesn't move:**
Check `ros2 topic echo /commands/velocity` on laptop. Verify cmd_vel_bridge.py
and Kobuki node are both running on TB.

**Serial permission denied on TB:**
`sudo chmod 666 /dev/ttyUSB*` or verify udev rules: `ls -l /dev/lidar /dev/kobuki`

**Serial ports swapped after reboot:**
Use `/dev/lidar` and `/dev/kobuki` symlinks instead of `/dev/ttyUSBx`.

**Arm driver timeout (real hardware):**
The 1-second RTDE timeout is hardcoded. Requires low-latency connection.
Direct ethernet preferred. WiFi works intermittently.

**Nav2 goal succeeds but FOLLOW doesn't move:**
Check `ros2 topic echo /cmd_vel` during FOLLOW. If empty, gap_explorer may
be aborting. Check logs for `FOLLOW step:` debug messages.

**TurtleBot sleeps with lid closed:**
Re-run `setup_turtlebot.sh` or manually edit `/etc/systemd/logind.conf`
and `sudo systemctl mask sleep.target suspend.target`.

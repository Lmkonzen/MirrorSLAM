# MirrorSLAM Turtle-One Startup Guide

This README is the simplified launch sequence for running the TurtleBot setup on `turtle-one` only. Ignore Baymax for now.

Goal:

1. SSH into `turtle-one` and start the robot-side processes.
2. Start rosbridge on the laptop so the TurtleBot and laptop can exchange topics over WebSocket.
3. Verify the laptop is receiving lidar and publishing velocity commands.
4. Bring up the SLAM/Nav2 stack on the laptop.
5. Bring up `gap_explorer`.

---

## Machines

| Machine | Role |
|---|---|
| Laptop | Runs rosbridge, SLAM/Nav2, RViz, and `gap_explorer` |
| `turtle-one` | Runs the Unitree lidar, lidar bridge, command bridge, and Kobuki base |

Known TurtleBot address:

```bash
ssh turtle-one@10.5.12.184
```

---

## Before starting

On the laptop, make sure your ROS 2 workspace is built and sourced:

```bash
cd ~/ros2_ws
colcon build --packages-select gap_explorer mirror_slam_bringup
source install/setup.bash
```

If you changed interfaces or the arm package too, use:

```bash
cd ~/ros2_ws
colcon build --packages-select gap_explorer_interfaces gap_explorer mirror_slam_bringup
source install/setup.bash
```

---

# Startup sequence

Use this order.

---

## Step 1 — Laptop: start rosbridge WebSocket

Open a laptop terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Leave this running.

This opens the WebSocket server on port `9090`. The bridge scripts on `turtle-one` connect to this.

---

## Step 2 — Turtle-one SSH terminal 1: start the Unitree lidar

Open a new terminal on the laptop and SSH into `turtle-one`:

```bash
ssh turtle-one@10.5.12.184
```

Then run:

```bash
source /opt/ros/foxy/setup.bash
source ~/unilidar_ws/src/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

Leave this running.

If the lidar has a serial permission issue, run this on `turtle-one` and try again:

```bash
sudo chmod 666 /dev/ttyUSB0
```

If that does not work, check which USB device is active:

```bash
ls /dev/ttyUSB*
dmesg | tail -20
```

---

## Step 3 — Turtle-one SSH terminal 2: start the lidar bridge to the laptop

Open another terminal and SSH into `turtle-one` again:

```bash
ssh turtle-one@10.5.12.184
```

Then run:

```bash
python3 ~/ros_bridge/lidar_bridge.py
```

Leave this running.

This sends TurtleBot lidar topics to the laptop through rosbridge.

---

## Step 4 — Turtle-one SSH terminal 3: start the command bridge from laptop to TurtleBot

Open a third terminal and SSH into `turtle-one` again:

```bash
ssh turtle-one@10.5.12.184
```

Then run:

```bash
python3 ~/ros_bridge/cmd_vel_bridge.py
```

Leave this running.

This receives velocity commands from the laptop and republishes them on the TurtleBot side.

---

## Step 5 — Turtle-one SSH terminal 4: start the Kobuki base

Open a fourth terminal and SSH into `turtle-one` again:

```bash
ssh turtle-one@10.5.12.184
```

Then run:

```bash
source /opt/ros/foxy/setup.bash
source ~/kobuki_ws_2/install/setup.bash
ros2 launch kobuki_node kobuki_node-launch.py
```

Leave this running.

The Kobuki base should be listening for velocity commands on:

```bash
/commands/velocity
```

---

## Step 6 — Laptop: verify bridge topics

In a new laptop terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list | grep -E "unilidar|commands"
```

You should see at least:

```text
/commands/velocity
/unilidar/cloud
/unilidar/imu
```

Check that lidar data is arriving:

```bash
ros2 topic hz /unilidar/cloud
ros2 topic hz /unilidar/imu
```

Optional RViz check:

```bash
rviz2
```

In RViz:

- Fixed Frame: `unilidar_lidar`
- Add display: `PointCloud2`
- Topic: `/unilidar/cloud`

---

## Step 7 — Laptop: test command bridge before launching autonomy

From the laptop, publish a tiny forward command:

```bash
ros2 topic pub --once /commands/velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.05, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

The Kobuki should twitch forward briefly.

If it does, the laptop → TurtleBot command path works.

To stop the robot manually:

```bash
ros2 topic pub --once /commands/velocity geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Step 8 — Laptop: bring up SLAM/Nav2

In a new laptop terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch mirror_slam_bringup mirror_slam_full.launch.py
```

This brings up the SLAM/Nav2 stack.

Wait for RViz/Nav2 to come up. In RViz, use a Nav2 Goal to verify that the robot can navigate.

Useful checks:

```bash
ros2 topic hz /map
ros2 topic hz /unilidar/scan
ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
```

Expected lifecycle state for Nav2 nodes:

```text
active [3]
```

---

## Step 9 — Laptop: bring up gap_explorer

Once SLAM/Nav2 is running and a manual Nav2 goal works, start `gap_explorer`:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run gap_explorer gap_explorer --ros-args -p scan_topic:=/unilidar/scan
```

The expected state sequence is:

```text
COLLECT → NAV → SETTLE → FOLLOW → PROBE → COLLECT
```

You should see logs like:

```text
Selected a new wall. Following wall on left. Sending Nav2 goal
Nav goal accepted: True
Nav result callback fired: status=4
```

`status=4` means the Nav2 goal succeeded.

---

# Optional: velocity topic notes

The Kobuki side expects commands on:

```bash
/commands/velocity
```

If the laptop publishes Nav2 commands on `/cmd_vel`, relay them to `/commands/velocity`:

```bash
ros2 run topic_tools relay /cmd_vel /commands/velocity
```

If your launch file already starts this relay, do not start a duplicate relay unless you intentionally want another publisher.

Check publishers and subscribers with:

```bash
ros2 topic info /cmd_vel -v
ros2 topic info /commands/velocity -v
```

---

# Shutdown order

Stop things in reverse order:

1. `gap_explorer`
2. `mirror_slam_bringup`
3. Kobuki base
4. Command bridge
5. Lidar bridge
6. Unitree lidar
7. rosbridge WebSocket

Use `Ctrl-C` in each terminal.

If things get messy, reset ROS processes:

```bash
pkill -f ros2
pkill -f rosbridge
ros2 daemon stop
ros2 daemon start
```

---

# Troubleshooting

## Rosbridge address already in use

On the laptop:

```bash
pkill -f rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

## No `/unilidar/cloud` on the laptop

Check this order:

1. Laptop rosbridge is running.
2. Unitree lidar is running on `turtle-one`.
3. `lidar_bridge.py` is running on `turtle-one`.
4. The bridge script has the correct laptop IP address.

On the laptop:

```bash
ip addr show
```

On `turtle-one`, check whether the laptop websocket is reachable:

```bash
nc -zv <LAPTOP_IP> 9090
```

## Kobuki does not move

On laptop:

```bash
ros2 topic info /commands/velocity -v
ros2 topic echo /commands/velocity
```

You should see commands being published.

On `turtle-one`, make sure the Kobuki node and `cmd_vel_bridge.py` are both running.

## Lidar permission denied

On `turtle-one`:

```bash
sudo chmod 666 /dev/ttyUSB0
```

If the device is not `/dev/ttyUSB0`:

```bash
ls /dev/ttyUSB*
dmesg | tail -20
```

## Nav2 goal works but follow does not move

Check whether `gap_explorer` is entering `FOLLOW` and whether `/cmd_vel` or `/commands/velocity` is receiving nonzero commands:

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /commands/velocity
```

During normal follow, you should eventually see:

```yaml
linear:
  x: 0.19
angular:
  z: <some turn value>
```

If `linear.x` stays `0.0`, `gap_explorer` may be stopping, settling, or aborting follow before it publishes forward motion.

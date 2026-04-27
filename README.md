Commit Hash ID: bf0c28d05498f254ff0d713c05455c9d60d51ce5
# MirrorSLAM

A ROS2 (Jazzy) system that drives a Kobuki base through an unknown space,
follows walls using a Unitree L1 3D lidar, and uses a UR3e arm to physically
probe surfaces. If the arm's motion fails (indicating a hard/reflective surface
like a mirror), the location is stored permanently and injected into the Nav2
costmap so the robot avoids it on future passes.

## Architecture

| Package | Role |
|---|---|
| `gap_explorer` | Robot brain — wall detection, Nav2 client, 5-state machine |
| `ur3` | UR3e action server — receives `ProbeArm` goals, executes home→poke→home |
| `gap_explorer_interfaces` | Shared `ProbeArm.action` definition |
| `mirror_slam_bringup` | Launch files, Nav2 params, world files, custom RTAB-Map launch |

### Pipeline

```
Unitree L1 (3D PointCloud2 + IMU)
    ↓
RTAB-Map (lidar3d_assemble, custom launch)
    ↓ /odom + odom→base_link TF
    ↓ /map  + map→odom  TF
    ↓ /assembled_cloud (denser merged cloud)
    ↓
pointcloud_to_laserscan (slice from assembled cloud)
    ↓ /unilidar/scan
    ↓
Nav2 (planner, controller, costmaps)
    ↓ /cmd_vel
topic_tools relay
    ↓ /commands/velocity
Kobuki (motor base, listens over LAN)

UR3e (UR driver + MoveIt + arm probe action server)
    ↓ probes walls when gap_explorer transitions to PROBE state
```

### State machine (gap_explorer)

`COLLECT` → `NAV` → `SETTLE` → `FOLLOW` → `PROBE` → `COLLECT`

The robot collects lidar scans, fits wall segments by PCA, navigates to the
nearest unvisited wall, follows it to the endpoint, then triggers the arm probe.
A failed poke stores the wall as a detected mirror and marks it permanently in
the Nav2 costmap so the robot routes around it on future passes.

### TF tree note

The UR driver's URDF roots the arm chain at `base_link_inertia`, not
`base_link`, so the arm and mobile base coexist in the same TF tree without
collision. The full chain is `map → odom → base_link_stabilized → base_link →
{unilidar_lidar, base_link_inertia → ... → tool0}`.

---

## Dependencies

```bash
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-rtabmap-examples \
    ros-jazzy-rtabmap-launch ros-jazzy-pointcloud-to-laserscan \
    ros-jazzy-topic-tools ros-jazzy-kobuki-node \
    ros-jazzy-ur-robot-driver ros-jazzy-ur-moveit-config \
    ros-jazzy-moveit ros-jazzy-turtlebot3-gazebo
```

Optional permanent fix for lidar USB permissions:

```bash
sudo usermod -aG dialout $USER
# log out and back in
```

If you skip the dialout group, run `sudo chmod 666 /dev/ttyUSB0` before
each launch.

### CycloneDDS participant limit (Jazzy)

This stack runs many nodes simultaneously. Cyclone DDS on Jazzy caps the
participant index, which causes new nodes to fail with
`Failed to find a free participant index for domain 1`. Fix once by adding
to `~/.bashrc`:

```bash
export CYCLONEDDS_URI='<CycloneDDS><Domain Id="any"><Discovery><ParticipantIndex>none</ParticipantIndex></Discovery></Domain></CycloneDDS>'
```

Then `source ~/.bashrc`. Every new terminal inherits this.

---

## Build

```bash
cd ~/ros2_ws
colcon build --packages-select \
    gap_explorer_interfaces ur3 gap_explorer mirror_slam_bringup
source install/setup.bash
```

---

## Full system bringup

The full system runs as three independent stages, in this order.

### Stage 1 — UR3e arm (sim mode)

Bring up the arm first. The arm subsystem is independent of the navigation
stack and easier to validate on its own.

#### Terminal 1 — UR3e driver

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e \
    use_mock_hardware:=true \
    robot_ip:=0.0.0.0 \
    launch_rviz:=false
```

#### Terminal 2 — MoveIt + RViz

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e \
    use_mock_hardware:=true \
    launch_rviz:=true \
    warehouse_sqlite_path:=$HOME/.ros/warehouse_ros.db
```

Wait for `"You can start planning now!"`.

#### Terminal 3 — Arm probe action server

```bash
ros2 run ur3 probe_server
```

Wait for `"probe_arm action server ready."`.

#### Optional — Test goal

To confirm the arm responds before continuing:

```bash
ros2 action send_goal --feedback /probe_arm \
    gap_explorer_interfaces/action/ProbeArm "{}"
```

Feedback states: `homing` → `poking` → `returning` → `done`. If you see all
four, the arm subsystem is good.

### Stage 2 — Robot bringup (lidar + RTAB-Map + Nav2)

Brings up the entire navigation stack: Unitree lidar, RTAB-Map SLAM,
pointcloud_to_laserscan, Nav2, RViz, and the cmd_vel relay.

The Kobuki itself must already be running on its onboard computer over
SSH (see Kobuki section below).

```bash
sudo chmod 666 /dev/ttyUSB0    # only if you skipped the dialout group fix
cd ~/ros2_ws && source install/setup.bash
ros2 launch mirror_slam_bringup mirror_slam_full.launch.py
```

Stages launch in sequence with TimerAction delays:

| T+   | Stage                                                       |
|------|-------------------------------------------------------------|
| 0s   | Unitree lidar driver, cmd_vel → Kobuki relay                |
| 5s   | RTAB-Map (custom launch with `Grid/MaxObstacleHeight=1.5`)  |
| 10s  | pointcloud_to_laserscan (slices `/assembled_cloud`)         |
| 15s  | Nav2 (no SLAM; RTAB-Map provides `/map` and TF)             |
| 20s  | RViz with Nav2 config                                       |

After ~20 seconds RViz opens with Fixed Frame = `map`. Use the **Nav2 Goal**
button in the toolbar to manually send a navigation goal and verify the
stack drives the robot.

#### Kobuki (over SSH on the robot computer)

In a tmux session on the Kobuki's onboard machine:

```bash
ros2 launch kobuki_node kobuki_node-launch.py
```

That's it on the robot side. The cmd_vel relay runs on the laptop.

### Stage 3 — gap_explorer wall-follower

Once Stage 1 (arm) and Stage 2 (nav stack) are both up and verified, launch
the brain:

```bash
ros2 run gap_explorer gap_explorer \
    --ros-args -p scan_topic:=/unilidar/scan
```

The robot will start wall-following autonomously. When it reaches the end
of a detected wall, it transitions to `PROBE` state and sends a
`ProbeArm` goal to the arm action server (Stage 1). A successful probe
moves on; a failed probe marks the wall as a mirror and stores it in
the Nav2 costmap so the robot avoids it on future passes.

---

## Stage 2 piecewise (debugging)

If you want to run each Stage 2 component in its own terminal for
debugging — easier to iterate on a single component without restarting the
world. Sequence matters; each stage depends on the previous.

### Terminal 1 — Unitree lidar driver

```bash
sudo chmod 666 /dev/ttyUSB0
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

### Terminal 2 — cmd_vel → Kobuki bridge

```bash
source /opt/ros/jazzy/setup.bash
ros2 run topic_tools relay /cmd_vel /commands/velocity
```

### Terminal 3 — RTAB-Map (custom launch)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch mirror_slam_bringup lidar3d_assemble_custom.launch.py \
    lidar_topic:=/unilidar/cloud \
    imu_topic:=/unilidar/imu \
    frame_id:=base_link
```

This custom launch is a copy of `rtabmap_examples/lidar3d_assemble.launch.py`
with `Grid/MaxObstacleHeight=1.5` and `Grid/MaxGroundHeight=0.1` injected into
the rtabmap node's parameters. The upstream version offers no way to set these
through launch arguments. The custom launch also publishes the
`base_link → unilidar_lidar` static TF internally.

### Terminal 4 — pointcloud_to_laserscan

```bash
source /opt/ros/jazzy/setup.bash
ros2 run pointcloud_to_laserscan pointcloud_to_laserscan_node \
    --ros-args \
    -r cloud_in:=/assembled_cloud \
    -r scan:=/unilidar/scan \
    -p target_frame:=base_link \
    -p min_height:=-0.1 \
    -p max_height:=0.5 \
    -p range_min:=0.01 \
    -p range_max:=20.0
```

### Terminal 5 — Nav2 (no SLAM)

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav2_bringup navigation_launch.py \
    params_file:=$HOME/ros2_ws/src/Mirror_SLAM/mirror_slam_bringup/params/nav2_params.yaml \
    autostart:=True
```

### Terminal 6 — RViz

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch nav2_bringup rviz_launch.py
```

---

## Sanity checks

After the stack is up:

```bash
ros2 topic info /odom --verbose                  # publisher: rtabmap
ros2 topic hz /assembled_cloud                   # ~1 Hz
ros2 topic hz /unilidar/scan                     # matches assembled rate
ros2 topic hz /map                               # ~1 Hz on update
ros2 run tf2_ros tf2_echo map base_link          # full TF chain works
ros2 run tf2_ros tf2_echo base_link tool0        # arm chain reachable
ros2 lifecycle get /bt_navigator                 # active [3]
ros2 lifecycle get /controller_server            # active [3]
ros2 param get /rtabmap Grid/MaxObstacleHeight   # 1.5
ros2 action list | grep probe_arm                # /probe_arm
```

---

# Standalone test modes

These are run **independently** of each other and the main pipeline. Kill any
running ROS2 processes first:

```bash
pkill -f ros2; pkill -f gazebo; pkill -f gz
ros2 daemon stop && ros2 daemon start
```

## Sim Mode B — TurtleBot3 simulation (legacy)

Tests gap_explorer wall-following in Gazebo. Does not include the arm or the
RTAB-Map/Unitree pipeline. Predates the Kobuki + Unitree hardware switch.

### Terminal 1 — TB3 sim + Nav2 + SLAM

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
    world:=$HOME/ros2_ws/src/Mirror_SLAM/mirror_slam_bringup/worlds/simpleroom.sdf.xacro \
    x_pose:=2.0 y_pose:=0.0 z_pose:=0.1 \
    slam:=True headless:=True
```

Wait for `"Managed nodes are active"`.

### Terminal 2 — Gap explorer

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run gap_explorer gap_explorer --ros-args -p use_sim_time:=true
```

---

## Real arm hardware

Run the UR3e on actual hardware. Same as Stage 1 but without
`use_mock_hardware:=true`.

### Terminal 1 — UR3e driver

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e robot_ip:=10.3.4.10 launch_rviz:=false
```

Start the `external_control` URCap program from the teach pendant.

### Terminal 2 — MoveIt + RViz

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e launch_rviz:=true
```

Wait for `"You can start planning now!"`.

### Terminal 3 — Arm probe server

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run ur3 probe_server
```

### Trigger a probe

```bash
ros2 action send_goal --feedback /probe_arm \
    gap_explorer_interfaces/action/ProbeArm "{}"
```

---

## Useful commands

```bash
# Kill everything and reset before a fresh launch
pkill -f ros2; pkill -f gazebo; pkill -f gz
ros2 daemon stop && ros2 daemon start

# Wipe RTAB-Map's database (forces fresh map on next launch)
rm ~/.ros/rtabmap.db

# View the full TF tree
cd /tmp && ros2 run tf2_tools view_frames && xdg-open /tmp/frames.pdf

# Check Nav2 lifecycle states
ros2 lifecycle get /bt_navigator
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server

# Confirm Nav2 is accepting goals
ros2 action list | grep navigate

# Confirm arm probe server is up
ros2 action list | grep probe_arm

# Watch mirror detections live
ros2 topic echo /detected_mirrors

# Check controllers are active (UR3e)
ros2 control list_controllers

# Inspect the current map's RTAB-Map params
rtabmap-info ~/.ros/rtabmap.db | grep -i grid

# Confirm CycloneDDS env var is set in the current shell
echo $CYCLONEDDS_URI
```

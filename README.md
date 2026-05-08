# MirrorSLAM

Autonomous mirror detection through physical probing. A Kobuki-based mobile robot follows walls with a 3D lidar, then a UR3e robot arm physically probes apparent wall surfaces. Surfaces the arm makes contact with are real obstacles — mirrors, windows, or walls — and get injected into the Nav2 costmap as permanent obstacles. Surfaces the arm passes through without contact are confirmed open space: the lidar's apparent wall there was a phantom (typically a mirror reflecting a more distant surface) and the robot is free to navigate through.

## Project Requirements:

1. Instructions: See Below
2. ur3_movement node was written by hand, but claude was used for troubleshooting. Many packages had to be abridged to work in this project. Unilidar was not compaitble with Ubuntu 24.04... had to be modified. RTAB launcher did not handle params correctly... had to be modified. TB3 did not want to play nice with my gap_explorer, but we figured that out. Websocket was used to make system entirely wireless.
3. Links/videos all embedded below.


## How it works

The robot operates in a five-state loop:

`COLLECT` → `NAV` → `SETTLE` → `FOLLOW` → `PROBE` → `COLLECT`

1. **COLLECT** — accumulate lidar scans and fit candidate wall segments
2. **NAV** — drive to the standoff start point of the chosen wall via Nav2
3. **SETTLE** — pause for SLAM and odometry to stabilize
4. **FOLLOW** — pure-pursuit wall following along the locked wall line
5. **PROBE** — extend the UR3e arm into the wall surface and measure contact

The probe outcome is the discriminator. The arm extends toward the lidar-perceived wall position. If the gravity-compensated TCP wrench exceeds threshold, the arm has made contact — a physical surface (mirror, window, or real wall) is at that location. The position is published on `/detected_mirrors` and injected into the Nav2 costmap, and the robot avoids it on every subsequent pass. If the arm completes its full extension without contact, the lidar return was a phantom: the apparent wall is actually open space and is safe to navigate through.

Contact detection runs in software using the UR controller's force/torque estimate, with thresholds tuned below the protective-stop limit so the arm reacts cleanly without latching the controller.

## Demos

### Arm contact detection

https://github.com/user-attachments/assets/bdcbc0aa-f6e7-40ab-a255-3e66a13fc9f9

### Full system trial run

https://github.com/user-attachments/assets/9a4f64f2-dd54-4281-bfcc-a7e6f507c09e

## Hardware

| Component | Details |
|---|---|
| Mobile base | Kobuki (TurtleBot 2 chassis) |
| 3D lidar | Unitree L1 |
| Robot arm | Universal Robots UR3e |
| Laptop | ROS 2 Jazzy |

The laptop runs SLAM, Nav2, MoveIt, and the autonomy stack. The TurtleBot runs the lidar driver and Kobuki base driver, bridging topics over WiFi via rosbridge.

## Packages

| Package | Role |
|---|---|
| `gap_explorer` | Wall detection, Nav2 client, five-state autonomy machine |
| `ur3` | UR3e action server with active contact monitoring |
| `gap_explorer_interfaces` | `ProbeArm.action` definition |
| `mirror_slam_bringup` | Launch files, Nav2 params, RTAB-Map config |

---
## Project Components

Mapping of required components to where they live in this repo. All paths
relative to repo root.

### MoveIt (Written by hand, advice given by Claude)
The arm probe is driven entirely through MoveIt's MoveGroup action.
- [`ur3/ur3/ur3_movement.py`](ur3/ur3/ur3_movement.py) — `_move()` builds a
  `MotionPlanRequest` with joint constraints and executes it via the
  `MoveGroup` action client. Goal cancellation on contact also flows
  through this client.

### Nav2
- [`gap_explorer/gap_explorer/gap_explorer_node.py`](gap_explorer/gap_explorer/gap_explorer_node.py)
  — `send_nav_goal()` is the NavigateToPose action client; `costmap_cb()`
  ingests the local costmap for path-clearance checks.
- [`mirror_slam_bringup/params/nav2_params.yaml`](mirror_slam_bringup/params/nav2_params.yaml)
  — full Nav2 stack config: controller_server (DWB), planner_server (NavFn),
  collision_monitor, velocity_smoother, both costmaps with the custom
  `mirror_layer`.
- [`mirror_slam_bringup/launch/mirror_slam_full.launch.py`](mirror_slam_bringup/launch/mirror_slam_full.launch.py)
  — bringup orchestration.

### Perception
- **Wall extraction** (lidar → 2D line segments): `scan_points_robot()`,
  `contiguous_segments()`, `fit_segment()` (PCA line fit with RMSE gate),
  and `select_best_wall()` in
  [`gap_explorer_node.py`](gap_explorer/gap_explorer/gap_explorer_node.py).
- **3D SLAM**: RTAB-Map's `lidar3d_assemble` pipeline, customized to
  inject Grid/* parameters and the lidar static TF —
  [`mirror_slam_bringup/launch/lidar3d_assemble_custom.launch.py`](mirror_slam_bringup/launch/lidar3d_assemble_custom.launch.py).
- **3D-to-2D slice for Nav2**: `pointcloud_to_laserscan` node in the
  bringup launch.

### Custom Components
- **Mirror detection through physical probing** — the central novel
  behavior. Probe outcome (contact vs. no-contact) re-classifies lidar
  returns and injects mirrors back into navigation.
  Lives in `finish_probe()` and `remember_mirror()` in
  [`gap_explorer_node.py`](gap_explorer/gap_explorer/gap_explorer_node.py).
- **Mirror costmap layer** — detected mirrors are republished as a
  `PointCloud2` on `/detected_mirrors` (`_publish_mirror_cloud()`) and
  consumed by a dedicated `nav2_costmap_2d::ObstacleLayer` named
  `mirror_layer` in both local and global costmaps.
- **Force/torque-based contact monitoring** — software-side contact
  detection on the UR3e's gravity-compensated wrench, faster and more
  controllable than the hardware protective stop. See
  [`ur3/ur3/ur3_movement.py`](ur3/ur3/ur3_movement.py),
  `_wrench_cb` and the `_move()` poll loop.
- **Custom action interface**:
  [`gap_explorer_interfaces/action/ProbeArm.action`](gap_explorer_interfaces/action/ProbeArm.action).

### Node Written Withthe assistance of Claude
[`gap_explorer/gap_explorer/gap_explorer_node.py`](gap_explorer/gap_explorer/gap_explorer_node.py)
— ~1300 lines. The full five-state autonomy machine, lidar-based wall
fitting and tracking, locked-wall pure-pursuit follower with safety
bail-out, exploration goal selection over the costmap, completed-wall
and mirror memory with canonical signatures, and Nav2/probe action
clients. Nothing in here is from a tutorial or template.

## Setup

### 1. Clone

```bash
cd ~/ros2_ws/src
git clone https://github.com/Lmkonzen/Mirror_SLAM.git
cd ~/ros2_ws
```

### 2. Build (laptop)

```bash
MAKEFLAGS="-j1" colcon build --symlink-install \
    --parallel-workers 1 --executor sequential \
    --packages-select gap_explorer_interfaces ur3 gap_explorer mirror_slam_bringup
source install/setup.bash
```

Single-threaded build is required to avoid OOM on the laptop.

### 3. Configure the TurtleBot (one-time)

From the laptop:

```bash
scp ~/ros2_ws/src/Mirror_SLAM/scripts/setup_turtlebot.sh turtle-one@<TB_IP>:~/
ssh turtle-one@<TB_IP>
chmod +x ~/setup_turtlebot.sh
./setup_turtlebot.sh <LAPTOP_IP>
```

The script installs the rosbridge client scripts. Run once per TurtleBot.

---

## Launch

Eight terminals: one for rosbridge, four SSH'd into the TurtleBot, three on the laptop for the arm and autonomy stack.

### Laptop terminal 1 — rosbridge

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### TurtleBot SSH terminals

```bash
# SSH 1 — lidar driver
ssh turtle-one@<TB_IP>
source /opt/ros/foxy/setup.bash
source ~/unilidar_ws/src/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
ros2 launch unitree_lidar_ros2 launch.py
```

```bash
# SSH 2 — lidar bridge to laptop
ssh turtle-one@<TB_IP>
source /opt/ros/foxy/setup.bash
python3 ~/ros_bridge/lidar_bridge.py
```

```bash
# SSH 3 — cmd_vel bridge from laptop
ssh turtle-one@<TB_IP>
source /opt/ros/foxy/setup.bash
python3 ~/ros_bridge/cmd_vel_bridge.py
```

```bash
# SSH 4 — Kobuki base driver
ssh turtle-one@<TB_IP>
source /opt/ros/foxy/setup.bash
source ~/kobuki_ws_2/install/setup.bash
ros2 launch kobuki_node kobuki_node-launch.py \
    device_port:=/dev/kobuki publish_tf:=false
```

### Laptop terminal 2 — UR3e driver

Real arm:

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e robot_ip:=<ARM_IP> launch_rviz:=false
```

Then start the External Control URCap from the UR teach pendant.

Simulated arm (no hardware):

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur3e use_mock_hardware:=true robot_ip:=0.0.0.0 launch_rviz:=false
```

### Laptop terminal 3 — MoveIt

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur3e launch_rviz:=true
```

Wait for `You can start planning now!` in the log.

### Laptop terminal 4 — probe server

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run ur3 probe_server
```

Wait for `probe_arm action server ready.`.

### Laptop terminal 5 — SLAM, Nav2, RViz

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 launch mirror_slam_bringup mirror_slam_full.launch.py
```

Brings up RTAB-Map, the pointcloud-to-laserscan slicer, Nav2, the velocity smoother and collision monitor, the cmd_vel-to-Kobuki relay, and RViz. Wait for Nav2 to reach `active` before continuing.

### Laptop terminal 6 — autonomy

```bash
cd ~/ros2_ws && source install/setup.bash
ros2 run gap_explorer gap_explorer --ros-args -p scan_topic:=/unilidar/scan
```

The robot begins exploring. State transitions appear in the log:

```
COLLECT → NAV → SETTLE → FOLLOW → PROBE → COLLECT
```

Mirror detections accumulate on `/detected_mirrors` and persist in the Nav2 costmap for the rest of the session.

## References

- [UR ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver) — driver and `ur_moveit_config` package used here.
- [RTAB-Map](https://github.com/introlab/rtabmap_ros) — SLAM backbone (`lidar3d_assemble` example).
- [Nav2](https://docs.nav2.org/) — navigation stack.
- [Unitree L1 SDK](https://github.com/unitreerobotics/unilidar_sdk) — lidar driver source.

To start and connect to the UR3e, follow the External Control URCap
setup in the UR ROS2 Driver docs.

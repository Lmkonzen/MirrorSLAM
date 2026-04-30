#!/bin/bash
# setup_turtlebot.sh — Run on the TurtleBot via SSH to set up everything.
#
# Usage:
#   scp scripts/setup_turtlebot.sh <user>@<TB_IP>:~/
#   ssh <user>@<TB_IP>
#   chmod +x ~/setup_turtlebot.sh
#   ./setup_turtlebot.sh <LAPTOP_IP>
#
# Example:
#   ./setup_turtlebot.sh 10.5.15.42

set -e

LAPTOP_IP="${1:?Usage: ./setup_turtlebot.sh <LAPTOP_IP>}"
echo "=== Setting up TurtleBot for MirrorSLAM ==="
echo "    Laptop IP: $LAPTOP_IP"
echo ""

# ── 1. Prevent sleep on lid close ─────────────────────────────────────
echo "[1/6] Disabling lid-close suspend..."
sudo sed -i 's/^#*HandleLidSwitch=.*/HandleLidSwitch=ignore/' /etc/systemd/logind.conf
sudo sed -i 's/^#*HandleLidSwitchExternalPower=.*/HandleLidSwitchExternalPower=ignore/' /etc/systemd/logind.conf
sudo sed -i 's/^#*HandleLidSwitchDocked=.*/HandleLidSwitchDocked=ignore/' /etc/systemd/logind.conf
sudo sed -i 's/^#*IdleAction=.*/IdleAction=ignore/' /etc/systemd/logind.conf
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target 2>/dev/null || true
sudo systemctl restart systemd-logind
echo "    Done."

# ── 2. Create udev rules for stable serial port names ────────────────
echo "[2/6] Creating udev rules for Kobuki and Unitree lidar..."
sudo bash -c 'cat > /etc/udev/rules.d/99-robot-serial.rules << EOF
# Unitree L1 lidar (Silicon Labs CP2104)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", SYMLINK+="lidar", MODE="0666"
# Kobuki base (FTDI)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", SYMLINK+="kobuki", MODE="0666"
EOF'
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "    Done. /dev/lidar and /dev/kobuki symlinks created."

# ── 3. Install roslibpy ───────────────────────────────────────────────
echo "[3/6] Installing roslibpy..."
if python3 -c "import roslibpy" 2>/dev/null; then
    echo "    roslibpy already installed."
else
    pip3 install --user roslibpy
    echo "    Installed."
fi

# ── 4. Install Unitree L1 lidar package ───────────────────────────────
echo "[4/6] Building Unitree lidar ROS2 package..."
sudo apt-get install -y libpcl-dev > /dev/null 2>&1 || true

if [ ! -d "$HOME/unilidar_sdk" ]; then
    cd ~
    git clone https://github.com/unitreerobotics/unilidar_sdk.git
fi

cd ~/unilidar_sdk/unitree_lidar_ros2
source /opt/ros/foxy/setup.bash
colcon build 2>&1 | tail -3
echo "    Done."

# ── 5. Write bridge scripts ──────────────────────────────────────────
echo "[5/6] Writing bridge scripts to ~/ros_bridge/..."
mkdir -p ~/ros_bridge

cat > ~/ros_bridge/cmd_vel_bridge.py << CMDEOF
#!/usr/bin/env python3
"""Pull /commands/velocity from laptop rosbridge and republish locally."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import roslibpy

LAPTOP_IP = '$LAPTOP_IP'
PORT = 9090
TOPIC = '/commands/velocity'

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.local_pub = self.create_publisher(Twist, TOPIC, 10)
        self.client = roslibpy.Ros(host=LAPTOP_IP, port=PORT)
        self.client.run()
        if not self.client.is_connected:
            self.get_logger().error(f'Could not connect to rosbridge at {LAPTOP_IP}:{PORT}')
            raise RuntimeError('rosbridge connection failed')
        self.remote_sub = roslibpy.Topic(self.client, TOPIC, 'geometry_msgs/Twist')
        self.remote_sub.subscribe(self.on_remote_msg)
        self.get_logger().info(f'Bridging {LAPTOP_IP}:{PORT}{TOPIC} -> local {TOPIC}')

    def on_remote_msg(self, msg):
        out = Twist()
        out.linear.x  = float(msg['linear']['x'])
        out.linear.y  = float(msg['linear']['y'])
        out.linear.z  = float(msg['linear']['z'])
        out.angular.x = float(msg['angular']['x'])
        out.angular.y = float(msg['angular']['y'])
        out.angular.z = float(msg['angular']['z'])
        self.local_pub.publish(out)

def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.remote_sub.unsubscribe()
            node.client.terminate()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
CMDEOF

cat > ~/ros_bridge/lidar_bridge.py << LIDEOF
#!/usr/bin/env python3
"""Push /unilidar/cloud and /unilidar/imu to laptop via rosbridge."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import roslibpy
import base64

LAPTOP_IP = '$LAPTOP_IP'
PORT = 9090

class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')
        self.client = roslibpy.Ros(host=LAPTOP_IP, port=PORT)
        self.client.run()
        if not self.client.is_connected:
            self.get_logger().error(f'Could not connect to rosbridge at {LAPTOP_IP}:{PORT}')
            raise RuntimeError('rosbridge connection failed')
        self.remote_cloud = roslibpy.Topic(self.client, '/unilidar/cloud', 'sensor_msgs/PointCloud2')
        self.remote_imu = roslibpy.Topic(self.client, '/unilidar/imu', 'sensor_msgs/Imu')
        self.create_subscription(PointCloud2, '/unilidar/cloud', self.cloud_cb, 10)
        self.create_subscription(Imu, '/unilidar/imu', self.imu_cb, 10)
        self.get_logger().info(f'Bridging lidar topics to {LAPTOP_IP}:{PORT}')

    def cloud_cb(self, msg):
        d = {
            'header': {'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}, 'frame_id': msg.header.frame_id},
            'height': msg.height, 'width': msg.width,
            'fields': [{'name': f.name, 'offset': f.offset, 'datatype': f.datatype, 'count': f.count} for f in msg.fields],
            'is_bigendian': msg.is_bigendian, 'point_step': msg.point_step, 'row_step': msg.row_step,
            'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
            'is_dense': msg.is_dense
        }
        self.remote_cloud.publish(roslibpy.Message(d))

    def imu_cb(self, msg):
        d = {
            'header': {'stamp': {'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec}, 'frame_id': msg.header.frame_id},
            'orientation': {'x': msg.orientation.x, 'y': msg.orientation.y, 'z': msg.orientation.z, 'w': msg.orientation.w},
            'orientation_covariance': list(msg.orientation_covariance),
            'angular_velocity': {'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y, 'z': msg.angular_velocity.z},
            'angular_velocity_covariance': list(msg.angular_velocity_covariance),
            'linear_acceleration': {'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y, 'z': msg.linear_acceleration.z},
            'linear_acceleration_covariance': list(msg.linear_acceleration_covariance)
        }
        self.remote_imu.publish(roslibpy.Message(d))

def main():
    rclpy.init()
    node = LidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.remote_cloud.unadvertise()
            node.remote_imu.unadvertise()
            node.client.terminate()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
LIDEOF

chmod +x ~/ros_bridge/cmd_vel_bridge.py ~/ros_bridge/lidar_bridge.py
echo "    Done."

# ── 6. Summary ───────────────────────────────────────────────────────
echo ""
echo "[6/6] Setup complete!"
echo ""
echo "  Serial symlinks:  /dev/lidar  /dev/kobuki"
echo "  Bridge scripts:   ~/ros_bridge/cmd_vel_bridge.py"
echo "                    ~/ros_bridge/lidar_bridge.py"
echo "  Lidar package:    ~/unilidar_sdk/unitree_lidar_ros2/"
echo ""
echo "  To launch the lidar:"
echo "    source /opt/ros/foxy/setup.bash"
echo "    source ~/unilidar_sdk/unitree_lidar_ros2/install/setup.bash"
echo "    ros2 launch unitree_lidar_ros2 launch.py"
echo ""
echo "  To launch bridges (in separate terminals):"
echo "    source /opt/ros/foxy/setup.bash"
echo "    python3 ~/ros_bridge/cmd_vel_bridge.py"
echo "    python3 ~/ros_bridge/lidar_bridge.py"
echo ""
echo "  To launch Kobuki:"
echo "    source /opt/ros/foxy/setup.bash"
echo "    source ~/kobuki_ws_2/install/setup.bash"
echo "    ros2 launch kobuki_node kobuki_node-launch.py device_port:=/dev/kobuki"

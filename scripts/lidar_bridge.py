#!/usr/bin/env python3
"""
lidar_bridge.py — Runs on TurtleBot (Foxy).

Subscribes to /unilidar/cloud and /unilidar/imu locally on the TurtleBot,
then publishes them to the laptop's rosbridge WebSocket so the laptop
can run SLAM/Nav2 against real lidar data.

Usage:
    source /opt/ros/foxy/setup.bash
    source ~/unilidar_ws/src/unilidar_sdk/unitree_lidar_ros2/install/setup.bash
    python3 ~/ros_bridge/lidar_bridge.py

Requires: pip3 install --user roslibpy
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Imu
import roslibpy
import base64

# ── Configure these ──────────────────────────────────────────────────
LAPTOP_IP = '10.5.15.42'
PORT = 9090
# ─────────────────────────────────────────────────────────────────────


class LidarBridge(Node):
    def __init__(self):
        super().__init__('lidar_bridge')

        self.client = roslibpy.Ros(host=LAPTOP_IP, port=PORT)
        self.client.run()
        if not self.client.is_connected:
            self.get_logger().error(
                f'Could not connect to rosbridge at {LAPTOP_IP}:{PORT}')
            raise RuntimeError('rosbridge connection failed')

        self.remote_cloud = roslibpy.Topic(
            self.client, '/unilidar/cloud', 'sensor_msgs/PointCloud2')
        self.remote_imu = roslibpy.Topic(
            self.client, '/unilidar/imu', 'sensor_msgs/Imu')

        self.create_subscription(PointCloud2, '/unilidar/cloud', self.cloud_cb, 10)
        self.create_subscription(Imu, '/unilidar/imu', self.imu_cb, 10)

        self.get_logger().info(
            f'Bridging /unilidar/cloud and /unilidar/imu to {LAPTOP_IP}:{PORT}')

    def cloud_cb(self, msg):
        d = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            },
            'height': msg.height,
            'width': msg.width,
            'fields': [
                {
                    'name': f.name,
                    'offset': f.offset,
                    'datatype': f.datatype,
                    'count': f.count,
                }
                for f in msg.fields
            ],
            'is_bigendian': msg.is_bigendian,
            'point_step': msg.point_step,
            'row_step': msg.row_step,
            'data': base64.b64encode(bytes(msg.data)).decode('ascii'),
            'is_dense': msg.is_dense,
        }
        self.remote_cloud.publish(roslibpy.Message(d))

    def imu_cb(self, msg):
        d = {
            'header': {
                'stamp': {
                    'sec': msg.header.stamp.sec,
                    'nanosec': msg.header.stamp.nanosec,
                },
                'frame_id': msg.header.frame_id,
            },
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w,
            },
            'orientation_covariance': list(msg.orientation_covariance),
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'angular_velocity_covariance': list(msg.angular_velocity_covariance),
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
            'linear_acceleration_covariance': list(
                msg.linear_acceleration_covariance
            ),
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

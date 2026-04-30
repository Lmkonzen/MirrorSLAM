#!/usr/bin/env python3
"""
cmd_vel_bridge.py — Runs on TurtleBot (Foxy).

Pulls /commands/velocity from the laptop's rosbridge WebSocket
and republishes it locally so the Kobuki base can receive drive commands.

Usage:
    source /opt/ros/foxy/setup.bash
    python3 ~/ros_bridge/cmd_vel_bridge.py

Requires: pip3 install --user roslibpy
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import roslibpy

# ── Configure these ──────────────────────────────────────────────────
LAPTOP_IP = '10.5.15.42'
PORT = 9090
TOPIC = '/commands/velocity'
# ─────────────────────────────────────────────────────────────────────


class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.local_pub = self.create_publisher(Twist, TOPIC, 10)

        self.client = roslibpy.Ros(host=LAPTOP_IP, port=PORT)
        self.client.run()
        if not self.client.is_connected:
            self.get_logger().error(
                f'Could not connect to rosbridge at {LAPTOP_IP}:{PORT}')
            raise RuntimeError('rosbridge connection failed')

        self.remote_sub = roslibpy.Topic(
            self.client, TOPIC, 'geometry_msgs/Twist')
        self.remote_sub.subscribe(self.on_remote_msg)
        self.get_logger().info(
            f'Bridging {LAPTOP_IP}:{PORT}{TOPIC} -> local {TOPIC}')

    def on_remote_msg(self, msg):
        out = Twist()
        out.linear.x = float(msg['linear']['x'])
        out.linear.y = float(msg['linear']['y'])
        out.linear.z = float(msg['linear']['z'])
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

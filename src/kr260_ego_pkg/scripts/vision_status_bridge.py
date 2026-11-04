#!/usr/bin/env python3
"""Translate YOLO26 HTTP metadata to ROS without moving images through DDS."""
import json
import time
import urllib.request
from rescue_protocol import observation


def main():
    import rclpy
    from rclpy.node import Node
    from kr260_ego_interfaces.msg import PersonObservation

    class Bridge(Node):
        def __init__(self):
            super().__init__('kr260_vision_status_bridge')
            self.url = self.declare_parameter('status_url', 'http://127.0.0.1:8081/status.json').value
            if self.url != 'http://127.0.0.1:8081/status.json':
                raise ValueError('Source monotonic timestamps require the same KR260 host')
            self.pub = self.create_publisher(PersonObservation, '/kr260/rescue/person', 1)
            self.create_timer(.25, self.poll)

        def poll(self):
            msg = PersonObservation()
            try:
                with urllib.request.urlopen(self.url, timeout=.15) as response:
                    raw = response.read(65537)
                if len(raw) > 65536:
                    raise ValueError('Oversized status')
                for key, value in observation(json.loads(raw), time.monotonic_ns()).items():
                    setattr(msg, key, value)
            except (OSError, ValueError, TypeError, KeyError):
                pass  # Empty observation explicitly clears confirmation.
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_image'
            self.pub.publish(msg)

    rclpy.init()
    node = Bridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

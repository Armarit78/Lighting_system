#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RoomControlNode(Node):
    def __init__(self, room_name):
        super().__init__('room_control_node')
        self.room_name = room_name
        self.publisher = self.create_publisher(String, 'adaptive_room_control', 10)
        self.timer = self.create_timer(2.0, self.publish_status)

    def publish_status(self):
        msg = String()
        msg.data = f'{self.room_name}: Normal lighting level'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    room_control_node = RoomControlNode("living_room")
    rclpy.spin(room_control_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


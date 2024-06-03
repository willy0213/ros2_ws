#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64  # 修改为 Float64 消息类型

class DroneCommandNode(Node):
    def __init__(self):
        super().__init__("drone_publisher")
        self.publisher_ = self.create_publisher(Float64, "drone_command", 10)
    
    def publish_command(self, command):
        msg = Float64()
        msg.data = command
        self.publisher_.publish(msg)
        self.get_logger().info(f"Published command: {command}")

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommandNode()
    
    try:
        while rclpy.ok():
            command_str = input("Enter a command to publish: ")
            try:
                command = float(command_str)  # 将输入字符串转换为浮点数
                node.publish_command(command)
            except ValueError:
                node.get_logger().error("Invalid input. Please enter a valid number.")
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

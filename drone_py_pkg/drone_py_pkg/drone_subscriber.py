#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float64  # 确保消息类型与发布者一致
import pymavlink.mavutil as utility
import pymavlink.dialects.v20.all as dialect

class DroneCommandSubscriber(Node):
    def __init__(self):
        super().__init__("drone_subscriber")
        self.subscription = self.create_subscription(
            Float64,
            "drone_command",
            self.listener_callback,
            10
        )
        self.subscription  # 防止未使用变量警告

        self.vehicle = utility.mavlink_connection(device="udpin:127.0.0.1:14550")
        self.get_logger().info("Waiting for heartbeat...")
        self.vehicle.wait_heartbeat()
        self.get_logger().info("Heartbeat received")

    def listener_callback(self, msg):
        if msg.data == 1.0:
            self.get_logger().info("Chang mode to GUIDED")
            self.mode_guided()
        elif msg.data == 2.0:
            self.get_logger().info("Chang mode to LAND")
            self.mode_land()
        else:
            self.get_logger().info("Received unsupported command")

    def mode_guided(self):
        flight_modes = self.vehicle.mode_mapping()
        if 'GUIDED' not in flight_modes:
            self.get_logger().error("GUIDED mode not supported")
            return
        set_mode_message = dialect.MAVLink_command_long_message(
            target_system=self.vehicle.target_system,
            target_component=self.vehicle.target_component,
            command=dialect.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=flight_modes['GUIDED'],
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.vehicle.mav.send(set_mode_message)

    def mode_land(self):
        flight_modes = self.vehicle.mode_mapping()
        if 'LAND' not in flight_modes:
            self.get_logger().error("LAND mode not supported")
            return
        set_mode_message = dialect.MAVLink_command_long_message(
            target_system=self.vehicle.target_system,
            target_component=self.vehicle.target_component,
            command=dialect.MAV_CMD_DO_SET_MODE,
            confirmation=0,
            param1=dialect.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            param2=flight_modes['LAND'],
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
        self.vehicle.mav.send(set_mode_message)

def main(args=None):
    rclpy.init(args=args)
    node = DroneCommandSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from parc_robot_interfaces.msg import CropYield


class YieldTest(Node):
    def __init__(self):
        super().__init__("yield_test")
        self.yield_pub = self.create_publisher(CropYield, "/parc_robot/crop_yield", 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 1

    def timer_callback(self):
        msg = CropYield()

        # Assign counter variable i to crop yield message data
        msg.data = self.i

        # Publish message
        self.yield_pub.publish(msg)

        # Log information to the console
        self.get_logger().info("Current crop yield is: %d" % msg.data)

        # Increment counter variable
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    yield_test = YieldTest()
    rclpy.spin(yield_test)

    yield_test.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

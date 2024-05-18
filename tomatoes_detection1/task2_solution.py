#!/usr/bin/env python3
"""
Script to detect tomatoes
"""
import rclpy
from rclpy.node import Node
from parc_robot_interfaces.msg import CropYield
from std_msgs.msg import String
from sensor_msgs.msg import Image  # Import Image message type
from cv_bridge import CvBridge
import cv2
import numpy as np

from parc_robot_interfaces.msg import CropYield


class TomatoDetector(Node):
    def __init__(self):
        super().__init__('tomato_detector')
        self.publisher_yield = self.create_publisher(CropYield, '/parc_robot/crop_yield', 1)
        self.publisher_status = self.create_publisher(String, '/parc_robot/robot_status', 1)
        self.counter = 0
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.detect_and_publish)
        self.bridge = CvBridge()

        # Subscribe to the right and left camera topics
        self.subscriber_right_camera = self.create_subscription(
            Image, '/right_camera/image_raw', self.right_camera_callback, 10)
        self.subscriber_left_camera = self.create_subscription(
            Image, '/left_camera/image_raw', self.left_camera_callback, 10)

        self.right_camera_data = None
        self.left_camera_data = None

    def right_camera_callback(self, msg):
        self.right_camera_data = msg

    def left_camera_callback(self, msg):
        self.left_camera_data = msg

    def detect_and_publish(self):
        # Check if both camera data is available
        if self.right_camera_data is None or self.left_camera_data is None:
            self.get_logger().warn("Camera data not available.")
            return

        # Convert camera data to OpenCV format
        right_image = self.bridge.imgmsg_to_cv2(self.right_camera_data, desired_encoding="bgr8")
        left_image = self.bridge.imgmsg_to_cv2(self.left_camera_data, desired_encoding="bgr8")

        # Perform tomato detection separately for each camera
        right_count = self.detect_tomatoes(right_image)
        left_count = self.detect_tomatoes(left_image)

        # Increment total count
        count = right_count + left_count
        self.counter += count

        # Publish crop yield
        msg_yield = CropYield()
        msg_yield.data = self.counter
        self.publisher_yield.publish(msg_yield)

        status_msg = String()
        # Publish robot status
        if self.counter > 0:
            status_msg.data = "Started"
        else:
            status_msg.data = "finished"
        self.publisher_status.publish(status_msg)
        self.get_logger().info("Published crop yield: " + str(self.counter) + " tomatoes ")
        self.get_logger().info("Published robot status:  " + str(status_msg) )

    def detect_tomatoes(self, img):
        # Implement tomato detection algorithm using the image data
        # This is where you process the image data to detect tomatoes
        # Example: You can use the same code as before
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        # Continue with tomato detection algorithm...
        # Define thresholds for red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        # Create a mask for red color
        mask = cv2.inRange(hsv_img, lower_red, upper_red)
        # Perform morphological operations (dilation and erosion) to improve accuracy
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.erode(mask, kernel, iterations=1)
        # Find contours of red areas (tomatoes)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Count tomatoes
        count = 0
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Filter out small areas to avoid noise
                count += 1
        return count

def main(args=None):
    rclpy.init(args=args)

    tomato_detector = TomatoDetector()
    rclpy.spin(tomato_detector)
    tomato_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

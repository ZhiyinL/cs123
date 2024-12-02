#!/usr/bin/env python3
"""
ROS2 node for Hailo object detection and tracking
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from sensor_msgs.msg import Image, CompressedImage
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import supervision as sv
import numpy as np
import cv2
import queue
import sys
import os
from typing import Dict, List, Tuple
import threading

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
from utils import HailoAsyncInference

# Our imports


class HailoDetectionNode(Node):
    def __init__(self):
        super().__init__('hailo_detection_node')

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Set up publishers and subscribers
        self.image_pub = self.create_publisher(CompressedImage, '/image', 10) #THIS SENDS IMG TO MAIN
        self.image_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10) #WAIT FOR IMAGE

        # Log completion of Node
        self.get_logger().info("ImageProcessorNode is up and running!")

    def image_callback(self, msg):
        # Convert ROS Image to CV2
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Rotate 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # # Swap r and b channels, then multiply r by 0.5 to fix the colors
        frame = frame[:, :, ::-1]
        frame[:, :, 0] = frame[:, :, 0] * 0.5

        # TODO: optionally, preprocess frame to crop the frame to desired size

        # Compress image into expected jpeg form
        _, jpg_buffer = cv2.imencode('.jpg', frame)
        compressed_img = CompressedImage()
        compressed_img.header = msg.header
        compressed_img.format = "jpeg"
        compressed_img.data = jpg_buffer.tobytes()

        # Publish RGB image
        self.image_pub.publish(compressed_img)

def main(args=None):
    rclpy.init(args=args)
    node = HailoDetectionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down HailoDetectionNode...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
ROS2 node for Hailo object detection and tracking
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
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
        self.color_detection_pub = self.create_publisher(Float32MultiArray, 'color_detecions', 10)

        # Log completion of Node
        self.get_logger().info("ImageProcessorNode is up and running!")

    def image_callback(self, msg):
        print("IN IMAGE CALLBACK")
        # Convert ROS Image to CV2
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        frame_h, frame_w = frame.shape[:2]

        # Rotate 180 degrees
        frame = cv2.rotate(frame, cv2.ROTATE_180)

        # TODO: Do preprocessing on 'frame' here as needed

        # Compress image into expected jpeg form
        _, jpg_buffer = cv2.imencode('.jpg', frame)
        compressed_img = CompressedImage()
        compressed_img.format = "jpeg"
        compressed_img.data = jpg_buffer.tobytes()

        # Publish to downstream
        self.image_pub.publish(compressed_img)

        # Deal with color publishers
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        print(frame[frame_h / 2, frame_w / 2])
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        lower_pink = np.array([140, 100, 100])
        upper_pink = np.array([170, 255, 255])

        mask_yellow = cv2.inRange(hsv_frame, lower_yellow, upper_yellow)
        mask_pink = cv2.inRange(hsv_frame, lower_pink, upper_pink)

        x_coords_yellow = np.where(mask_yellow > 0)[1]
        x_coords_pink = np.where(mask_pink > 0)[1]

        # Compute median x-coordinate
        yellow_x = np.median(x_coords_yellow) if len(x_coords_yellow) > 0 else -1
        pink_x = np.median(x_coords_pink) if len(x_coords_pink) > 0 else -1
        
        color_msg = Float32MultiArray()
        color_msg.data = [float(yellow_x), float(pink_x)]
        self.color_detection_pub.publish(color_msg)

    def find_color_clusters(self, frame : np.ndarray, target_color : tuple, tolerance : int, min_cluster_size : int) -> list :
        """
        Return a list of coordinates (relative to the shape of the frame) to matching color clusters on the image.
        """
        # Color mask to filter out non-target colors (either yellow ball or pink net)
        target_color_array = np.array(target_color)
        lower_bound = np.clip(target_color_array - tolerance, 0, 255)
        upper_bound = np.clip(target_color_array + tolerance, 0, 255)
        mask = np.all((frame >= lower_bound) & (frame <= upper_bound), axis=2)

        # Get coordinates of matching pixels
        y_coords, x_coords = np.where(mask)
        if len(x_coords) == 0:
            return []
        coords = np.column_stack((x_coords, y_coords))

        # Perform clustering with some algorithm TODO
        return []

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

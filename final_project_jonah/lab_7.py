from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np
import time
from std_msgs.msg import Float32MultiArray

IMAGE_WIDTH = 1400

# TODO: Add your new constants here

TIMEOUT = 2.0 #TODO threshold in timer_callback
SEARCH_YAW_VEL = 1.0 #TODO searching constant
TRACK_FORWARD_VEL = 0.5 #TODO tracking constant
TRACK_HORIZONTAL_VEL = 0.1
KP = 0.5 #TODO proportional gain for tracking

class State(Enum):
    SEARCH = 0
    ALIGN = 1
    SHOOT = 2

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.color_detection_subscription = self.create_subscription(
            Float32MultiArray, 
            'color_detections',
            self.color_detection_callback,
            10
        )

        self.yellow_x = None
        self.pink_x = None
        self.latest_color_detection_ts = 0

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.SEARCH

        # TODO: Add your new member variables here
        self.kp = KP 
        self.latest_ts = 0
        self.target_x = 0
        self.image = None # most up-to-date image

    def color_detection_callback(self, msg):
        self.yellow_x = msg.data[0]
        self.pink_x = msg.data[1]
        self.latest_color_detection_ts = time.time()

    def timer_callback(self):
        print("in timer callback")
        # print(f"pink {self.pink_x}")
        # print(f"yellow {self.yellow_x}")

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

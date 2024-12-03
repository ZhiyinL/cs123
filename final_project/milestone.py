from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CompressedImage
from detect import detect_ball
import numpy as np
import time
import cv2
import os

# Planning hyperparameters
SHOOT_THRESHOLD = 0.05 # Within this percentage of image width for the difference between ball and net horizontal center, we shoot

# Control hyperparameters
SEARCH_YAW_VEL = 1.0 #TODO searching constant
TRACK_FORWARD_VEL = 0.3 #TODO tracking x-axis constant
TRACK_HORIZONTAL_VEL = 0.1 #TODO tracking y-axis constant
KP = 0.5 #TODO proportional gain for tracking
AREA_THRESHOLD = 1        

# Other hyperparameters
YELLOW = (0, 150, 255)
PINK = (231, 120, 131)
IMAGE_WIDTH = 1400

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

        self.img_subscription = self.create_subscription(
            CompressedImage,
            '/image',
            self.image_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.SEARCH

        self.kp = KP 
        self.image = None # most up-to-date image
        self.yellow_exist, self.yellow_coord  = False, (0, 0)
        self.normalized_ball_x = None

    def image_callback(self, msg):
        
        self.image = np.frombuffer(msg.data, np.uint8)
        self.image = cv2.imdecode(self.image, cv2.IMREAD_COLOR)

        # update image related features here
        self.yellow_exist, self.yellow_coord, self.yellow_radius = detect_ball(self.image) # TODO: check if jonah's return is normalized
        self.normalized_ball_x = (self.yellow_coord[0] - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2) if self.yellow_exist else None
        print(self.image[self.image.shape[0] // 2, self.image.shape[1] // 2])
        print("Yellow Status: ", self.yellow_exist)
    
    def check_search_to_align(self):
        '''
        We switch state 'SEARCH' to 'ALIGN' when we see both ball and net.
        '''
        return self.yellow_exist

    def check_align_to_shoot(self):
        '''
        We switch state 'ALIGN' to 'SHOOT' when we are in target position to shoot.
        '''
        return np.abs(self.yellow_coord[0]) <= IMAGE_WIDTH * SHOOT_THRESHOLD

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine 
        based on the most recent RGB image input

        states: SEARCH, ALIGN, SHOOT
            SEARCH ---see ball + net--> ALIGN
            ALIGN --in target position--> SHOOT
        """
        
        # Step 1. State Transition
        if self.state == State.SEARCH and self.check_search_to_align():
            print("switching to align!")
            self.state = State.ALIGN
        if self.state == State.ALIGN and self.check_align_to_shoot():
            print("switching to shoot!")
            self.state = State.SHOOT

        print(self.state)

        # Step 2. State Execution
        yaw_command = 0.0
        horizontal_vel_command = 0.0
        forward_vel_command = 0.0
        if self.state == State.SEARCH:
            '''
            rotate towards last time we've seen the ball
            '''
            if not self.normalized_ball_x: 
                yaw_command = SEARCH_YAW_VEL * 1
            else:
                yaw_command = SEARCH_YAW_VEL * (-1 if self.normalized_ball_x > 0 else 1) 
        elif self.state == State.ALIGN:
            '''
            consider move away from ball on x axis (forward / backward) and move towards the ball on y axis (horizontal)
            '''
            forward_vel_command = -TRACK_FORWARD_VEL * 0.2
            horizontal_vel_command = TRACK_HORIZONTAL_VEL * (-1 if self.normalized_ball_x > 0 else 1) 
        elif self.state == State.SHOOT:
            '''
            shoot! assuming we have pupper face ball as well as the net farther away
            '''
            forward_vel_command = TRACK_FORWARD_VEL
            horizontal_vel_command = TRACK_HORIZONTAL_VEL * (-1 if self.normalized_ball_x > 0 else 1) 

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        cmd.linear.y = horizontal_vel_command
        self.command_publisher.publish(cmd) 

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

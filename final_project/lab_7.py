from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np
import time

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

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.annotation_subscription = self.create_subscription(
            CompressedImage,
            '/annotated_images', # todo: check path 
            self.annotation_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.SEARCH

        # TODO: Add your new member variables here
        self.kp = KP 
        self.latest_ts = 0
        self.target_x = 0
        self.image = None # most up-to-date image

    def annotation_callback(self, msg):
        image = msg.data # todo: check ros2 msg
        self.image = image

    def check_search_to_align(self):
        pass

    def check_align_to_shoot(self):
        pass

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        # Step 1. Image Processing (e.g. ball, net detection)
        # find yellow
        # find pink

        # Step 2. Generate Motion
        '''
        states: SEARCH, ALIGN, SHOOT
            SEARCH ---see ball + net--> ALIGN
            ALIGN --in target position--> SHOOT
        '''
        # Step 2.1. State Transition
        # switch search -> align if we see both ball and net, otherwise do nothing
        if self.state == State.SEARCH and check_search_to_align():
            self.state = State.ALIGN
        # switch align -> shoot if we see 
        if self.state == State.ALIGN and check_align_to_shoot():
            self.state = State.SHOOT

        # Step 2.2. State Execution
        yaw_command = 0.0
        horizontal_vel_command = 0.0
        forward_vel_command = 0.0
        if self.state == State.SEARCH:
            # rotate
            yaw_command = SEARCH_YAW_VEL * (-1 if self.target_x > 0 else 1) 
        elif self.state == State.ALIGN:
            # move horizontally
            # TODO: consider move away from ball on x axis and move towards the ball on y axis
            yaw_command = 0.0 # -self.kp*self.target_x # USE CONSTANT IN PT3
            horizontal_vel_command = TRACK_HORIZONTAL_VEL
        elif self.state == State.SHOOT:
            # move forward to shoot
            forward_vel_command = TRACK_FORWARD_VEL

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

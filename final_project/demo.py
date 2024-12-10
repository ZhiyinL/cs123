from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CompressedImage
from detect import detect_ball, detect_net
import numpy as np
import time
import cv2
import os

# Control hyperparameters
SEARCH_YAW_VEL = 0.5 #TODO searching constant
TRACK_FORWARD_VEL = 0.04 #TODO tracking x-axis constant
TRACK_HORIZONTAL_VEL = 0.2 #TODO tracking y-axis constant 
SHOOT_VEL = 0.3
SHOOT_HORIZONTAL_ADJUST = -0.03

IMAGE_WIDTH = 1400

class State(Enum):
    SEARCH = 0
    ALIGN = 1
    SHOOT = 2
    BREATHE = 3

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

        # Time to setup for testing
        time.sleep(10)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.state = State.SEARCH

        self.image = None # most up-to-date image
        self.pink_exist, self.pink_coord = False, (0, 0) # (x, y) coordinate
        self.yellow_exist, self.yellow_coord  = False, (0, 0)
        self.normalized_ball_x = None
        self.normalized_net_x = None

        self.shoot_count = 0
        self.breathe_count = 0

    def image_callback(self, msg):
        
        self.image = np.frombuffer(msg.data, np.uint8)
        self.image = cv2.imdecode(self.image, cv2.IMREAD_COLOR)

        # update image related features here
        self.yellow_exist, self.yellow_coord, self.yellow_radius = detect_ball(self.image) # TODO: check if jonah's return is normalized
        self.normalized_ball_x = (self.yellow_coord[0] - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2) if self.yellow_exist else None
        self.pink_exist, self.pink_coord, self.pink_radius = detect_net(self.image)
        self.normalized_net_x = (self.pink_coord[0] - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2) if self.pink_exist else None
        print("Yellow Status: ", self.yellow_exist, " yellow x: ", self.normalized_ball_x)
        print("Pink Status: ", self.pink_exist, " pink x: ", self.normalized_net_x)

    def aligned_to_shoot(self):
        '''
        We switch state 'ALIGN' to 'SHOOT' when we are in target position to shoot.
        '''
        ball = np.abs(self.normalized_ball_x)
        net = np.abs(self.normalized_net_x)
        return ball < 0.04 and net < 0.02 and np.abs(ball - net) < 0.04

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine 
        based on the most recent RGB image input

        states: SEARCH, ALIGN, SHOOT
            SEARCH ---see ball + net--> ALIGN
            ALIGN --in target position--> SHOOT
        """
        # Step 0. Sleep when shooting
        if self.state == State.SHOOT:
            self.shoot_count += 1
            print("STATE: SHOOT COMMITTED")
            if self.shoot_count % 20 != 0:
                return
            
            
        # Step 1. State Transition
        if not self.yellow_exist or not self.pink_exist:
            self.state = State.SEARCH
        elif not self.aligned_to_shoot():
            self.state = State.ALIGN
        elif self.breathe_count < 10:
            self.state = State.BREATHE
        else:
            self.state = State.SHOOT

        print("State: ", self.state)

        # Step 2. State Execution
        yaw_command = 0.0
        horizontal_vel_command = 0.0
        forward_vel_command = 0.0
        if self.state == State.SEARCH:
            '''
            rotate towards last time we've seen the ball
            '''
            if self.normalized_ball_x is None or self.normalized_ball_x < 0: 
                print("turning  left")
                yaw_command = SEARCH_YAW_VEL
            else:
                print("turning right")
                yaw_command = -SEARCH_YAW_VEL
        
        elif self.state == State.ALIGN:
            '''
            consider move away from ball on x axis (forward / backward) and move towards the ball on y axis (horizontal)
            '''
            # Account for failed shot attempt

            if not self.normalized_ball_x is None:
                # if aligned but off-center, turn. Angular velocity is half of search speed for accuracy
                if np.abs(self.normalized_ball_x - self.normalized_net_x) < 0.05:
                    print("Align - turning")
                    yaw_command = SEARCH_YAW_VEL * (-1 if self.normalized_ball_x > 0 else 1)
                    yaw_command = 0.1 * yaw_command + 0.9 * abs(self.normalized_net_x)

                else: # Keep moving backwards and sideways
                    print("Align - walking")
                    forward_vel_command = -TRACK_FORWARD_VEL
                    horizontal_vel_command = TRACK_HORIZONTAL_VEL * (-1 if self.normalized_ball_x > 0 else 1)
                    scaling_factor = abs(self.normalized_ball_x - self.normalized_net_x) / 2.0
                    forward_vel_command = 0.25 * forward_vel_command + 0.75 * forward_vel_command * scaling_factor
                    horizontal_vel_command = 0.25 * horizontal_vel_command + 0.75 * horizontal_vel_command * scaling_factor

        elif self.state == State.BREATHE:
            self.stop_moving()
            self.breathe_count += 1
            return

        elif self.state == State.SHOOT:
            '''
            shoot! assuming we have pupper face ball as well as the net farther away
            '''

            if not self.normalized_ball_x is None:
                forward_vel_command = SHOOT_VEL
                horizontal_vel_command = SHOOT_HORIZONTAL_ADJUST
                self.shoot_count = 0
        
        
        self.breathe_count = 0

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        cmd.linear.y = horizontal_vel_command
        print(f"angular vel: {cmd.angular.z}, x vel: {cmd.linear.x}, y vel: {cmd.linear.y}")
        self.command_publisher.publish(cmd)

    def stop_moving(self):
        cmd = Twist()
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

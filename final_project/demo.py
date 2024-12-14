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
from just_playback import Playback
import threading

# Control hyperparameters
SEARCH_YAW_VEL = 0.5 #TODO searching constant
TRACK_FORWARD_VEL = 0.03 #TODO tracking x-axis constant
TRACK_HORIZONTAL_VEL = 0.2 #TODO tracking y-axis constant 
SHOOT_VEL = 0.5
SHOOT_HORIZONTAL_ADJUST = -0.05

IMAGE_WIDTH = 1400
HOME_SOUND = "/home/pi/cs123/final_project/sounds/"

class SoundModule:
    def __init__(self):
        self.playback = Playback()
        self.current_sound_file = None
        self.has_howled = False

    def play(self, sound_file):
        # Check if the same sound is already playing
        if self.playback.playing and self.current_sound_file == sound_file:
            return
        
        # Stop current playback if a different sound is requested
        self.stop()

        # Load and play the new sound file
        self.current_sound_file = sound_file
        self.playback.load_file(sound_file)
        self.playback.play()
    
    def stop(self):
        """Stops the currently playing sound."""
        if self.playback.playing:
            self.playback.stop()
            self.current_sound_file = None

    def bark(self):
        print("bark!")
        sound_file = f"{HOME_SOUND}/dog_bark.wav"
        self.play(sound_file)

    def howl(self):  # Only play once for every initialized sound module 
        if not self.has_howled:
            print("howl!")
            sound_file = f"{HOME_SOUND}/howl.wav"
            self.play(sound_file)
            self.has_howled = True

    def messi(self):
        print("messi!")
        sound_file = f"{HOME_SOUND}/messi.wav"
        self.play(sound_file)

    def jeopardy(self):
        print("jeopardy!")
        sound_file = f"{HOME_SOUND}/jeopardy.wav"
        self.play(sound_file)

    def siu(self):
        print("siu!")
        sound_file = f"{HOME_SOUND}/siu.wav"
        self.play(sound_file)

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
        time.sleep(20)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.state = State.SEARCH

        self.image = None # most up-to-date image
        self.pink_exist, self.pink_coord = False, (0, 0) # (x, y) coordinate
        self.yellow_exist, self.yellow_coord  = False, (0, 0)
        self.normalized_ball_x = None
        self.normalized_net_x = None
        self.prev_ball_x = None

        self.shoot_count = 0
        self.breathe_count = 0
        self.debug_idx = 0

        # sound module
        self.sound_module = SoundModule()

    def image_callback(self, msg):
        
        self.image = np.frombuffer(msg.data, np.uint8)
        self.image = cv2.imdecode(self.image, cv2.IMREAD_COLOR)

        # update image related features here
        self.yellow_exist, self.yellow_coord, self.yellow_radius = detect_ball(self.image)
        if self.normalized_ball_x is not None:
            self.prev_ball_x == self.normalized_ball_x
        self.normalized_ball_x = (self.yellow_coord[0] - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2) if self.yellow_exist else None
        self.pink_exist, self.pink_coord, self.pink_width = detect_net(self.image)
        self.normalized_net_x = (self.pink_coord[0] - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2) if self.pink_exist else None
        print("Yellow Status: ", self.yellow_exist, " yellow x: ", self.normalized_ball_x, "yellow radius: ", self.yellow_radius)
        print("Pink Status: ", self.pink_exist, " pink x: ", self.normalized_net_x, "pink width: ", self.pink_width)

    def aligned_to_shoot(self):
        '''
        We switch state 'ALIGN' to 'SHOOT' when we are in target position to shoot.
        '''
        aligned_far = np.abs(self.normalized_ball_x) < 0.05 and np.abs(self.normalized_net_x) < 0.05 and np.abs(self.normalized_ball_x - self.normalized_net_x) < 0.05
        # relax parameters if really close since we lose accurate track of ball VALUES TESTED THROUGH IMAGE
        aligned_close = self.yellow_radius > 115 and self.pink_width > 150 and np.abs(self.normalized_ball_x) < 0.1 and \
        np.abs(self.normalized_net_x) < 0.1 and np.abs(self.normalized_ball_x - self.normalized_net_x) < 0.1

        return aligned_close or aligned_far


    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine 
        based on the most recent RGB image input

        states: SEARCH, ALIGN, SHOOT
            SEARCH ---see ball + net--> ALIGN
            ALIGN --in target position--> SHOOT
        """ 
        # Step 1. State Transition
        if self.state == State.SHOOT and self.shoot_count % 8 != 0:
            print("skipping")
            print(self.shoot_count)
        elif not self.yellow_exist or not self.pink_exist:
            self.state = State.SEARCH
        elif not self.aligned_to_shoot():
            self.state = State.ALIGN
        elif self.breathe_count < 3:
            self.state = State.BREATHE
        else:
            # self.breathe_count = 0
            self.state = State.SHOOT
            self.shoot_count = 1

        if self.state != State.BREATHE:
            self.breathe_count = 0
        print("State: ", self.state)

        # Step 1.5 Sound Module
        if self.state == State.SEARCH:
            self.sound_module.jeopardy()
        elif self.state == State.ALIGN:
            self.sound_module.howl() # will only activate the first it tries to howl
            self.sound_module.messi()
        elif self.state == State.BREATHE:
            self.sound_module.stop()
        elif self.state == State.SHOOT:
            self.sound_module.bark()
        
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

            if not self.normalized_ball_x is None and not self.normalized_net_x is None:
                # if aligned but off-center, turn. Angular velocity is half of search speed for accuracy
                
                target_x = 0.5 * (self.normalized_ball_x + self.normalized_net_x)
                if np.abs(target_x) < 0.05:
                    print("Align - walking")
                    cv2.imwrite(f'./images/all/{self.debug_idx}_align_walking.jpg', self.image)
                    self.debug_idx += 1
                    # Walk forward or back towards ball threshold is at 0.25 because of fisheye (TESTED)
                    if np.abs(self.normalized_ball_x) > 0.25:
                        forward_vel_command = -TRACK_FORWARD_VEL
                    elif np.abs(self.normalized_ball_x) < 0.15:
                        forward_vel_command = TRACK_FORWARD_VEL

                    # Walk left or right towards ball
                    if self.normalized_ball_x > self.normalized_net_x:
                        horizontal_vel_command = -TRACK_HORIZONTAL_VEL
                    else:
                        horizontal_vel_command = TRACK_HORIZONTAL_VEL
                else:
                    print("Align - turning")
                    cv2.imwrite(f'./images/all/{self.debug_idx}_align_turning.jpg', self.image)
                    self.debug_idx += 1
                    if target_x < -0.05: # Attempt to correct for left turn bias
                        yaw_command = SEARCH_YAW_VEL * 0.5
                    else:
                        yaw_command = -SEARCH_YAW_VEL * 0.5
                
                    # horizontal_vel_command = TRACK_HORIZONTAL_VEL * (-1 if self.normalized_ball_x > 0 else 1)
                    # scaling_factor = abs(self.normalized_ball_x - self.normalized_net_x) / 2.0
                    # forward_vel_command = 0.25 * forward_vel_command + 0.75 * forward_vel_command * scaling_factor
                    # horizontal_vel_command = 0.25 * horizontal_vel_command + 0.75 * horizontal_vel_command * scaling_factor

        elif self.state == State.BREATHE:
            cv2.imwrite(f'./images/all/{self.debug_idx}_align_breathe.jpg', self.image)
            self.debug_idx += 1
            self.breathe_count += 1

        elif self.state == State.SHOOT:
            '''
            shoot! assuming we have pupper face ball as well as the net farther away
            '''
            print("SHOOTING")
            cv2.imwrite(f'./images/all/{self.debug_idx}_align_shoot.jpg', self.image)
            self.debug_idx += 1
            
            if self.normalized_ball_x:
                yaw_command = - SEARCH_YAW_VEL * 10.0 * self.normalized_ball_x
            elif self.prev_ball_x:
                yaw_command = - SEARCH_YAW_VEL * 10.0 * self.prev_ball_x
            forward_vel_command = SHOOT_VEL * max(0, min(1, (1 - 2 * np.abs(self.normalized_ball_x))))
            print("yaw_command, forward_vel:", yaw_command, forward_vel_command)
            # horizontal_vel_command = SHOOT_HORIZONTAL_ADJUST - TRACK_HORIZONTAL_VEL * (self.normalized_ball_x)
            self.shoot_count += 1

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

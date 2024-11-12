from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np
import time

IMAGE_WIDTH = 1400

# TODO: Add your new constants here

TIMEOUT = 5.0 #TODO threshold in timer_callback
SEARCH_YAW_VEL = 1.0 #TODO searching constant
TRACK_FORWARD_VEL = 1.0 #TODO tracking constant
KP = 0.1 #TODO proportional gain for tracking

class State(Enum):
    SEARCH = 0
    TRACK = 1

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

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.TRACK

        # TODO: Add your new member variables here
        self.kp = KP 
        self.latest_ts = 0

    def detection_callback(self, msg):
        """
        Determine which of the HAILO detections is the most central detected object
        """
        
        # Part 1.3, 1.5: Find the most central detection
        if msg.detections:
            normalized_x_positions = [
                (detection.bbox.center.x - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2)
                for detection in msg.detections
            ]
            most_central_index = np.argmin(np.abs(normalized_x_positions))
            self.target_x = normalized_x_positions[most_central_index]

        # part 1.4 print x coordinate
        for i, x_pos in enumerate(normalized_x_positions):
            print(f"coordinate x_{i} normalized ", x_pos)

        # Update time of last detection
        self.latest_ts = time.time()

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        dt = time.time() - self.latest_ts
        if dt > TIMEOUT: # TODO: Part 3.2
            self.state = State.SEARCH
        else:
            self.state = State.TRACK
        
        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.SEARCH:
            # TODO: Part 3.1
            yaw_command = SEARCH_YAW_VEL * (-1 if self.target_x > 0 else 1)
            
        elif self.state == State.TRACK:
            # TODO: Part 2 / 3.4
            yaw_command = -self.kp*self.target_x # USE CONSTANT IN PT3
            forward_vel_command = TRACK_FORWARD_VEL

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
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

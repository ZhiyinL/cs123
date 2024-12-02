from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
from sensor_msgs.msg import CompressedImage
import numpy as np
import time
import cv2
import os

# Planning hyperparameters
SHOOT_THRESHOLD = 0.05 # Within this percentage of image width for the difference between ball and net horizontal center, we shoot

# Control hyperparameters
SEARCH_YAW_VEL = 1.0 #TODO searching constant
TRACK_FORWARD_VEL = 0.5 #TODO tracking x-axis constant
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
        self.pink_exist, self.pink_coord = False, (0, 0) # (x, y) coordinate
        self.yellow_exist, self.yellow_coord  = False, (0, 0)
        self.normalized_ball_x = None
        self.normalized_net_x = None

    def image_callback(self, msg):
        
        self.image = np.frombuffer(msg.data, np.uint8)
        self.image = cv2.imdecode(self.image, cv2.IMREAD_COLOR)

        # update image related features here
        self.pink_exist, self.pink_coord = self.detect_color(PINK)
        self.yellow_exist, self.yellow_coord = self.detect_color(YELLOW)
        self.normalized_ball_x = (self.yellow_coord[0] - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2) if self.yellow_exist else None
        self.normalized_net_x = (self.pink_coord[0] - IMAGE_WIDTH / 2) / (IMAGE_WIDTH / 2) if self.pink_exist else None
        print(self.image[self.image.shape[0] // 2, self.image.shape[1] // 2])
        print("Yellow Status: ", self.yellow_exist)
        print("Pink Status: ", self.pink_exist)

    def detect_color(self, color, tol=50, save_folder="./images"):
        """
        Return whether in current image the color of interest exists
        and the median coordinates of the color of interest.
        """
        assert(self.image.shape[2] == 3) # RGB channels only, should equal to 3

        # Ensure the target color is a NumPy array
        color_bgr = np.array(color, dtype=np.uint8)  # Ensure the color is an array
        color_bgr = color_bgr.reshape(1, 1, 3)  # Reshape to 1x1 image (BGR format)
        color_hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)[0][0]  # Convert to HSV
        
        # Define lower and upper bounds for the color range
        lower_bound = np.clip(color_hsv - tol, 0, 255)
        upper_bound = np.clip(color_hsv + tol, 0, 255)

        # Create a mask for pixels within the color range
        mask = cv2.inRange(self.image, lower_bound, upper_bound)
        
        # Convert the mask to a binary image for blob detection (0 or 255)
        binary_mask = np.uint8(mask) * 255  # Convert boolean to binary (255 for match, 0 for non-match)
        
        # Set up the SimpleBlobDetector parameters
        params = cv2.SimpleBlobDetector_Params()

        # Filter by area (adjust this as needed)
        params.filterByArea = True
        params.minArea = AREA_THRESHOLD  # Minimum blob size in pixels

        # Filter by circularity (optional)
        params.filterByCircularity = False

        # Filter by convexity (optional)
        params.filterByConvexity = False

        # Filter by inertia (optional)
        params.filterByInertia = False

        # Create a blob detector with the parameters
        detector = cv2.SimpleBlobDetector_create(params)

        # Detect blobs
        keypoints = detector.detect(binary_mask)

        # If no blobs are detected, return (False, None)
        if len(keypoints) == 0:
            return False, None
        
        # Sort the keypoints by area (largest first)
        keypoints = sorted(keypoints, key=lambda kp: kp.size, reverse=True)

        # The largest blob (first in the sorted list)
        largest_blob = keypoints[0]
        
        # Check if the largest blob's area is above the threshold
        if largest_blob.size >= AREA_THRESHOLD:
            # Extract the coordinates (center) of the largest blob
            largest_blob_coords = largest_blob.pt  # (x, y) coordinates
            largest_blob_area = largest_blob.size  # Area of the blob
            print("largest_blob_area", largest_blob_area)
            # Weighted coin flip to decide whether to save the image
            flip = np.random.choice([True, False], p=[0.1, 0.9])  # 2% chance of saving the image

            if flip:
                # Draw keypoints on the image
                
                image_with_blobs = cv2.drawKeypoints(self.image, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

                # Save the image to the specified folder
                if not os.path.exists(save_folder):
                    os.makedirs(save_folder)  # Create the folder if it doesn't exist

                save_path = os.path.join(save_folder, "blob_detected.jpg")
                cv2.imwrite(save_path, image_with_blobs)
                print(f"Image with blobs saved to: {save_path}")

                # Display the image
                cv2.imshow("Detected Blobs", image_with_blobs)
                cv2.waitKey(1)  # Wait for a key event to display the image

            # Return True and the median of the largest blob's coordinates
            return True, tuple(np.round(largest_blob_coords).astype(int))
        else:
            return False, None
    
    def check_search_to_align(self):
        '''
        We switch state 'SEARCH' to 'ALIGN' when we see both ball and net.
        '''
        return self.yellow_exist and self.pink_exist

    def check_align_to_shoot(self):
        '''
        We switch state 'ALIGN' to 'SHOOT' when we are in target position to shoot.
        '''
        return np.abs(self.yellow_coord[0] - self.pink_coord[0]) <= IMAGE_WIDTH * SHOOT_THRESHOLD

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
            self.state = State.ALIGN
        if self.state == State.ALIGN and self.check_align_to_shoot():
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
            forward_vel_command = -TRACK_FORWARD_VEL
            horizontal_vel_command = TRACK_HORIZONTAL_VEL * (-1 if self.normalized_ball_x > 0 else 1) 
        elif self.state == State.SHOOT:
            '''
            shoot! assuming we have pupper face ball as well as the net farther away
            '''
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

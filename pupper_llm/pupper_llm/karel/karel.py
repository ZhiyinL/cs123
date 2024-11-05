# karel.py
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import simpleaudio as sa
import pygame

class KarelPupper:
    def start():
        rclpy.init()

    def __init__(self):
        rclpy.init()
        self.node = Node('karel_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def move(self):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move forward...')
        time.sleep(1)
        self.stop()

    def turn_left(self):
        pass
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 1.5 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Turn left...')
        time.sleep(1)
        self.stop()

    def turn_right(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -1.5 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Turn right...')
#        time.sleep(0.5)
        self.stop()

    def bark(self):
        self.node.get_logger().info('Bark...')
        pygame.mixer.init()
        # bark_sound = pygame.mixer.Sound('/home/pi/pupper_llm/sounds/dog_bark.wav')
        bark_sound = pygame.mixer.Sound('/home/pi/Desktop/cs123/pupper_llm/sounds/dog_bark.wav')
        bark_sound.play()
        
#        time.sleep(0.5)
        self.stop()


    def stop(self):
        self.node.get_logger().info('Stopping...')
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
    
    def __del__(self):
        self.node.get_logger().info('Tearing down...')
        self.node.destroy_node()
        rclpy.shutdown()

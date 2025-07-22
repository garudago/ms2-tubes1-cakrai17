#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random


class TwistCommandRandomizer(Node):
    def __init__(self):
        super().__init__('twist_command_randomizer')
        
        # Declare parameters for topic names
        self.declare_parameter('autonomous_vel_topic', 'autonomous_vel')
        self.declare_parameter('joy_vel_topic', 'joy_vel')
        self.declare_parameter('keyboard_vel_topic', 'keyboard_vel')
        self.declare_parameter('publish_rate', 1.0)  # Hz

        # Get parameter values
        autonomous_topic = self.get_parameter('autonomous_vel_topic').get_parameter_value().string_value
        joy_topic = self.get_parameter('joy_vel_topic').get_parameter_value().string_value
        keyboard_topic = self.get_parameter('keyboard_vel_topic').get_parameter_value().string_value
        publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        
        # Create publishers
        self.autonomous_pub = self.create_publisher(Twist, autonomous_topic, 10)
        self.joy_pub = self.create_publisher(Twist, joy_topic, 10)
        self.keyboard_pub = self.create_publisher(Twist, keyboard_topic, 10)
        
        # Create timer
        self.timer = self.create_timer(1.0 / publish_rate, self.timer_callback)
        
        self.get_logger().info(f'Twist Command Randomizer started!')
        self.get_logger().info(f'Publishing to:')
        self.get_logger().info(f'  - {joy_topic} (always)')
        self.get_logger().info(f'  - {autonomous_topic} (occasionally)')
        self.get_logger().info(f'  - {keyboard_topic} (occasionally)')
        self.get_logger().info(f'  - Publishing rate: {publish_rate} Hz')
    
    def generate_random_twist(self):
        """Generate a random Twist message simulating directional movements"""
        twist = Twist()
        
        # Simulate directional movements (up, down, left, right)
        movements = [
            (1.0, 0.0),   # Forward
            (-1.0, 0.0),  # Backward
            (0.0, 1.0),   # Left (angular)
            (0.0, -1.0),  # Right (angular)
            (0.5, 0.5),   # Forward + Left
            (0.5, -0.5),  # Forward + Right
            (0.0, 0.0),   # Stop
        ]
        
        linear_x, angular_z = random.choice(movements)
        
        # Add some randomness to the values
        twist.linear.x = linear_x * random.uniform(0.5, 1.5)
        twist.angular.z = angular_z * random.uniform(0.5, 1.5)
        
        return twist
    
    def timer_callback(self):
        """Timer callback that publishes twist messages"""
        
        # Always publish to joy_vel
        autonomous_twist = self.generate_random_twist()
        self.autonomous_pub.publish(autonomous_twist)

        
        # Occasionally publish to joy_vel
        if random.random() > 0.8:
            joy_twist = self.generate_random_twist()
            self.joy_pub.publish(joy_twist)


        # Occasionally publish to keyboard_vel
        if random.random() > 0.8:
            keyboard_twist = self.generate_random_twist()
            self.keyboard_pub.publish(keyboard_twist)



def main(args=None):
    rclpy.init(args=args)
    node = TwistCommandRandomizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
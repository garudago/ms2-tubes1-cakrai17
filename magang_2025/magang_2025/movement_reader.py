#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math


class MovementReader(Node):
    def __init__(self):
        super().__init__('movement_reader')
        
        # Declare parameters for topic names
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('cmd_type_topic', 'cmd_type')
        
        # Get parameter values
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        cmd_type_topic = self.get_parameter('cmd_type_topic').get_parameter_value().string_value
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )
        
        self.cmd_type_sub = self.create_subscription(
            String,
            cmd_type_topic,
            self.cmd_type_callback,
            10
        )
        
        # Store the latest command type
        self.current_cmd_type = "unknown"
        
        self.get_logger().info(f'Movement Reader started!')
        self.get_logger().info(f'Subscribing to:')
        self.get_logger().info(f'  - {cmd_vel_topic} (Twist)')
        self.get_logger().info(f'  - {cmd_type_topic} (String)')
        self.get_logger().info('=' * 60)
    
    def cmd_type_callback(self, msg):
        """Callback for command type messages"""
        self.current_cmd_type = msg.data
        self.get_logger().debug(f'Command type updated: {self.current_cmd_type}')
    
    def analyze_movement(self, twist):
        """Analyze the twist message and return movement description"""
        linear_x = twist.linear.x
        angular_z = twist.angular.z
        
        # Threshold for considering movement significant
        linear_threshold = 0.1
        angular_threshold = 0.1
        
        movements = []
        
        # Analyze linear movement
        if abs(linear_x) > linear_threshold:
            if linear_x > 0:
                movements.append("FORWARD")
            else:
                movements.append("BACKWARD")
        
        # Analyze angular movement
        if abs(angular_z) > angular_threshold:
            if angular_z > 0:
                movements.append("LEFT")
            else:
                movements.append("RIGHT")
        
        # Determine overall movement description
        if not movements:
            return "STOPPED"
        elif len(movements) == 1:
            return movements[0]
        else:
            return " + ".join(movements)
    
    def get_movement_emoji(self, movement):
        """Return emoji representation of movement"""
        emoji_map = {
            "FORWARD": "⬆️",
            "BACKWARD": "⬇️", 
            "LEFT": "⬅️",
            "RIGHT": "➡️",
            "STOPPED": "⏹️",
            "FORWARD + LEFT": "↖️",
            "FORWARD + RIGHT": "↗️",
            "BACKWARD + LEFT": "↙️",
            "BACKWARD + RIGHT": "↘️"
        }
        return emoji_map.get(movement, "❓")
    
    def get_source_emoji(self, cmd_type):
        """Return emoji representation of command source"""
        emoji_map = {
            "autonomous": "🤖",
            "joy": "🎮",
            "keyboard": "⌨️",
            "unknown": "❓"
        }
        return emoji_map.get(cmd_type, "❓")
    
    def cmd_vel_callback(self, msg):
        """Callback for cmd_vel messages"""
        # Analyze the movement
        movement = self.analyze_movement(msg)
        
        # Get emojis for better visualization
        movement_emoji = self.get_movement_emoji(movement)
        source_emoji = self.get_source_emoji(self.current_cmd_type)
        
        # Create detailed log message
        log_msg = (
            f"{source_emoji} {self.current_cmd_type.upper():>10} → {movement_emoji} {movement:<15} | "
            f"Lin: {msg.linear.x:+6.2f} m/s, Ang: {msg.angular.z:+6.2f} rad/s"
        )
        
        # Log with different levels based on movement
        if movement == "STOPPED":
            self.get_logger().debug(log_msg)
        else:
            self.get_logger().info(log_msg)
        
        # Additional validation logging
        if self.current_cmd_type not in ["autonomous", "joy", "keyboard"]:
            self.get_logger().warn(f"⚠️  Unknown command type: '{self.current_cmd_type}' - Expected: autonomous/joy/keyboard")


def main(args=None):
    rclpy.init(args=args)
    node = MovementReader()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
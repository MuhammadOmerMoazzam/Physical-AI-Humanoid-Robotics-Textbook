# raise_controller_ros2/raise_controller.py
import rclpy
from rclpy.node import Node
import numpy as np
import torch
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray

class RAISEController(Node):
    def __init__(self):
        super().__init__('raise_controller')
        
        # Load the pretrained RAISE policy
        self.policy = self.load_raise_policy("unitree_g1_walk_v2025")
        
        # Publishers and subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Timer for control loop (100 Hz)
        self.control_timer = self.create_timer(0.01, self.run_policy)
        
        self.get_logger().info('RAISE Controller initialized')

    def load_raise_policy(self, policy_name):
        """Load the pretrained RAISE policy"""
        # In practice, this would load a PyTorch model
        # For now, we'll create a placeholder
        self.get_logger().info(f'Loading RAISE policy: {policy_name}')
        return lambda obs: np.random.randn(36)  # Placeholder for 36 DoF

    def cmd_callback(self, msg):
        """Store the velocity command"""
        self.desired_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def run_policy(self):
        """Run the RAISE policy at 100 Hz"""
        # Get current observation (simplified)
        obs = self.get_current_observation()
        
        # Get action from policy
        action = self.policy(obs)  # 36 joint torques at 100 Hz
        
        # Publish joint commands
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.effort = action.tolist()  # Using effort control
        self.joint_pub.publish(joint_msg)

    def get_current_observation(self):
        """Get current robot state observation"""
        # Simplified observation including joint states, IMU, etc.
        obs_size = 100  # Simplified for example
        obs = np.random.randn(obs_size)
        return obs

def main():
    rclpy.init()
    node = RAISEController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
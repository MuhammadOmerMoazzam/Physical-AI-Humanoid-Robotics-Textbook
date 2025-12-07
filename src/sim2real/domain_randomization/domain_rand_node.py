# domain_randomization/domain_rand_node.py
import rclpy
from rclpy.node import Node
import random
import numpy as np
from std_msgs.msg import Float64

class DomainRandNode(Node):
    def __init__(self):
        super().__init__('domain_rand_node')
        
        # Initialize randomization ranges
        self.dr_ranges = {
            "mass":            (0.70, 1.40),   # × nominal
            "com_offset":      (-0.03, 0.03),  # meters
            "friction":        (0.4, 1.6),
            "restitution":     (0.0, 0.6),
            "joint_damping":   (0.5, 3.0),
            "joint_stiffness": (0.8, 1.3),
            "latency_ms":      (20, 120),
            "sensor_noise":    True,
            "lighting":        "random_hdri",
            "texture_variation": True
        }
        
        # Publishers for randomized parameters
        self.mass_pub = self.create_publisher(Float64, '/randomized_mass', 10)
        self.friction_pub = self.create_publisher(Float64, '/randomized_friction', 10)
        
        # Timer to update randomization (every 5 seconds)
        self.rand_timer = self.create_timer(5.0, self.update_randomization)
        
        self.get_logger().info('Domain Randomization Node initialized')

    def update_randomization(self):
        """Update randomization parameters"""
        # Sample random values from ranges
        mass_factor = random.uniform(*self.dr_ranges["mass"])
        friction_factor = random.uniform(*self.dr_ranges["friction"])
        
        # Publish randomized parameters
        mass_msg = Float64()
        mass_msg.data = mass_factor
        self.mass_pub.publish(mass_msg)
        
        friction_msg = Float64()
        friction_msg.data = friction_factor
        self.friction_pub.publish(friction_msg)
        
        self.get_logger().info(f'Randomized: mass×{mass_factor:.2f}, friction×{friction_factor:.2f}')

def main():
    rclpy.init()
    node = DomainRandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
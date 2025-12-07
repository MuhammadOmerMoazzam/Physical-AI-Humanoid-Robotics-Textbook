# src/vla/openvla_inference/openvla_node.py
import rclpy
from rclpy.node import Node
import torch
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String
from geometry_msgs.msg import Pose
import openvla

class OpenVLANode(Node):
    def __init__(self):
        super().__init__('openvla_node')
        
        # Initialize OpenVLA model
        self.model = openvla.OpenVLA.from_pretrained("openvla-7b")
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model.to(self.device)
        self.model.eval()
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Publishers and subscribers
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        self.command_sub = self.create_subscription(String, '/natural_language_command', self.command_callback, 10)
        self.joint_pub = self.create_publisher(JointState, '/humanoid/joint_commands', 10)
        
        # Current command and image
        self.current_command = "default: do nothing"
        self.current_image = None
        
        # Timer for inference (10 Hz - conservative for 7B model)
        self.inference_timer = self.create_timer(0.1, self.run_inference)
        
        self.get_logger().info('OpenVLA Node initialized')

    def image_callback(self, msg):
        """Receive and store camera image"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def command_callback(self, msg):
        """Receive and store natural language command"""
        self.current_command = msg.data
        self.get_logger().info(f'Received command: {self.current_command}')

    def run_inference(self):
        """Run VLA inference if both image and command are available"""
        if self.current_image is not None and self.current_command != "default: do nothing":
            try:
                # Run inference
                action = self.model.predict(self.current_image, self.current_command)
                
                # Convert action to joint commands
                joint_commands = self.action_to_joints(action)
                
                # Publish joint commands
                joint_msg = JointState()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.name = [f'joint_{i}' for i in range(len(joint_commands))]
                joint_msg.position = joint_commands
                self.joint_pub.publish(joint_msg)
                
                self.get_logger().info(f'Published action for command: {self.current_command}')
            except Exception as e:
                self.get_logger().error(f'Error running inference: {e}')

    def action_to_joints(self, action):
        """Convert VLA output action to joint positions"""
        # Simplified conversion - in practice, this would be specific to your robot
        # The VLA outputs action deltas for the robot
        n_joints = 36  # For humanoid
        if isinstance(action, torch.Tensor):
            action = action.cpu().numpy()
        
        if len(action) < n_joints:
            # Pad with zeros if needed
            padded_action = np.zeros(n_joints)
            padded_action[:len(action)] = action
            return padded_action.tolist()
        else:
            return action[:n_joints].tolist()

def main():
    rclpy.init()
    node = OpenVLANode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
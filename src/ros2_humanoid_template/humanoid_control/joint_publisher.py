# humanoid_control/joint_publisher.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class SimpleHumanoidPublisher(Node):
    def __init__(self):
        super().__init__('simple_humanoid_publisher')
        self.pub = self.create_publisher(
            JointState, '/humanoid/joint_commands', 10)
        timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['waist', 'left_hip', 'right_hip', 'head']
        msg.position = [0.0, 0.5 * (self.i % 100)/50, -0.5, 0.3]
        self.pub.publish(msg)
        self.i += 1

def main():
    rclpy.init()
    node = SimpleHumanoidPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
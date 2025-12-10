"""Joint publisher node for humanoid control."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class JointPublisher(Node):
    def __init__(self):
        super().__init__('joint_publisher')
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = JointState()
        msg.name = ['joint1', 'joint2', 'joint3']
        msg.position = [0.0, 0.0, 0.0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing joint states: "%s"' % msg.position)
        self.i += 1
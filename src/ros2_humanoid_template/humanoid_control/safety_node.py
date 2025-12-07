# humanoid_control/safety_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import os

class HumanoidSafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node', allow_undeclared_parameters=True)
        self.last_heartbeat = self.get_clock().now()
        # self.create_subscription(Heartbeat, '/heartbeat', self.cb, 10)  # Heartbeat msg type would need to be defined
        self.create_timer(0.1, self.safety_check)

    def cb(self, msg):
        self.last_heartbeat = self.get_clock().now()

    def safety_check(self):
        if (self.get_clock().now() - self.last_heartbeat).nanoseconds > 200_000_000:  # 200ms
            self.get_logger().fatal("HEARTBEAT LOST → E-STOP")
            # os.system("ros2 topic pub /emergency_stop std_msgs/Bool '{data: true}'")

def main():
    rclpy.init()
    node = HumanoidSafetyNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
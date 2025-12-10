"""Safety node for humanoid control."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Header
from builtin_interfaces.msg import Time


class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')
        self.last_heartbeat = self.get_clock().now()
        self.heartbeat_sub = self.create_subscription(
            Bool,
            '/heartbeat',
            self.heartbeat_callback,
            10
        )
        self.emergency_stop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.safety_timer = self.create_timer(0.1, self.safety_check)

    def heartbeat_callback(self, msg):
        self.last_heartbeat = self.get_clock().now()

    def safety_check(self):
        if (self.get_clock().now() - self.last_heartbeat).nanoseconds > 200_000_000:  # 200ms
            self.get_logger().fatal("HEARTBEAT LOST -> E-STOP")
            stop_msg = Bool()
            stop_msg.data = True
            self.emergency_stop_pub.publish(stop_msg)
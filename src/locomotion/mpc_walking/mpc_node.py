# mpc_walking/mpc_node.py (runs at 200 Hz)
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

class ConvexMPCWalker(Node):
    def __init__(self):
        super().__init__('convex_mpc_walker')
        
        # Initialize MPC parameters
        self.com_state = np.array([0.0, 0.0, 0.8])  # [x, y, z]
        self.footstep_queue = []
        self.horizon = 1.6  # seconds
        self.dt = 0.005  # 200 Hz
        
        # Publishers and subscribers
        self.joint_pub = self.create_publisher(JointState, '/joint_commands', 10)
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        
        # Timer for MPC loop
        self.mpc_timer = self.create_timer(self.dt, self.solve_mpc)
        
        self.get_logger().info('Convex MPC Walker initialized')

    def cmd_callback(self, msg):
        # Store desired velocity command
        self.desired_vel = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def solve_mpc(self):
        # Simplified MPC solver
        prob = ConvexMPCProblem(
            com_state=self.com_state,
            foot_plan=self.footstep_queue,
            horizon=1.6  # seconds
        )
        com_traj, zmp_traj, foot_forces = prob.solve_qp()
        
        # Return the first command in the trajectory
        next_com_vel = com_traj[0] if len(com_traj) > 0 else np.zeros(3)
        
        # Convert to joint commands (simplified)
        joint_commands = self.compute_joint_commands(next_com_vel)
        
        # Publish joint commands
        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.position = joint_commands
        self.joint_pub.publish(joint_msg)

    def compute_joint_commands(self, com_vel):
        # Simplified inverse kinematics and whole-body control
        # In practice, this would be a full QP solver
        joints = [0.0] * 36  # 36 DoF for humanoid
        return joints

class ConvexMPCProblem:
    def __init__(self, com_state, foot_plan, horizon):
        self.com_state = com_state
        self.foot_plan = foot_plan
        self.horizon = horizon

    def solve_qp(self):
        # Simplified QP solver
        # In practice, this would use osqp or similar
        n_steps = int(self.horizon / 0.005)
        com_traj = [self.com_state + i * 0.01 for i in range(n_steps)]
        zmp_traj = [np.array([0.0, 0.0]) for _ in range(n_steps)]
        foot_forces = [np.array([0.0, 0.0, 500.0]) for _ in range(n_steps)]
        
        return com_traj, zmp_traj, foot_forces

def main():
    rclpy.init()
    node = ConvexMPCWalker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
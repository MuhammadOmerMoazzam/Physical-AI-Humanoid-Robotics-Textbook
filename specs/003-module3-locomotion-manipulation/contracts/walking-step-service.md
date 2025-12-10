# Locomotion Service API Contract

**Service**: `/locomotion/walking_step`
**Interface**: ROS 2 Service
**Module**: Chapter 06 - Bipedal Locomotion & Whole-Body Control
**Status**: Stable

## Description
The Walking Step service controls the humanoid's walking gait and maintains balance during locomotion.

## Request
| Field | Type | Description |
|-------|------|-------------|
| target_velocity | geometry_msgs/Twist | Desired linear and angular velocity |
| terrain_type | std_msgs/String | Classification of terrain (flat, rough, stairs, etc.) |
| step_height | std_msgs/Float64 | Desired foot step height |
| gait_pattern | std_msgs/String | Gait type (walking, running, climbing) |

## Response
| Field | Type | Description |
|-------|------|-------------|
| success | std_msgs/Bool | Whether step was executed successfully |
| actual_velocity | geometry_msgs/Twist | Actual achieved velocity |
| zmp_error | std_msgs/Float64 | ZMP tracking error magnitude |
| balance_state | std_msgs/String | Current balance state (stable, recovering, unstable) |
| next_support_foot | std_msgs/String | Which foot will be support foot next (left, right, none) |

## Example Usage
```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
# Import custom service types

class WalkingControllerClient(Node):
    def __init__(self):
        super().__init__('walking_controller_client')
        self.client = self.create_client(WalkingStep, '/locomotion/walking_step')
        
    def execute_step(self, target_vel, terrain="flat", height=0.05):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Walking step service not available...')
            
        request = WalkingStep.Request()
        request.target_velocity = target_vel
        request.terrain_type = terrain
        request.step_height = height
        request.gait_pattern = "walking"
        
        future = self.client.call_async(request)
        return future
```

## Performance Requirements
- Response time: < 10ms for typical walking steps
- Balance maintenance: Keep humanoid upright during continuous walking
- Velocity tracking: Achieve >90% of commanded velocity while maintaining stability
- Terrain adaptation: Automatically adjust gait based on terrain classification
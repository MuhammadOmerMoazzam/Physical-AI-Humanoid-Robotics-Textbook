# ROS 2 Service API Contract: Joint State Publisher

**Service**: `/joint_state_publisher`
**Interface**: ROS 2 Service
**Module**: Chapter 02 - ROS 2: The Robotic Nervous System
**Status**: Stable

## Description
The Joint State Publisher service manages the publishing of joint positions, velocities, and efforts for the humanoid robot.

## Request
| Field | Type | Description |
|-------|------|-------------|
| joint_names | string[] | Names of joints to publish state for |
| positions | float64[] | Joint positions in radians or meters |
| velocities | float64[] | Joint velocities in rad/s or m/s |
| efforts | float64[] | Joint efforts in Nm or N |

## Response
| Field | Type | Description |
|-------|------|-------------|
| success | bool | Whether state was published successfully |
| message | string | Descriptive message about the result |

## Example Usage
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
# Import custom service types

class JointStatePublisherClient(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_client')
        self.client = self.create_client(JointStateSrv, '/joint_state_publisher')
        
    def publish_joint_state(self, joint_names, positions, velocities=None, efforts=None):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Joint state publisher service not available...')
            
        request = JointStateSrv.Request()
        request.joint_names = joint_names
        request.positions = positions
        request.velocities = velocities if velocities else [0.0] * len(positions)
        request.efforts = efforts if efforts else [0.0] * len(positions)
        
        future = self.client.call_async(request)
        return future
```

## Performance Requirements
- Response time: < 1ms for typical joint state updates
- Frequency: Supports publishing at 100 Hz control rates
- Validation: Reject malformed joint state data with descriptive error
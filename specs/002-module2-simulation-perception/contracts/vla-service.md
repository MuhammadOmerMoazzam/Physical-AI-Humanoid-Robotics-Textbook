# VLA Service API Contract - Duplicate

**Service**: `/vla_predict`
**Interface**: ROS 2 Service
**Module**: Chapter 08 - Vision-Language-Action Models
**Status**: Stable

## Description
The VLA service accepts an image and natural language command and returns an action sequence for the humanoid robot.

## Request
| Field | Type | Description |
|-------|------|-------------|
| image | sensor_msgs/Image | RGB image from robot's camera |
| command | std_msgs/String | Natural language instruction |

## Response
| Field | Type | Description |
|-------|------|-------------|
| action_sequence | std_msgs/Float64MultiArray | Sequence of joint positions or velocities |
| success | std_msgs/Bool | Whether prediction was successful |
| confidence | std_msgs/Float64 | Model confidence (0.0-1.0) |

## Example Usage
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
# Import custom service types

class VLAClient(Node):
    def __init__(self):
        super().__init__('vla_client')
        self.client = self.create_client(VLAPredict, '/vla_predict')
        
    def predict_action(self, image, command):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('VLA service not available...')
            
        request = VLAPredict.Request()
        request.image = image
        request.command = command
        
        future = self.client.call_async(request)
        return future
```

## Performance Requirements
- Response time: < 100ms for 28Hz inference
- Success rate: >95% on validated commands
- Input validation: Reject unsupported commands with descriptive error
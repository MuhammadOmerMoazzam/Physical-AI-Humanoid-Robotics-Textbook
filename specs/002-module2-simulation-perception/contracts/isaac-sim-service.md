# Isaac Sim Service API Contract

**Service**: `/isaac_sim/control_step`
**Interface**: ROS 2 Service
**Module**: Chapter 04 - Physics Simulation with Isaac Sim
**Status**: Stable

## Description
The Isaac Sim service controls stepping the Isaac Sim physics engine and managing simulation state during humanoid simulation.

## Request
| Field | Type | Description |
|-------|------|-------------|
| step_count | int32 | Number of physics steps to execute |
| dt | float64 | Time delta for each step in seconds |
| sync_to_real_time | bool | Whether to sync simulation time to real time |

## Response
| Field | Type | Description |
|-------|------|-------------|
| success | bool | Whether simulation stepped successfully |
| actual_steps | int32 | Actual number of steps executed |
| simulation_time | float64 | Total simulation time elapsed |
| error_message | string | Error details if success is false |

## Example Usage
```python
import rclpy
from rclpy.node import Node
from your_custom_interfaces.srv import IsaacSimStep  # You would define this service

class IsaacSimController(Node):
    def __init__(self):
        super().__init__('isaac_sim_controller')
        self.client = self.create_client(IsaacSimStep, '/isaac_sim/control_step')
        
    def step_simulation(self, num_steps, time_delta, real_time_sync=False):
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Isaac Sim service not available...')
            
        request = IsaacSimStep.Request()
        request.step_count = num_steps
        request.dt = time_delta
        request.sync_to_real_time = real_time_sync
        
        future = self.client.call_async(request)
        return future
```

## Performance Requirements
- Response time: < 10ms for typical simulation steps
- Consistency: Physics steps must maintain determinism across runs
- Validation: Reject invalid time deltas that would break physics stability
#!/usr/bin/env python3
"""
Shoe Spawner Node for Gazebo Fortress
Spawns shoes at random positions in the workspace
Provides service to change shoe position dynamically
"""

import rclpy
from rclpy.node import Node
import random
import math
import os
from pathlib import Path

from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty, Trigger

class ShoeSpawnerNode(Node):
"""
Node to spawn and manage shoe positions in Gazebo Fortress
"""

```
def __init__(self):
    super().__init__('shoe_spawner_node')

    # ---------------------------------------------------------------------
    # AUTO-DETECT SHOE MODEL PATH
    # shoe_env/models/shoe/shoe_model.sdf
    # ---------------------------------------------------------------------
    default_model_path = str(
        Path(__file__).resolve().parent.parent
        / "models" / "shoe" / "shoe_model.sdf"
    )

    # Parameters
    self.declare_parameter('shoe_model_path', default_model_path)
    self.declare_parameter('workspace_x_min', -0.3)
    self.declare_parameter('workspace_x_max', 0.3)
    self.declare_parameter('workspace_y_min', -0.2)
    self.declare_parameter('workspace_y_max', 0.2)
    self.declare_parameter('table_height', 0.45)
    self.declare_parameter('spawn_on_startup', True)

    # Load parameters
    self.shoe_model_path = (
        self.get_parameter('shoe_model_path')
        .get_parameter_value()
        .string_value
    )
    self.workspace_x_min = self.get_parameter('workspace_x_min').value
    self.workspace_x_max = self.get_parameter('workspace_x_max').value
    self.workspace_y_min = self.get_parameter('workspace_y_min').value
    self.workspace_y_max = self.get_parameter('workspace_y_max').value
    self.table_height = self.get_parameter('table_height').value
    self.spawn_on_startup = self.get_parameter('spawn_on_startup').value

    # State
    self.shoe_spawned = False
    self.shoe_name = "shoe_defect_sample"

    # Gazebo services
    self.spawn_client = self.create_client(
        SpawnEntity,
        '/world/shoe_inspection_world/create'
    )

    self.delete_client = self.create_client(
        DeleteEntity,
        '/world/shoe_inspection_world/remove'
    )

    # External control services
    self.randomize_service = self.create_service(
        Trigger,
        '/shoe_spawner/randomize_position',
        self.randomize_position_callback
    )

    self.spawn_service = self.create_service(
        Trigger,
        '/shoe_spawner/spawn_shoe',
        self.spawn_shoe_callback
    )

    self.remove_service = self.create_service(
        Trigger,
        '/shoe_spawner/remove_shoe',
        self.remove_shoe_callback
    )

    self.get_logger().info("Shoe Spawner Node initialized")

    # Wait for Gazebo
    self.get_logger().info("Waiting for Gazebo services...")
    self.spawn_client.wait_for_service(timeout_sec=10.0)
    self.delete_client.wait_for_service(timeout_sec=10.0)
    self.get_logger().info("Gazebo services available")

    # Auto-spawn on startup
    if self.spawn_on_startup:
        self.spawn_shoe_at_random_position()

# -------------------------------------------------------------------------
# HELPER FUNCTIONS
# -------------------------------------------------------------------------

def generate_random_pose(self) -> Pose:
    pose = Pose()
    pose.position.x = random.uniform(self.workspace_x_min, self.workspace_x_max)
    pose.position.y = random.uniform(self.workspace_y_min, self.workspace_y_max)
    pose.position.z = self.table_height

    yaw = random.uniform(0, 2 * math.pi)
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = math.sin(yaw / 2.0)
    pose.orientation.w = math.cos(yaw / 2.0)
    return pose

def load_shoe_model_sdf(self) -> str:
    """Load shoe_model.sdf from workspace"""
    try:
        if os.path.exists(self.shoe_model_path):
            self.get_logger().info(f"Loading SDF: {self.shoe_model_path}")
            with open(self.shoe_model_path, 'r') as f:
                return f.read()
        else:
            self.get_logger().error(
                f"Shoe model not found at: {self.shoe_model_path}. Using default cube model."
            )
            return self.get_default_shoe_sdf()

    except Exception as e:
        self.get_logger().error(f"Error loading shoe model: {e}")
        return self.get_default_shoe_sdf()

def get_default_shoe_sdf(self) -> str:
    """Fallback simple shoe SDF"""
    return """<?xml version="1.0"?>
```

<sdf version="1.8">
  <model name="shoe_fallback">
    <link name="body">
      <collision name="shape">
        <geometry><box><size>0.28 0.10 0.03</size></box></geometry>
      </collision>
      <visual name="shape_v">
        <geometry><box><size>0.28 0.10 0.03</size></box></geometry>
      </visual>
    </link>
  </model>
</sdf>"""

```
# -------------------------------------------------------------------------
# SPAWN / DELETE FUNCTIONS
# -------------------------------------------------------------------------

def spawn_shoe_at_random_position(self) -> bool:
    if self.shoe_spawned:
        self.get_logger().warn("Shoe already spawned, remove first.")
        return False

    pose = self.generate_random_pose()
    sdf_content = self.load_shoe_model_sdf()

    request = SpawnEntity.Request()
    request.name = self.shoe_name
    request.xml = sdf_content
    request.initial_pose = pose

    self.get_logger().info(
        f"Spawning shoe at x={pose.position.x:.3f}, y={pose.position.y:.3f}, z={pose.position.z:.3f}"
    )

    future = self.spawn_client.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    if future.result() and future.result().success:
        self.shoe_spawned = True
        self.get_logger().info("Shoe spawned successfully!")
        return True

    self.get_logger().error("Failed to spawn shoe.")
    return False

def remove_shoe(self) -> bool:
    if not self.shoe_spawned:
        self.get_logger().warn("No shoe to remove.")
        return False

    request = DeleteEntity.Request()
    request.name = self.shoe_name

    future = self.delete_client.call_async(request)
    rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

    if future.result() and future.result().success:
        self.shoe_spawned = False
        self.get_logger().info("Shoe removed successfully!")
        return True

    self.get_logger().error("Failed to remove shoe.")
    return False

# -------------------------------------------------------------------------
# SERVICE CALLBACKS
# -------------------------------------------------------------------------

def randomize_position_callback(self, request, response):
    self.get_logger().info("Randomizing shoe position...")
    if self.shoe_spawned:
        if not self.remove_shoe():
            response.success = False
            response.message = "Error removing shoe"
            return response

    import time
    time.sleep(0.5)

    if self.spawn_shoe_at_random_position():
        response.success = True
        response.message = "Randomized successfully"
    else:
        response.success = False
        response.message = "Spawn failed"

    return response

def spawn_shoe_callback(self, request, response):
    if self.spawn_shoe_at_random_position():
        response.success = True
        response.message = "Shoe spawned"
    else:
        response.success = False
        response.message = "Spawn failed"
    return response

def remove_shoe_callback(self, request, response):
    if self.remove_shoe():
        response.success = True
        response.message = "Shoe removed"
    else:
        response.success = False
        response.message = "Remove failed"
    return response
```

def main(args=None):
rclpy.init(args=args)

```
node = ShoeSpawnerNode()

try:
    rclpy.spin(node)
except KeyboardInterrupt:
    node.get_logger().info("Shutting down spawner...")
finally:
    node.destroy_node()
    rclpy.shutdown()
```

if **name** == '**main**':
main()


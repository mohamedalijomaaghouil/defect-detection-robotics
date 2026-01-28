import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from geometry_msgs.msg import PoseStamped
import math
import time

class ShoeOrbiNode(Node):
    def __init__(self):
        super().__init__('shoe_orbiter')
        self._action_client = ActionClient(self, MoveGroup, 'move_action')
        
        # Orbit Parameters
        self.center_x = 0.3    # Shoe X
        self.center_y = 0.0    # Shoe Y
        self.radius = 0.2      # Distance from shoe
        self.height = 0.3      # Height above table
        
        self.angles = [0.0, 90.0, 180.0, 270.0] # Angles to take pictures from
        self.current_index = 0
        
        self.run_orbit()

    def run_orbit(self):
        self._action_client.wait_for_server()
        self.send_next_pose()

    def send_next_pose(self):
        if self.current_index >= len(self.angles):
            self.get_logger().info('Orbit complete!')
            return

        angle_deg = self.angles[self.current_index]
        angle_rad = math.radians(angle_deg)

        # Calculate X and Y on a circle
        target_x = self.center_x + self.radius * math.cos(angle_rad)
        target_y = self.center_y + self.radius * math.sin(angle_rad)

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'ur_manipulator'
        
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = target_x
        target_pose.pose.position.y = target_y
        target_pose.pose.position.z = self.height

        # Orientation: This is tricky. 
        # For a simple "looking down" orbit, we keep orientation fixed.
        # For a "sidelooking" orbit, you'd need to calculate the quaternion.
        target_pose.pose.orientation.x = 1.0 
        target_pose.pose.orientation.w = 0.0

        self.get_logger().info(f'Moving to angle {angle_deg}...')
        goal_msg.request.goal_constraints.append(self.create_pose_constraint(target_pose))
        
        future = self._action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def create_pose_constraint(self, pose):
        from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
        from shape_msgs.msg import SolidPrimitive
        pos_con = PositionConstraint()
        pos_con.header.frame_id = 'base_link'
        pos_con.link_name = 'tool0'
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.02, 0.02, 0.02])
        pos_con.constraint_region = BoundingVolume(primitives=[box], primitive_poses=[pose.pose])
        return Constraints(position_constraints=[pos_con])

    def goal_response_callback(self, future):
        goal_handle = future.result()
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info('Position reached. Taking picture...')
        time.sleep(2) # Simulate taking a photo
        self.current_index += 1
        self.send_next_pose()

def main():
    rclpy.init()
    node = ShoeOrbiNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
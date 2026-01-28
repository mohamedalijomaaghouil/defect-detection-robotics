#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive


class MoveItActionTest(Node):
    def __init__(self):
        super().__init__("moveit_action_test")

        self.client = ActionClient(self, MoveGroup, "/move_action")

        self.get_logger().info("Waiting for MoveGroup action server...")
        self.client.wait_for_server()

        self.send_goal()

    def send_goal(self):
        goal = MoveGroup.Goal()
        goal.request.group_name = "ur_manipulator"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 5.0

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.4
        pose.pose.orientation.w = 1.0

        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.01, 0.01, 0.01]

        pc.constraint_region.primitives.append(primitive)
        pc.constraint_region.primitive_poses.append(pose.pose)
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        oc.orientation.w = 1.0
        oc.absolute_x_axis_tolerance = 0.1
        oc.absolute_y_axis_tolerance = 0.1
        oc.absolute_z_axis_tolerance = 0.1
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info("Sending MoveIt goal")
        self.client.send_goal_async(goal)


def main():
    rclpy.init()
    node = MoveItActionTest()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit.planning import MoveItPy
from geometry_msgs.msg import PoseStamped


class MoveIt2Test(Node):
    def __init__(self):
        super().__init__("moveit2_test")

        self.get_logger().info("Starting MoveItPy...")

        self.moveit = MoveItPy(node_name="moveit_py")
        self.arm = self.moveit.get_planning_component("ur_manipulator")

        self.move_robot()

    def move_robot(self):
        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.4
        pose.pose.orientation.w = 1.0

        self.arm.set_goal_state(pose_stamped_msg=pose)

        plan = self.arm.plan()

        if plan:
            self.get_logger().info("Executing plan")
            self.arm.execute()
        else:
            self.get_logger().error("Planning failed")


def main():
    rclpy.init()
    node = MoveIt2Test()
    rclpy.spin_once(node, timeout_sec=2)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

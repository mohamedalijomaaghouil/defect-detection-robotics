#!/usr/bin/env python3
"""
Shoe Inspection Node — Clean 3D Dome + UI (No Nudging)
===================================================================
1. UI Debug: Dessine la bounding box et publie sur /inspection/debug_image.
2. Hemispheric Dome: Calcule les poses latérales directement autour de la coordonnée.
3. Dynamic Angling: Pointeur dynamique (look_at) avec annulation du support 3D.
4. Sécurités: Zone de collision caméra et correctif de vélocité (Ghost Jumps).
"""

import os
import json
import time
import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from tf2_geometry_msgs import do_transform_point

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MoveItErrorCodes, MotionPlanRequest,
    Constraints, PositionConstraint, OrientationConstraint,
    BoundingVolume, PlanningOptions, WorkspaceParameters,
    CollisionObject, AttachedCollisionObject, RobotState
)
from geometry_msgs.msg import (
    Pose, Point, Quaternion, PoseStamped,
    PointStamped, Vector3,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header, Bool, String
from sensor_msgs.msg import JointState, Image
from cv_bridge import CvBridge

# ── Configuration Globale ─────────────────────────────────────────────
PLANNING_GROUP  = "ur_manipulator"
WORLD_FRAME     = "base"         # Aligné avec la base du robot pour éviter les erreurs TF
END_EFFECTOR    = "tool0"        
CAMERA_FRAME    = "camera_color_optical_frame"

VELOCITY_SCALE  = 0.15  
ACCEL_SCALE     = 0.15
PLANNING_TIME   = 10.0
PLANNING_TRIES  = 10

POS_TOLERANCE = 0.01
ORI_TOLERANCE = 0.25  

# --- MATHÉMATIQUES DES QUATERNIONS ---
def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w
    return Quaternion(
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    )

def quaternion_inverse(q):
    return Quaternion(x=-q.x, y=-q.y, z=-q.z, w=q.w)

def get_look_at_quat(cam_pos, target_pos):
    forward = np.array(target_pos) - np.array(cam_pos)
    norm_f = np.linalg.norm(forward)
    if norm_f < 1e-6:
        return Quaternion(w=1.0, x=0.0, y=0.0, z=0.0)
    forward = forward / norm_f

    up_ref = np.array([0.0, 0.0, 1.0])

    if abs(forward[2]) > 0.99:
        right = np.array([1.0, 0.0, 0.0])
        down = np.array([0.0, -1.0, 0.0])
    else:
        right = np.cross(up_ref, forward)
        right = right / np.linalg.norm(right)
        down = np.cross(forward, right)

    R = np.column_stack((right, down, forward))
    
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2.0
        return Quaternion(w=0.25*S, x=(R[2,1]-R[1,2])/S, y=(R[0,2]-R[2,0])/S, z=(R[1,0]-R[0,1])/S)
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        return Quaternion(w=(R[2,1]-R[1,2])/S, x=0.25*S, y=(R[0,1]+R[1,0])/S, z=(R[0,2]+R[2,0])/S)
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        return Quaternion(w=(R[0,2]-R[2,0])/S, x=(R[0,1]+R[1,0])/S, y=0.25*S, z=(R[1,2]+R[2,1])/S)
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        return Quaternion(w=(R[1,0]-R[0,1])/S, x=(R[0,2]+R[2,0])/S, y=(R[1,2]+R[2,1])/S, z=0.25*S)


class ShoeInspectionNode(Node):
    def __init__(self):
        super().__init__("shoe_inspection_node")
        self._cb_group = ReentrantCallbackGroup()

        self._move_client = ActionClient(self, MoveGroup, "/move_action", callback_group=self._cb_group)
        self._collision_pub = self.create_publisher(CollisionObject, "/collision_object", 10)
        self._attached_obj_pub = self.create_publisher(AttachedCollisionObject, "/attached_collision_object", 10)
        self._capture_pub = self.create_publisher(Bool, "/capture_photo", 10)
        self._progress_pub = self.create_publisher(String, "/photo_sequence_progress", 10)
        
        # Publisher for rqt_image_view (Friend's UI visualization)
        self._debug_img_pub = self.create_publisher(Image, "/inspection/debug_image", 10)

        self._shoe_data = None
        self.create_subscription(String, "/shoe_pose", self._shoe_cb, 10, callback_group=self._cb_group)
        self.create_subscription(Bool, "/start_photo_sequence", self._trigger_cb, 10, callback_group=self._cb_group)
        
        self.latest_bgr = None
        self.bridge = CvBridge()
        self.create_subscription(Image, "/camera/camera/color/image_raw", self._img_cb, 10, callback_group=self._cb_group)

        self._joint_state = None
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10, callback_group=self._cb_group)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.save_dir = os.path.expanduser("~/new_ws_compressed/new_ws/inspection_results")
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)

        # Inverse of 3D printed mount
        self.q_tool_to_cam = Quaternion(x=-0.003951, y=-0.007457, z=-0.692347, w=0.721515)
        self.q_cam_to_tool = quaternion_inverse(self.q_tool_to_cam)

        self._is_executing = False

        self.get_logger().info("Waiting for MoveGroup action server...")
        self._move_client.wait_for_server()
        self.get_logger().info("MoveGroup connected!")

        while self._joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        self._add_table()
        self._attach_camera_safety_box()
        self.get_logger().info("Ready! Waiting for trigger on /start_photo_sequence")

    def _joint_cb(self, msg):
        self._joint_state = msg

    def _shoe_cb(self, msg):
        try:
            self._shoe_data = json.loads(msg.data)
        except Exception:
            pass

    def _img_cb(self, msg: Image):
        """Draws the Debug UI without triggering any physical centering"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            if self._shoe_data and self._shoe_data.get("present", False):
                u = int(self._shoe_data.get("u", 0))
                v = int(self._shoe_data.get("v", 0))
                yaw_deg = float(self._shoe_data.get("yaw_deg", 0.0))
                depth = float(self._shoe_data.get("depth", 0.0))

                rect = ((float(u), float(v)), (180.0, 90.0), float(yaw_deg))
                box = cv2.boxPoints(rect).astype(np.int32)
                cv2.polylines(frame, [box], True, (0, 255, 0), 2)
                cv2.circle(frame, (u, v), 4, (0, 255, 0), -1)

                cv2.putText(frame, f"Shoe D={depth:.2f}m Yaw={yaw_deg:.1f}", (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(frame, f"u={u} v={v}", (20, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, "NO SHOE DETECTED", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)

            self.latest_bgr = frame
            
            # Publish directly to rqt_image_view
            dbg_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            dbg_msg.header = msg.header
            self._debug_img_pub.publish(dbg_msg)

        except Exception as e:
            self.get_logger().warn(f"Image processing failed: {e}")

    def _trigger_cb(self, msg):
        if msg.data and not self._is_executing:
            self._is_executing = True
            self.run_inspection()
            self._is_executing = False

    def save_photo(self, name):
        if self.latest_bgr is not None:
            try:
                safe_name = name.replace(" ", "_").lower().replace("(", "").replace(")", "")
                path = os.path.join(self.save_dir, f"{safe_name}.png")
                cv2.imwrite(path, self.latest_bgr)
                self.get_logger().info(f"📸 Image capturée : {path}")
            except Exception as e:
                self.get_logger().error(f"❌ Échec de sauvegarde : {e}")

    def _add_table(self):
        table = CollisionObject()
        table.header.frame_id = WORLD_FRAME
        table.id = "table"
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[2.0, 2.0, 0.05])
        pose = Pose()
        pose.position.z = -0.035
        pose.orientation.w = 1.0
        table.primitives.append(box)
        table.primitive_poses.append(pose)
        table.operation = CollisionObject.ADD
        self._collision_pub.publish(table)

    def _attach_camera_safety_box(self):
        cam_box = CollisionObject()
        cam_box.header.frame_id = END_EFFECTOR  
        cam_box.id = "realsense_camera"
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.12, 0.06, 0.08])
        pose = Pose()
        pose.position.x = -0.05
        pose.position.y = 0.04
        pose.position.z = 0.02
        pose.orientation.w = 1.0
        cam_box.primitives.append(box)
        cam_box.primitive_poses.append(pose)
        cam_box.operation = CollisionObject.ADD

        attached_cam = AttachedCollisionObject(link_name=END_EFFECTOR, object=cam_box)
        attached_cam.touch_links = ["tool0", "wrist_3_link", "flange"]
        self._attached_obj_pub.publish(attached_cam)

    def _add_shoe(self, sx, sy, sz):
        self._remove_shoe()
        time.sleep(0.05)
        shoe = CollisionObject()
        shoe.header.frame_id = WORLD_FRAME
        shoe.id = "shoe"
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.15, 0.10, 0.12])
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = sx, sy, 0.06
        pose.orientation.w = 1.0
        shoe.primitives.append(box)
        shoe.primitive_poses.append(pose)
        shoe.operation = CollisionObject.ADD
        self._collision_pub.publish(shoe)

    def _remove_shoe(self):
        shoe = CollisionObject()
        shoe.header.frame_id = WORLD_FRAME
        shoe.id = "shoe"
        shoe.operation = CollisionObject.REMOVE
        self._collision_pub.publish(shoe)

    def _get_shoe_in_world(self):
        for _ in range(10):
            if self._shoe_data and self._shoe_data.get("present", False):
                break
            time.sleep(0.5)
        
        if not self._shoe_data or not self._shoe_data.get("present", False):
            return None

        pt = PointStamped()
        pt.header.frame_id = CAMERA_FRAME
        pt.header.stamp = self.get_clock().now().to_msg()
        pt.point.x = float(self._shoe_data.get("x_cam", 0.0))
        pt.point.y = float(self._shoe_data.get("y_cam", 0.0))
        pt.point.z = float(self._shoe_data.get("depth", 0.0))

        try:
            transform = self._tf_buffer.lookup_transform(WORLD_FRAME, CAMERA_FRAME, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            return do_transform_point(pt, transform).point
        except Exception:
            return None

    def _move_to_pose(self, x, y, z, orientation, label="pose") -> bool:
        dist = math.sqrt(x**2 + y**2 + z**2)
        if dist > 0.48:
            self.get_logger().warn(f"⚠️ [{label}] Distance is {dist:.2f}m (Max UR3e reach is 0.50m).")

        self.get_logger().info(f"Planning [{label}]...")
        target = PoseStamped(header=Header(frame_id=WORLD_FRAME), pose=Pose(position=Point(x=x, y=y, z=z), orientation=orientation))
        
        pos_c = PositionConstraint(header=Header(frame_id=WORLD_FRAME), link_name=END_EFFECTOR, weight=1.0)
        pos_c.constraint_region = BoundingVolume(primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[POS_TOLERANCE])], primitive_poses=[target.pose])
            
        ori_c = OrientationConstraint(header=Header(frame_id=WORLD_FRAME), link_name=END_EFFECTOR, orientation=orientation, weight=1.0)
        ori_c.absolute_x_axis_tolerance = ori_c.absolute_y_axis_tolerance = ori_c.absolute_z_axis_tolerance = ORI_TOLERANCE

        # Force MoveIt to use the exact physical motor positions as the starting line
        start_state = RobotState()
        if self._joint_state is not None:
            start_state.joint_state = self._joint_state
        start_state.is_diff = False

        request = MotionPlanRequest(
            start_state=start_state, group_name=PLANNING_GROUP, num_planning_attempts=PLANNING_TRIES, allowed_planning_time=PLANNING_TIME,
            max_velocity_scaling_factor=VELOCITY_SCALE, max_acceleration_scaling_factor=ACCEL_SCALE,
            goal_constraints=[Constraints(position_constraints=[pos_c], orientation_constraints=[ori_c])],
            workspace_parameters=WorkspaceParameters(header=Header(frame_id=WORLD_FRAME), min_corner=Vector3(x=-1.0, y=-1.0, z=-0.1), max_corner=Vector3(x=1.0, y=1.0, z=1.5)))

        goal = MoveGroup.Goal(request=request, planning_options=PlanningOptions(plan_only=False, replan=True, replan_attempts=3))
        
        future = self._move_client.send_goal_async(goal)
        while not future.done(): time.sleep(0.05)
            
        gh = future.result()
        if not gh.accepted: return False

        result_future = gh.get_result_async()
        while not result_future.done(): time.sleep(0.05)
            
        return result_future.result().result.error_code.val == MoveItErrorCodes.SUCCESS


    def run_inspection(self):
        self.get_logger().info("=== Starting Shoe Inspection ===")

        shoe_pt = self._get_shoe_in_world()
        if shoe_pt is None:
            self.get_logger().error("Shoe not found or TF2 error.")
            return

        cx, cy, cz = shoe_pt.x, shoe_pt.y, shoe_pt.z
        self.get_logger().info(f"Shoe at: ({cx:.3f}, {cy:.3f}, {cz:.3f})")
        
        # Publish the collision box at the center
        self._add_shoe(cx, cy, cz)

        # 2. HEMISPHERIC DOME: Use our radius math directly around the detected coordinate
        radius = 0.13   
        h_side = 0.18   
        h_top  = 0.30   

        dynamic_poses = [
            {"name": "Pose 1 (Top)",   "pos": [cx, cy, cz + h_top]},
            {"name": "Pose 2 (Front)", "pos": [cx + radius, cy, cz + h_side]},
            {"name": "Pose 3 (Left)",  "pos": [cx, cy + radius, cz + h_side]},
            {"name": "Pose 4 (Back)",  "pos": [cx - radius, cy, cz + h_side]},
            {"name": "Pose 5 (Right)", "pos": [cx, cy - radius, cz + h_side]},
        ]

        total = len(dynamic_poses)
        target_pt = [cx, cy, cz]

        # 3. EXECUTE POSES
        for i, pose_data in enumerate(dynamic_poses):
            label = pose_data["name"]
            tx, ty, tz = pose_data["pos"]
            
            q_cam_world = get_look_at_quat([tx, ty, tz], target_pt)
            q_tool0_world = quaternion_multiply(q_cam_world, self.q_cam_to_tool)

            progress = String()
            progress.data = json.dumps({"status": "MOVING", "current": i + 1, "total": total, "pose": label})
            self._progress_pub.publish(progress)
            
            time.sleep(1.0) # Velocity fix pause
            success = self._move_to_pose(tx, ty, tz, q_tool0_world, label)

            if success:
                self.get_logger().info(f"✓ Pose atteinte ! Stabilisation du bras (2.5 sec)...")
                time.sleep(2.5) 
                self.get_logger().info(f"📸 Clic-Clac pour {label} !")
                self.save_photo(label)
                self._capture_pub.publish(Bool(data=True)) # TRIGGERS THE AI NODE
                time.sleep(1.0) 
            else:
                self.get_logger().warn(f"Skipping capture at [{label}] due to movement failure.")

        self.get_logger().info("=== Inspection Complete ===")
        self._progress_pub.publish(String(data=json.dumps({"status": "COMPLETE", "total": total})))


def main(args=None):
    rclpy.init(args=args)
    node = ShoeInspectionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import numpy as np
import cv2


class ShoeLocalizationNode(Node):
    def __init__(self):
        super().__init__('shoe_localization_node')

        # Subscribe to depth image from Gazebo / RealSense simulation
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth_image',
            self.depth_callback,
            10
        )

        # Publish a simple String with the pose info (for now)
        self.pose_pub = self.create_publisher(
            String,
            '/shoe_pose',
            10
        )

        # Camera intrinsics from SDF: width=640, height=480, hfov=1.047 rad
        self.fx = 554.38
        self.fy = 554.38
        self.cx = 320.0
        self.cy = 240.0

        self.get_logger().info('ShoeLocalizationNode started (waiting for depth images)...')

    def depth_callback(self, msg: Image):
        # Convert ROS2 Image (32FC1) to NumPy array
        try:
            depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
            return

        # Ignore empty depth images
        valid = depth[depth > 0]
        if valid.size == 0:
            self.get_logger().warn('Depth image has no valid pixels.')
            return

        h, w = depth.shape

        # --- 1) Estimate table depth from bottom 20% of the image ---
        zone_floor = depth[int(h * 0.8):, :]
        valid_floor = zone_floor[zone_floor > 0]
        if valid_floor.size == 0:
            self.get_logger().warn('No valid floor pixels to estimate table depth.')
            return

        Z_floor = float(np.median(valid_floor))

        # --- 2) Segment shoe as object above the table (by 2 cm) ---
        mask = np.zeros_like(depth, dtype=np.uint8)
        mask[(depth > 0) & (depth < Z_floor - 0.02)] = 255

        # Clean mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.medianBlur(mask, 5)

        # --- 3) Find largest contour (assumed to be the shoe) ---
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            msg_out = String()
            msg_out.data = 'present=False (no contour)'
            self.pose_pub.publish(msg_out)
            self.get_logger().info(msg_out.data)
            return

        c = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(c)
        if area < 500:
            msg_out = String()
            msg_out.data = f'present=False (contour too small: area={area:.1f})'
            self.pose_pub.publish(msg_out)
            self.get_logger().info(msg_out.data)
            return

        # --- 4) 2D bounding box + centroid ---
        x, y, w_rect, h_rect = cv2.boundingRect(c)
        u = int(x + w_rect / 2)
        v = int(y + h_rect / 2)

        # --- 5) Orientation (image space) using minAreaRect ---
        rect = cv2.minAreaRect(c)
        (_, _), (_, _), angle_deg = rect

        # --- 6) Robust depth around centroid ---
        win = 5
        u_min = max(u - win, 0)
        u_max = min(u + win + 1, w)
        v_min = max(v - win, 0)
        v_max = min(v + win + 1, h)

        patch = depth[v_min:v_max, u_min:u_max]
        valid_patch = patch[patch > 0]

        if valid_patch.size == 0:
            Z = float(depth[v, u])
        else:
            Z = float(np.median(valid_patch))

        # --- 7) Back-project (u, v, Z) -> (X, Y, Z) in camera frame ---
        if self.fx == 0.0 or self.fy == 0.0:
            self.get_logger().error('Camera intrinsics fx/fy are zero.')
            return

        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy

        # --- 8) Publish result as text (for now) ---
        text = (
            f'present=True '
            f'u={u} v={v} '
            f'depth={Z:.3f}m '
            f'X={X:.3f}m Y={Y:.3f}m Z={Z:.3f}m '
            f'yaw_deg={angle_deg:.1f}'
        )
        msg_out = String()
        msg_out.data = text
        self.pose_pub.publish(msg_out)
        self.get_logger().info(text)


def main(args=None):
    rclpy.init(args=args)
    node = ShoeLocalizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
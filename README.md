# Robotic Shoe Defect Inspection — UR3e, ROS 2, Deep Learning

Semester 7–8 team project at **SIGMA Clermont** (2025–2026), part of the European project **REMAIN-SUDOE**.

A **Universal Robots UR3e** with a wrist-mounted **Intel RealSense** camera finds a shoe on the table, photographs it from several viewpoints, and a deep learning pipeline detects and classifies the defects in each photo.

## Team

- **Mohamed Ali Jomaa Ghouil** — 3D localization of the shoe from the depth image (`localization_node.py`) and the debug image overlay
- **Chahd** and **Thomas** — MoveIt multi-view acquisition and Gazebo simulation

## Pipeline

1. **Localization** (`src/shoe_localization/localization_node.py`)
   - Estimates the floor depth (median of the bottom of the depth image) and keeps the pixels at least 3 cm above it.
   - Cleans the mask with a morphological opening, then rejects contours that are too small, too large or touching the image border.
   - Takes the largest remaining contour: its minimum-area rectangle gives the pixel center and the yaw angle.
   - Back-projects the center to a 3D point in the camera frame with the pinhole model (median depth around the center).
   - Publishes `{present, u, v, depth, x_cam, y_cam, z_cam, yaw_deg}` as JSON on `/shoe_pose`.
2. **Multi-view acquisition** (`shoe_inspection_node.py`)
   - Transforms the shoe position into the robot base frame with TF2.
   - Adds the table, the shoe and a camera safety box to the MoveIt planning scene.
   - Moves through a 5-pose hemispheric dome around the shoe (top, front, left, back, right) with **MoveIt**, orienting the camera with a look-at quaternion.
   - Saves one photo per pose.
   - Publishes a debug image on `/inspection/debug_image` with the shoe's oriented box, distance and yaw, or "NO SHOE DETECTED".
3. **Defect detection** (`src/defect_detection/defect_detection/node_dd.py`)
   - **U-Net** segmentation (ResNet-34 encoder, `segmentation_models_pytorch`) produces a defect mask for each photo.
   - Each defect region is classified by a small CNN.
   - Annotated results are saved to a results folder.
4. **Simulation** (`src/ur3e_env/`)
   A Gazebo world with the UR3e, a table and a shoe for testing motions without the real robot.

## Hardware and software

- Universal Robots UR3e, Intel RealSense depth camera mounted on `tool0`
- ROS 2 Humble, Universal Robots ROS 2 Driver, MoveIt 2, Gazebo
- Python, OpenCV, PyTorch, torchvision, segmentation_models_pytorch

## Running on the real robot

```bash
# 1. Camera → tool0 static transform
ros2 run tf2_ros static_transform_publisher \
  --x -0.050036 --y 0.042568 --z 0.021518 \
  --qx -0.003951 --qy -0.007457 --qz -0.692347 --qw 0.721515 \
  --frame-id tool0 --child-frame-id camera_color_optical_frame

# 2. Robot driver and MoveIt
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=<robot_ip> launch_rviz:=false
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur3e use_fake_hardware:=false launch_rviz:=true

# 3. Camera, localization and inspection
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true
python3 src/shoe_localization/localization_node.py
python3 shoe_inspection_node.py

# 4. Start the inspection
ros2 topic pub --once /start_photo_sequence std_msgs/msg/Bool "data: true"

# 5. Run defect detection on the saved photos
python3 src/defect_detection/defect_detection/node_dd.py
```

Debug view: `ros2 run rqt_image_view rqt_image_view` on `/inspection/debug_image`.

## Repository structure

```
├── shoe_inspection_node.py          # MoveIt multi-view acquisition
├── src/shoe_localization/           # Depth-based shoe localization
├── src/defect_detection/            # U-Net segmentation + CNN classification, trained models
├── src/ur3e_env/                    # Gazebo world, table and shoe models
├── src/Universal_Robots_ROS2_*      # Upstream Universal Robots packages
└── docs/                            # Command notes
```

#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, ExecuteProcess
from launch.substitutions import LaunchConfiguration, FindExecutable, Command
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    
    # 1. Resolve Arguments
    robot_name = LaunchConfiguration('robot_name').perform(context)
    robot_pose = LaunchConfiguration('robot_pose').perform(context).split(' ')
    shoe_pose = LaunchConfiguration('shoe_pose').perform(context).split(' ')
    world_file_name = LaunchConfiguration('world_file').perform(context)
    controllers_file_name = LaunchConfiguration('controllers_file').perform(context)
    ur_xacro_file = LaunchConfiguration('ur_xacro').perform(context)
    ur_pkg_name = LaunchConfiguration('ur_pkg').perform(context)

    # 2. Get Package Shares
    ur3e_sim_launch_share = get_package_share_directory('ur3e_sim_launch')
    shoe_env_share = get_package_share_directory('shoe_env')
    ur_pkg_share = get_package_share_directory(ur_pkg_name)
    
    # 3. Build Paths
    world_file = os.path.join(shoe_env_share, 'worlds', world_file_name)
    controllers_file_path = os.path.join(ur3e_sim_launch_share, 'config', controllers_file_name)
    initial_positions_file = os.path.join(ur_pkg_share, 'config', 'initial_positions.yaml')
    urdf_xacro_file_path = os.path.join(ur_pkg_share, 'urdf', ur_xacro_file)
    
    # Path to the RViz config file (ensure this file exists in your package)
    rviz_config_file = os.path.join(ur3e_sim_launch_share, 'rviz', 'inspection.rviz')
    
    # 4. Build Xacro Content
    xacro_executable = FindExecutable(name='xacro').perform(context)
    xacro_cmd_string = f"{xacro_executable} {urdf_xacro_file_path} sim_ignition:=true simulation_controllers:={controllers_file_path} ur_type:=ur3e name:={robot_name} initial_positions_file:={initial_positions_file}"
    robot_description_content = Command([xacro_cmd_string])

    gz_sim = ExecuteProcess(
        cmd=['ign', 'gazebo', world_file],
        output='screen',
    )

    gz_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
        '/camera/image_raw/image@sensor_msgs/msg/Image[ignition.msgs.Image',
        '/camera/image_raw/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        '/camera/image_raw/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
    ],
    # We use remappings to keep the ROS side clean (/camera/depth_image)
    remappings=[
        ('/camera/image_raw/image', '/camera/image_raw'),
        ('/camera/image_raw/depth_image', '/camera/depth_image'),
    ],
    output='screen'
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-string', robot_description_content, '-name', robot_name, '-x', robot_pose[0], '-y', robot_pose[1], '-z', robot_pose[2]],
    )

    shoe_model_file = os.path.join(shoe_env_share, 'models', 'Mens_ASV_Billfish_Boat_Shoe_in_Dark_Brown_Leather_zdHVHXueI3w', 'model.sdf')

    spawn_shoe = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-file', shoe_model_file,
            '-name', 'shoe_model',
            '-allow_renaming',
            '-x', shoe_pose[0], '-y', shoe_pose[1], '-z', shoe_pose[2]
        ],
    )

    jsb_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    traj_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
    )

    # ADDED: RViz2 Node to visualize the inspection

    rviz_config_file = os.path.join(
    get_package_share_directory('ur3e_sim_launch'),
    'rviz', # Check if your folder is named 'rviz' or 'config'
    'inspection.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file], # Loads your specific config
        parameters=[{'use_sim_time': True}] # Fixes timing issues with Gazebo
    )

    return [gz_sim, gz_bridge, rsp, spawn_robot, spawn_shoe, jsb_spawner, traj_spawner, rviz_node]

def generate_launch_description():
    shoe_env_path = get_package_share_directory('shoe_env')
    ur_desc_path = os.path.dirname(get_package_share_directory('ur_description'))

    set_ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(shoe_env_path, 'models'),
            ':',
            ur_desc_path
        ] 
    )

    return LaunchDescription([
        set_ign_resource_path,
        DeclareLaunchArgument('world_file', default_value='shoe_inspection_world.sdf'),
        DeclareLaunchArgument('ur_pkg', default_value='ur_description'),
        DeclareLaunchArgument('ur_xacro', default_value='ur.urdf.xacro'),
        DeclareLaunchArgument('robot_name', default_value='ur3e'),
        DeclareLaunchArgument('robot_pose', default_value='0 0 0.455 0 0 0'),
        DeclareLaunchArgument('shoe_pose', default_value='0.25 0 0.445 0 0 0'),
        DeclareLaunchArgument('controllers_file', default_value='ur_controllers.yaml'),
        OpaqueFunction(function=launch_setup)
    ])

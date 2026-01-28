source install/setup.bash


ros2 launch ur3e_sim_launch spawn_inspection_world.py

ign gazebo -g

ros2 control list_controllers 

ros2 control set_controller_state joint_state_broadcaster active
Successfully activated joint_state_broadcaster

ros2 control set_controller_state joint_trajectory_controller active
Successfully activated joint_trajectory_controller

ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [
      'shoulder_pan_joint',
      'shoulder_lift_joint',
      'elbow_joint',
      'wrist_1_joint',
      'wrist_2_joint',
      'wrist_3_joint'
    ],
    points: [
      {
        positions: [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        velocities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 5, nanosec: 0}
      }
    ]
  }
}"



ros2 launch ur_moveit_config ur_moveit.launch.py   ur_type:=ur3e   use_sim_time:=true   launch_rviz:=false




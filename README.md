# Maths-Task-2-Forward-Kinematics
Step 1: Create and build the ROS 2 package
Build your package:
-colcon build
Source your workspace
-source install/setup.bash
Step 2: Run the forward kinematics node
 - ros2 run forward_kinematics forward_kine
 - ros2 topic pub -r 1 /joint_states sensor_msgs/JointState
   "{header: {stamp: {sec: 0, nanosec: 0}},
   name: ['joint1', 'joint2', 'joint3', 'joint4'], position: [0.5, 0.5, 0.5, 0.5]}"
 -ros2 topic echo /end_effector_position
 

int8 KINEMATICS_PLANNING_RESULT_OPTIMAL_SOLUTION=0

int8 KINEMATICS_PLANNING_RESULT_EXCEED_JOINT_VELOCITY_LIMIT=1

int8 KINEMATICS_PLANNING_RESULT_UNREACHABLE_KEYFRAME=2

# This message is part of the IHMC whole-body inverse kinematics module.
# This output status will be converted into the WholeBodyTrajectoryMessage.
# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Field default value -1
int32 plan_id

# List of times for each key frames.
# The length of this should be same with the length of the configurations.
float64[] key_frame_times

# List of configurations for each key frames.
controller_msgs/KinematicsToolboxOutputStatus[] robot_configurations

# Solution quality.
# The total summation of the all solution quality for each key frames.
# Field default value -1.0
float64 solution_quality

# Suggested message to send to the IHMC walking controller to request the robot to go through the key frames.
controller_msgs/WholeBodyTrajectoryMessage suggested_controller_message



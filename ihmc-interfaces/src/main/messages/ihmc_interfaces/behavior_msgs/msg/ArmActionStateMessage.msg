# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/ArmActionDefinitionMessage definition

# This is the estimated goal chest frame as the robot executes a potential whole body action.
controller_msgs/RigidBodyTransformMessage goal_chest_transform_to_world

# Hand force
geometry_msgs/Vector3 force

# Hand torque
geometry_msgs/Vector3 torque

# Joint angles
float64[7] joint_angles

# Quality of the IK solution
float64 solution_quality

# Screw primitive preview fields
geometry_msgs/Pose[<=50] preview_trajectory

float64 preview_trajectory_duration

float64 preview_trajectory_linear_velocity

float64 preview_trajectory_angular_velocity

# The user requested IK preview time normalized from 0.0 to 1.0
float64 preview_requested_time

float64[7] preview_joint_angles

float64 preview_solution_quality

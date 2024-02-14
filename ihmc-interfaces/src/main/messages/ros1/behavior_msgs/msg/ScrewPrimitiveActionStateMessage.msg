
# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/ScrewPrimitiveActionDefinitionMessage definition

geometry_msgs/Pose[] preview_trajectory

# Hand force
geometry_msgs/Vector3 force

# Hand torque
geometry_msgs/Vector3 torque

float64 preview_trajectory_duration

float64 preview_trajectory_linear_velocity

float64 preview_trajectory_angular_velocity

# The user requested IK preview time normalized from 0.0 to 1.0
float64 preview_requested_time

float64[] preview_joint_angles

float64 preview_solution_quality



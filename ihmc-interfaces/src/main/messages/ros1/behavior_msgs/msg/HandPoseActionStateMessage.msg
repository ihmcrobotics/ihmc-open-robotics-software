
# Parent state fields
behavior_msgs/ActionNodeStateMessage state

# Definition
behavior_msgs/HandPoseActionDefinitionMessage definition

# This is the estimated goal chest frame as the robot executes a potential whole body action.
controller_msgs/RigidBodyTransformMessage goal_chest_transform_to_world

# Linear hand wrench magnitude
float64 hand_wrench_magnitude_linear

# Joint angles
float64[] joint_angles

# Quality of the IK solution
float64 solution_quality



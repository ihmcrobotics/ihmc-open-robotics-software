# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the frame the goal is expressed in
string parent_frame_name

# Transform that expresses the goal pose in the parent frame
controller_msgs/RigidBodyTransformMessage goal_transform_to_parent

# Walking speed limit in meters per second. 0 or less uses the controller limit.
float64 max_forward_speed

# Turning speed limit in radians per second. 0 or less uses the controller limit.
float64 max_turning_speed

# Distance in meters from the goal position that counts as reached
float64 position_proximity

# How to head while walking, as a us.ihmc.behaviors.behaviorTree.action.actions.RLWalkHeadingMode ordinal
byte heading_mode

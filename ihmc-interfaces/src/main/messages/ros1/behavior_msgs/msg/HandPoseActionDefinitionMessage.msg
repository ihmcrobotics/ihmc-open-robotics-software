
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage action_definition

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the hand pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent

# The trajectory duration
float64 trajectory_duration

# Whether the next action can be executed at the same time of this one
bool execute_with_next_action

# Whether maintaining the rigid body controlled in world after the action is complete
bool hold_pose_in_world

# Whether the rigid body is controlled in jointspace (true) or taskspace (false)
bool joint_space_control



# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the hand pose in the parent frame
controller_msgs/RigidBodyTransformMessage right_hand_transform_to_parent

# Transform that expresses the hand pose in the parent frame
controller_msgs/RigidBodyTransformMessage left_hand_transform_to_parent

# The trajectory duration
float64 trajectory_duration

# Whether maintaining the rigid body controlled in world after the action is complete
bool hold_pose_in_world
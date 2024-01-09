# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the chest pose in the parent frame
controller_msgs/RigidBodyTransformMessage chest_transform_to_parent

# Duration of the trajectory
float64 trajectory_duration

# Whether maintaining the rigid body controlled in world after the action is complete
bool hold_pose_in_world
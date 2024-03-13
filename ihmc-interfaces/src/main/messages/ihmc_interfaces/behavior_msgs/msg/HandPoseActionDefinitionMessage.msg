# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the hand pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent

# The trajectory duration
float64 trajectory_duration

# Whether maintaining the rigid body controlled in world after the action is complete
bool hold_pose_in_world

# Whether the rigid body is controlled in jointspace (true) or taskspace (false)
bool joint_space_control

float64 linear_position_weight

float64 angular_position_weight

float64 jointspace_weight

float64 position_error_tolerance

float64 orientation_error_tolerance

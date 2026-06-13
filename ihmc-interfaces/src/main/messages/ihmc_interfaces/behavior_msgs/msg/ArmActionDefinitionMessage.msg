# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Name of the frame the this action is expressed in
string parent_frame_name

# Transform that expresses the hand pose in the parent frame
controller_msgs/RigidBodyTransformMessage transform_to_parent

# Whether the rigid body is controlled in jointspace (true) or hybrid (false) during single-point taskspace execution
bool joint_space_control

# Whether the action is defined in jointspace (true) or taskspace (false)
bool defined_in_jointspace

# Taskspace trajectory mode: 0 = single pose, 1 = screw primitive
byte taskspace_trajectory_mode 0

# Preset arm configuration
byte preset

# Joint angles
float64[7] joint_angles

# The trajectory duration
float64 trajectory_duration

# Whether maintaining the rigid body controlled in world after the action is complete
bool hold_pose_in_world

float64 linear_position_weight

float64 angular_position_weight

float64 jointspace_weight

float64 position_error_tolerance

float64 orientation_error_tolerance

# Screw primitive fields
controller_msgs/RigidBodyTransformMessage screw_axis_pose

float64 translation

float64 rotation

float64 max_linear_velocity

float64 max_angular_velocity

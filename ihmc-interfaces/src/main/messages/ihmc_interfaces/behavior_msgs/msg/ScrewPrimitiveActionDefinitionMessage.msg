# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
byte robot_side 255

# Name of the frame the this action is expressed in
string object_frame_name

# The pose of the screw axis in the object frame
controller_msgs/RigidBodyTransformMessage screw_axis_pose

# The magnitude of the translation component
float64 translation

# The magnitude of the rotation component
float64 rotation

# The max linear velocity
float64 max_linear_velocity

# The max angular velocity
float64 max_angular_velocity

float64 linear_position_weight

float64 angular_position_weight

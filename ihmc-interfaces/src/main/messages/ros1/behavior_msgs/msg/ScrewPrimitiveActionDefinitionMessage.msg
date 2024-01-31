
# Parent definition fields
behavior_msgs/ActionNodeDefinitionMessage definition

# Specifies the side of the robot that this message refers to.
# Field default value 255
int8 robot_side

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

# The max force
float64 max_force

# The max torque
float64 max_torque

float64 linear_position_weight

float64 angular_position_weight

# The pose of the wrench contact in hand control frame
controller_msgs/RigidBodyTransformMessage wrench_contact_pose



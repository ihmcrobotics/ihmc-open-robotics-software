int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which foot is expected to be executing the footstep to be adjusted.
# Field default value 255
int8 robot_side

# Specifies the adjusted position of the footstep. It is expressed in world frame.
geometry_msgs/Point location

# Specifies the adjusted orientation of the footstep. It is expressed in world frame.
geometry_msgs/Quaternion orientation

# Predicted contact points represent the vertices of the expected contact polygon between the foot and the world.
# An empty list will request the controller to use the default foot support polygon.
# Contact points  are expressed in sole frame. The ordering does not matter.
# For example: to tell the controller to use the entire foot, the predicted contact points would be:
# - x: 0.5 * foot_length, y: -0.5 * toe_width
# - x: 0.5 * foot_length, y: 0.5 * toe_width
# - x: -0.5 * foot_length, y: -0.5 * heel_width
# - x: -0.5 * foot_length, y: 0.5 * heel_width
# Note: The z coordinate of each point is ignored.
geometry_msgs/Point[] predicted_contact_points_2d

# The time to delay this command on the controller side before being executed.
float64 execution_delay_time



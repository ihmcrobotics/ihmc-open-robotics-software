# Specifies which foot will swing to reach the footstep.
# Field default value 255
int8 robot_side

# Specifies the position of the footstep (sole frame) in world frame.
geometry_msgs/Point position

# Specifies the orientation of the footstep (sole frame) in world frame.
geometry_msgs/Quaternion orientation

# Support polygon
ihmc_common_msgs/Point2DMessage[<=16] support_polygon

# Description of the footstep
string description
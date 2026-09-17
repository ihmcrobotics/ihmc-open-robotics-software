int8 ROBOT_SIDE_LEFT=0
int8 ROBOT_SIDE_RIGHT=1


uint32 sequence_id

float64 start_time
float64 end_time
float64 swing_height -1.0
# Specifies which foot will swing to reach the footstep.
byte robot_side 255
# Specifies the position of the footstep (sole frame) in world frame.
geometry_msgs/Point location
# Specifies the orientation of the footstep (sole frame) in world frame.
geometry_msgs/Quaternion orientation
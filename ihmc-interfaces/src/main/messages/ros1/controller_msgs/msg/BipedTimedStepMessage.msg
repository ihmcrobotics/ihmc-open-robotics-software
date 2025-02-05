
int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

uint32 sequence_id

float64 start_time

float64 end_time

# Field default value -1.0
float64 swing_height

# Specifies which foot will swing to reach the footstep.
# Field default value 255
int8 robot_side

# Specifies the position of the footstep (sole frame) in world frame.
geometry_msgs/Point location

# Specifies the orientation of the footstep (sole frame) in world frame.
geometry_msgs/Quaternion orientation



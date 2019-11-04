int8 FRONT_LEFT=0

int8 FRONT_RIGHT=1

int8 HIND_RIGHT=2

int8 HIND_LEFT=3

int8 TRAJECTORY_TYPE_DEFAULT=0

int8 TRAJECTORY_TYPE_OBSTACLE_CLEARANCE=1

int8 TRAJECTORY_TYPE_CUSTOM=2

int8 TRAJECTORY_TYPE_WAYPOINTS=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the quadrant of the foot to swing.
# Field default value 255
int8 robot_quadrant

# Specifies the world-frame touchdown position of the step
geometry_msgs/Point goal_position

# Specifies the height of the swing trajectory.
# If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
# Field default value -1.0
float64 ground_clearance

# This contains information on what the swing trajectory should be for each step. Recommended is TRAJECTORY_TYPE_DEFAULT.
# TRAJECTORY_TYPE_CUSTOM and TRAJECTORY_TYPE_WAYPOINTS are currently not supported. If a trajectory type is not set,
# the controller uses either TRAJECTORY_TYPE_DEFAULT or TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
# Field default value 255
int8 trajectory_type



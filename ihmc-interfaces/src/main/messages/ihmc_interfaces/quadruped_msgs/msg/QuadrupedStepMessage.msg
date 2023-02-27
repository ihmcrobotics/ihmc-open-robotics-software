# This message is part of the IHMC quadruped controller API.
# This message is used to switch the control scheme between different stepping modes.

byte FRONT_LEFT = 0
byte FRONT_RIGHT = 1
byte HIND_RIGHT = 2
byte HIND_LEFT = 3

byte TRAJECTORY_TYPE_DEFAULT = 0
byte TRAJECTORY_TYPE_OBSTACLE_CLEARANCE = 1
byte TRAJECTORY_TYPE_CUSTOM = 2
byte TRAJECTORY_TYPE_WAYPOINTS = 3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies the quadrant of the foot to swing.
byte robot_quadrant 255
# Specifies the world-frame touchdown position of the step 
geometry_msgs/Point goal_position
# Specifies the height of the swing trajectory.
# If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
float64 ground_clearance -1.0
# This contains information on what the swing trajectory should be for each step. Recommended is TRAJECTORY_TYPE_DEFAULT.
# TRAJECTORY_TYPE_CUSTOM and TRAJECTORY_TYPE_WAYPOINTS are currently not supported. If a trajectory type is not set,
# the controller uses either TRAJECTORY_TYPE_DEFAULT or TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
byte trajectory_type 255
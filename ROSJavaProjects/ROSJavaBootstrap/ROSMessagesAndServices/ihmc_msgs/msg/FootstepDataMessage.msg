## FootstepDataMessage
# This message specifies the position, orientation and side (left or right) of a desired footstep in
# world frame

# Options for robotSide
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot
uint8 robot_side

geometry_msgs/Vector3 location

geometry_msgs/Quaternion orientation

# predictedContactPoints specifies the vertices of the expected contact polygon between the foot and
# the world. A value of null will default to using the entire foot. Contact points should be specified
# in foot sole frame, where the origin is at the center of the foot. Order of the points does not matter.
# For example: to tell the controller to use the entire foot, the predicted contact points would be:
# predicted_contact_points:
# - {x: 0.5 * foot_length, y: -0.5 * toe_width}
# - {x: 0.5 * foot_length, y: 0.5 * toe_width}
# - {x: -0.5 * foot_length, y: -0.5 * heel_width}
# - {x: -0.5 * foot_length, y: 0.5 * heel_width}
Point2dMessage[] predicted_contact_points

# This contains information on what the swing trajectory should be for each step. Recomended is to default to basic.
# Options for trajectoryType
uint8 DEFAULT=0 # is a default trajectory
uint8 BASIC=1 # will do a basic swing with the specified swing height
uint8 PUSH_RECOVERY=2 # uses a low swing height for fast steps
uint8 OBSTACLE_CLEARANCE=3 # will attempt to step over an obstacle
uint8 trajectory_type

# Contains information on how high the robot should step. This affects only basic and obstacle clearance trajectories.Recommended values are between 0.1 (default) and 0.25.
float64 swing_height

int64 unique_id



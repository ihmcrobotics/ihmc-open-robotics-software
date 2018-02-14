## FootstepDataRosMessage
# This message specifies the position, orientation and side (left or right) of a desired footstep in
# world frame.

# Specifies which foot will swing to reach the foostep.
uint8 robot_side

# Specifies the position of the footstep (sole frame) in world frame.
geometry_msgs/Point location

# Specifies the orientation of the footstep (sole frame) in world frame.
geometry_msgs/Quaternion orientation

# predictedContactPoints specifies the vertices of the expected contact polygon between the foot and
# the world. A value of null or an empty list will default to using the entire foot. Contact points
# are expressed in sole frame. This ordering does not matter. For example: to tell the controller to
# use the entire foot, the predicted contact points would be: predicted_contact_points: - {x: 0.5 *
# foot_length, y: -0.5 * toe_width} - {x: 0.5 * foot_length, y: 0.5 * toe_width} - {x: -0.5 *
# foot_length, y: -0.5 * heel_width} - {x: -0.5 * foot_length, y: 0.5 * heel_width} 
ihmc_msgs/Point2dRosMessage[] predicted_contact_points

# This contains information on what the swing trajectory should be for each step. Recomended is
# DEFAULT.
uint8 trajectory_type

# Contains information on how high the robot should swing its foot. This affects trajectory types
# DEFAULT and OBSTACLE_CLEARANCE.If a value smaller then the minumal swing height is chosen (e.g. 0.0)
# the swing height will be changed to a default value.
float64 swing_height

# In case the trajectory type is set to CUSTOM two swing waypoints can be specified here. The
# waypoints define sole positions.The controller will compute times and velocities at the waypoints.
# This is a convinient way to shape the trajectory of the swing. If full control over the
# swingtrajectory is desired use the trajectory type WAYPOINTS instead. The position waypoints are
# expected in the trajectory frame.
geometry_msgs/Point[] position_waypoints

# In case the trajectory type is set to WAYPOINTS, swing waypoints can be specified here. The
# waypoints do not include thestart point (which is set to the current foot state at lift-off) and the
# touch down point (which is specified by the location and orientation fields).All waypoints are for
# the sole frame and expressed in the trajectory frame. The maximum number of points can be found in
# the Footstep class.
ihmc_msgs/SE3TrajectoryPointRosMessage[] swing_trajectory

# In case the trajectory type is set to WAYPOINTS, this value can be used to specify the trajectory
# blend duration  in seconds. If greater than zero, waypoints that fall within the valid time window
# (beginning at the start of the swing phase and spanning  the desired blend duration) will be
# adjusted to account for the initial error between the actual and expected position and orientation
# of the swing foot. Note that the expectedInitialLocation and expectedInitialOrientation fields must
# be defined in order to enable trajectory blending.
float64 swing_trajectory_blend_duration

# The swingDuration is the time a foot is not in ground contact during a step. If the value of this
# field is invalid (not positive) it will be replaced by a default swingDuration.
float64 swing_duration

# The transferDuration is the time spent with the feet in ground contact before a step. If the value
# of this field is invalid (not positive) it will be replaced by a default transferDuration.
float64 transfer_duration

# (Experimental) The touchdown duration is the time spent trying to do a soft touchdown. If the value
# of this field is invalid (not positive) it will be replaced by a default transferDuration. If the
# default is set to zero, the touchdown state will be disabled
float64 touchdown_duration

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

# "trajectory_type" enum values:
uint8 DEFAULT=0 # The controller will execute a default trajectory.
uint8 OBSTACLE_CLEARANCE=1 # The controller will attempt to step on/off an obstacle.
uint8 CUSTOM=2 # In this mode two trajectory position waypoints can be specified.
uint8 WAYPOINTS=3 # The swing trajectory is fully defined by the given waypoints.


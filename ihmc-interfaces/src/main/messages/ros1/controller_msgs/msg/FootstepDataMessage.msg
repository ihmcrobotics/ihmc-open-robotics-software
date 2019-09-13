int8 ROBOT_SIDE_LEFT=0

int8 ROBOT_SIDE_RIGHT=1

int8 TRAJECTORY_TYPE_DEFAULT=0

int8 TRAJECTORY_TYPE_OBSTACLE_CLEARANCE=1

int8 TRAJECTORY_TYPE_CUSTOM=2

int8 TRAJECTORY_TYPE_WAYPOINTS=3

# Unique ID used to identify this message, should preferably be consecutively increasing.
uint32 sequence_id

# Specifies which foot will swing to reach the footstep.
# Field default value 255
int8 robot_side

# Specifies the position of the footstep (sole frame) in world frame.
geometry_msgs/Point location

# Specifies the orientation of the footstep (sole frame) in world frame.
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

# This contains information on what the swing trajectory should be for each step. Recommended is TRAJECTORY_TYPE_DEFAULT.
int8 trajectory_type

# Contains information on how high the robot should swing its foot.
# This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
# If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
# Field default value -1.0
float64 swing_height

# In case the trajectory type is set to TRAJECTORY_TYPE_DEFAULT or TRAJECTORY_TYPE_OBSTACLE_CLEARANCE, custom waypoint proportions
# can be requested. These proportions encode the xy positions of the swing trajectory's two waypoints. A proportion of 0.0 and 1.0 will
# place a waypoint's xy-position at the start and end of the trajectory, respectively. If this value is empty, the default proportions are used.
float64[] custom_waypoint_proportions

# In case the trajectory type is set to TRAJECTORY_TYPE_CUSTOM two swing waypoints can be specified here.
# The waypoints define sole positions.
# The controller will compute times and velocities at the waypoints.
# This is a convenient way to shape the trajectory of the swing.
# If full control over the swing trajectory is desired use the trajectory type TRAJECTORY_TYPE_WAYPOINTS instead.
# The position waypoints are expected in the trajectory frame.
geometry_msgs/Point[] custom_position_waypoints

# In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, swing waypoints can be specified here.
# The waypoints do not include the start point (which is set to the current foot state at lift-off) and the touch down point
# (which is specified by the location and orientation fields).
# All waypoints are for the sole frame and expressed in the trajectory frame.
# The maximum number of points can be found in the Footstep class.
controller_msgs/SE3TrajectoryPointMessage[] swing_trajectory

# In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, this value can be used to specify the trajectory blend duration in seconds.
# If greater than zero, waypoints that fall within the valid time window (beginning at the start of the swing phase and spanning the desired blend duration)
# will be adjusted to account for the initial error between the actual and expected position and orientation of the swing foot.
# Note that the expected_initial_location and expected_initial_orientation fields must be defined in order to enable trajectory blending.
float64 swing_trajectory_blend_duration

# The swingDuration is the time a foot is not in ground contact during a step.
# If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
# Field default value -1.0
float64 swing_duration

# The transferDuration is the time spent with the feet in ground contact before a step.
# If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
# Field default value -1.0
float64 transfer_duration

# The time to delay this command on the controller side before being executed.
float64 execution_delay_time

# The swing_duration_shift_fraction is the fraction of the swing duration spent shifting the weight from the heel of the foot to the toe of the foot.
# A higher split fraction means that the weight is shifted to the toe slowly, then spends very little time on the toe.
# A lower split fraction means that the weight is shifted to the toe quickly, then spends a long time on the toe.
# Field default value -1.0
float64 swing_duration_shift_fraction

# The swing_split_fraction is the fraction of the shift portion of swing duration spent shifting the weight from the heel of the foot to the ball of the foot.
# A higher split fraction means that the weight is shifted to the ball slowly, then to the toe quickly.
# A lower split fraction means that the weight is shifted to the ball quickly, then to the toe slowly.
# Field default value -1.0
float64 swing_split_fraction

# The transfer_split_fraction is the fraction of the transfer duration spent shifting the weight from the trailing foot to the middle of the stance.
# A higher split fraction means that the weight is shifted to the center slowly, then to the upcoming support foot quickly.
# A lower split fraction means that the weight is shifted to the center quickly, then to the upcoming support foot slowly.
# Field default value -1.0
float64 transfer_split_fraction

# The transfer_weight_distribution is the fraction through transfer that the CoP midpoint is located at.
# A lower fraction means that the midpoint is located near the trailing foot.
# A higher fraction means that the midpoint is located near the leading foot.
# Field default value -1.0
float64 transfer_weight_distribution

# Time spent after touchdown to transition from heel or toe support to full foot support. Note, that this only has an
# effect if the foot touches down non-flat. More specific: the foot pitch (in sole z-up frame) at touchdown must be
# different from the pitch of the foothold pose provided in this message.
# Field default value -1.0
float64 touchdown_duration

# Time spent in toe or heel support before the step. This duration is part of the transfer duration and must therefore
# be shorter then the transfer. Note, that this only has an effect if the swing trajectory is provided, the swing trajectory
# has its first waypoint at time 0.0, and the pitch of the first swing waypoint (in sole z-up frame) is different from the
# foot pitch.
# Field default value -1.0
float64 liftoff_duration



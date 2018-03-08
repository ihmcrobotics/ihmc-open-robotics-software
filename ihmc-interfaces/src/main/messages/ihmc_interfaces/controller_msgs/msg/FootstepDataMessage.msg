# This message is part of the IHMC whole-body controller API.
# This message specifies the position, orientation and side (left or right) of a desired footstep in world frame.

byte ROBOT_SIDE_LEFT = 0
byte ROBOT_SIDE_RIGHT = 1

byte TRAJECTORY_TYPE_DEFAULT = 0
byte TRAJECTORY_TYPE_OBSTACLE_CLEARANCE = 1
byte TRAJECTORY_TYPE_CUSTOM = 2
byte TRAJECTORY_TYPE_WAYPOINTS = 3

# Specifies which foot will swing to reach the footstep.
byte robot_side 255
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
byte trajectory_type 0
# Contains information on how high the robot should swing its foot.
# This affects trajectory types TRAJECTORY_TYPE_DEFAULT and TRAJECTORY_TYPE_OBSTACLE_CLEARANCE.
# If a value smaller then the minimal swing height is chosen (e.g. 0.0) the swing height will be changed to a default value.
float64 swing_height 0.0
# In case the trajectory type is set to TRAJECTORY_TYPE_CUSTOM two swing waypoints can be specified here.
# The waypoints define sole positions.
# The controller will compute times and velocities at the waypoints.
# This is a convenient way to shape the trajectory of the swing.
# If full control over the swing trajectory is desired use the trajectory type TRAJECTORY_TYPE_WAYPOINTS instead.
# The position waypoints are expected in the trajectory frame.
geometry_msgs/Point[<=2] custom_position_waypoints
# In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, swing waypoints can be specified here.
# The waypoints do not include the start point (which is set to the current foot state at lift-off) and the touch down point
# (which is specified by the location and orientation fields).
# All waypoints are for the sole frame and expressed in the trajectory frame.
# The maximum number of points can be found in the Footstep class.
SE3TrajectoryPointMessage[] swing_trajectory
# In case the trajectory type is set to TRAJECTORY_TYPE_WAYPOINTS, this value can be used to specify the trajectory blend duration in seconds.
# If greater than zero, waypoints that fall within the valid time window (beginning at the start of the swing phase and spanning the desired blend duration)
# will be adjusted to account for the initial error between the actual and expected position and orientation of the swing foot.
# Note that the expected_initial_location and expected_initial_orientation fields must be defined in order to enable trajectory blending.
float64 swing_trajectory_blend_duration 0.0
# The swingDuration is the time a foot is not in ground contact during a step.
# If the value of this field is invalid (not positive) it will be replaced by a default swing_duration.
float64 swing_duration -1.0
# The transferDuration is the time spent with the feet in ground contact before a step.
# If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
float64 transfer_duration -1.0
# (Experimental) The touchdown duration is the time spent trying to do a soft touchdown.
# If the value of this field is invalid (not positive) it will be replaced by a default transfer_duration.
# If the default is set to zero, the touchdown state will be disabled.
float64 touchdown_duration -1.0
# The time to delay this command on the controller side before being executed.
float64 execution_delay_time 0.0
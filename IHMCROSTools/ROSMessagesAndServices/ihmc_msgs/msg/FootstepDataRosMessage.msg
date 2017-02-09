## FootstepDataRosMessage
# This message specifies the position, orientation and side (left or right) of a desired footstep in
# world frame.

# Specifies whether the given location is the location of the ankle or the sole.
uint8 origin

# Specifies which foot will swing to reach the foostep.
uint8 robot_side

# Specifies the position of the footstep. It is expressed in world frame.
geometry_msgs/Vector3 location

# Specifies the orientation of the footstep. It is expressed in world frame.
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

# In case the trajectory type is set to custom the swing waypoints can be specified here (As of Dec
# 2016 only two waypoints are supported). The waypoints specify the sole position in the world frame.
geometry_msgs/Vector3[] trajectory_waypoints

# Contains information on how high the robot should step. This affects trajectory types default and
# obstacle clearance.Recommended values are between 0.1 (minimum swing height, default) and 0.25. 
float64 swing_height

# Boolean that determines whether the controller should use swing and transfer times specifies in this
# message (if true) orif the default swing and transfer times from the FootstepDataListMessage should
# be used (if false).
bool has_timings

# Specifies the swing time for this footstep.
float64 swing_time

# Specifies the transfer time before this step.
float64 transfer_time

# Boolean that determines whether the controller should attemp at keeping absolute timings for the
# execution of this footstep. This means that a time for foot lift-off (end of toe off) needs to be
# specified. The timing is set with respect to the start of the execution of the FootstepDataList that
# this footstep is part of.
bool has_absolute_time

# If using absolute timings this is the time at which the controller will start the swing. The time is
# with respect to the time at which the controller recieves the walking command. The value of this
# time must be increasing throughout a FootstepDataListMessage, otherwise it is ignored.
float64 swing_start_time

# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id
# in the top level message is used, the unique id in nested messages is ignored. Use
# /output/last_received_message for feedback about when the last message was received. A message with
# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.
int64 unique_id


# This message utilizes "enums". Enum value information for this message follows.

# "robot_side" enum values:
uint8 LEFT=0 # refers to the LEFT side of a robot
uint8 RIGHT=1 # refers to the RIGHT side of a robot

# "footstep_origin" enum values:
uint8 AT_ANKLE_FRAME=0 # The location of the footstep refers to the location of the ankle frame. The ankle frame is fixed in the foot, centered at the last ankle joint. The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward. This option is for backward compatibility only and will be gone in an upcoming release. This origin is deprecated as it directly depends on the robot structure and is not directly related to the actual foot sole.
uint8 AT_SOLE_FRAME=1 # The location of the footstep refers to the location of the sole frame. The sole frame is fixed in the foot, centered at the center of the sole. The orientation = [qx = 0.0, qy = 0.0, qz = 0.0, qs = 1.0] corresponds to: x-axis pointing forward, y-axis pointing left, z-axis pointing upward. This origin is preferred as it directly depends on the actual foot sole and is less dependent on the robot structure.

# "trajectory_type" enum values:
uint8 DEFAULT=0 # is a default trajectory
uint8 OBSTACLE_CLEARANCE=1 # will attempt to step over an obstacle
uint8 CUSTOM=2 # allows to specify trajectory waypoints


## WholeBodyTrajectoryPacketMessage
# Send whole body trajectories to the robot. A best effort is 
# made to execute the trajectory while balance is kept.
# Internally the first waypoint at time 0.0 is set to the current
# position, do not provide a waypoint at time 0.0.
# Positions and orientations are set to null if no motion is desired
# velocities are set to null if they are zero

# Sequence of desired time at the waypoints of the trajecotry. 
# The execution starts at 0. Do not proivde time 0.0
# Should be numWaypoint elements long
float64[] time_at_waypoint

# Sequence of desired positions of the pelvis in world coordinates. 
# Provide an empty list if no motion is desired
# Should be numWaypoints elements long
geometry_msgs/Vector3[] pelvis_world_position

# Sequence of desired velocities of the pelvis. 
# Provide an empty list if zero velocity is required
# Should be numWaypoints elements long
geometry_msgs/Vector3[] pelvis_linear_velocity

# Sequence of desired angular velocities of the pelvis. 
# Provide an empty list if zero angular velocity is desired
# Should be numWaypoints elements long
geometry_msgs/Vector3[] pelvis_angular_velocity

# Sequence of desired quaternion (x,y,z,w) orientations of the pelvis in world coordinates. 
# Provide an empty list if no motion is desired
# Should be numWaypoints elements long
geometry_msgs/Quaternion[] pelvis_world_orientation

# Sequence of desired quaternion (x,y,z,w) orientations of the chest in world coordinates. 
# Provide an empty list if no motion is desired
# Should be numWaypoints elements long
geometry_msgs/Quaternion[] chest_world_orientation

# Sequence of desired angular velocities of the chest. 
# Provide an empty list if zero velocity is desired
# Should be numWaypoints elements long
geometry_msgs/Vector3[] chest_angular_velocity

# Arm trajectory for the right arm. Should have numWaypoints waypoints.
# Time in trajectory_points should match the corresponding element of timeAtWaypoint
ArmJointTrajectoryPacketMessage right_arm_trajectory

# Arm trajectory for the left arm. Should have numWaypoints waypoints.
# Time in trajectory_points should match the corresponding element of timeAtWaypoint
ArmJointTrajectoryPacketMessage left_arm_trajectory

# Number of waypoints in the trajectory. Should be at least 1
int32 num_waypoints

# Number of joints in a single arm
int32 num_joints_per_arm

# A unique id for the current message. This can be a timestamp or sequence number.
# Only the unique id in the top level message is used, the unique id in nested messages is ignored.
# Use /output/last_received_message for feedback about when the last message was received.
int64 unique_id



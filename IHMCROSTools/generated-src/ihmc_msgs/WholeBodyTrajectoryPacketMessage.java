package ihmc_msgs;

public interface WholeBodyTrajectoryPacketMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/WholeBodyTrajectoryPacketMessage";
  static final java.lang.String _DEFINITION = "## WholeBodyTrajectoryPacketMessage\n# Send whole body trajectories to the robot. A best effort is \n# made to execute the trajectory while balance is kept.\n# Internally the first waypoint at time 0.0 is set to the current\n# position, do not provide a waypoint at time 0.0.\n# Positions and orientations are set to null if no motion is desired\n# velocities are set to null if they are zero\n\n# Sequence of desired time at the waypoints of the trajecotry. \n# The execution starts at 0. Do not proivde time 0.0\n# Should be numWaypoint elements long\nfloat64[] time_at_waypoint\n\n# Sequence of desired positions of the pelvis in world coordinates. \n# Provide an empty list if no motion is desired\n# Should be numWaypoints elements long\ngeometry_msgs/Vector3[] pelvis_world_position\n\n# Sequence of desired velocities of the pelvis. \n# Provide an empty list if zero velocity is required\n# Should be numWaypoints elements long\ngeometry_msgs/Vector3[] pelvis_linear_velocity\n\n# Sequence of desired angular velocities of the pelvis. \n# Provide an empty list if zero angular velocity is desired\n# Should be numWaypoints elements long\ngeometry_msgs/Vector3[] pelvis_angular_velocity\n\n# Sequence of desired quaternion (x,y,z,w) orientations of the pelvis in world coordinates. \n# Provide an empty list if no motion is desired\n# Should be numWaypoints elements long\ngeometry_msgs/Quaternion[] pelvis_world_orientation\n\n# Sequence of desired quaternion (x,y,z,w) orientations of the chest in world coordinates. \n# Provide an empty list if no motion is desired\n# Should be numWaypoints elements long\ngeometry_msgs/Quaternion[] chest_world_orientation\n\n# Sequence of desired angular velocities of the chest. \n# Provide an empty list if zero velocity is desired\n# Should be numWaypoints elements long\ngeometry_msgs/Vector3[] chest_angular_velocity\n\n# Arm trajectory for the right arm. Should have numWaypoints waypoints.\n# Time in trajectory_points should match the corresponding element of timeAtWaypoint\nArmJointTrajectoryPacketMessage right_arm_trajectory\n\n# Arm trajectory for the left arm. Should have numWaypoints waypoints.\n# Time in trajectory_points should match the corresponding element of timeAtWaypoint\nArmJointTrajectoryPacketMessage left_arm_trajectory\n\n# Number of waypoints in the trajectory. Should be at least 1\nint32 num_waypoints\n\n# Number of joints in a single arm\nint32 num_joints_per_arm\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\nint64 unique_id\n\n\n";
  double[] getTimeAtWaypoint();
  void setTimeAtWaypoint(double[] value);
  java.util.List<geometry_msgs.Vector3> getPelvisWorldPosition();
  void setPelvisWorldPosition(java.util.List<geometry_msgs.Vector3> value);
  java.util.List<geometry_msgs.Vector3> getPelvisLinearVelocity();
  void setPelvisLinearVelocity(java.util.List<geometry_msgs.Vector3> value);
  java.util.List<geometry_msgs.Vector3> getPelvisAngularVelocity();
  void setPelvisAngularVelocity(java.util.List<geometry_msgs.Vector3> value);
  java.util.List<geometry_msgs.Quaternion> getPelvisWorldOrientation();
  void setPelvisWorldOrientation(java.util.List<geometry_msgs.Quaternion> value);
  java.util.List<geometry_msgs.Quaternion> getChestWorldOrientation();
  void setChestWorldOrientation(java.util.List<geometry_msgs.Quaternion> value);
  java.util.List<geometry_msgs.Vector3> getChestAngularVelocity();
  void setChestAngularVelocity(java.util.List<geometry_msgs.Vector3> value);
  ihmc_msgs.ArmJointTrajectoryPacketMessage getRightArmTrajectory();
  void setRightArmTrajectory(ihmc_msgs.ArmJointTrajectoryPacketMessage value);
  ihmc_msgs.ArmJointTrajectoryPacketMessage getLeftArmTrajectory();
  void setLeftArmTrajectory(ihmc_msgs.ArmJointTrajectoryPacketMessage value);
  int getNumWaypoints();
  void setNumWaypoints(int value);
  int getNumJointsPerArm();
  void setNumJointsPerArm(int value);
  long getUniqueId();
  void setUniqueId(long value);
}

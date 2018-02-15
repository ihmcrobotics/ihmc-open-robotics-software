package ihmc_msgs;

public interface PelvisTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PelvisTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## PelvisTrajectoryRosMessage\n# This message commands the controller to move in taskspace the pelvis to the desired pose (position &\n# orientation) while going through the specified trajectory points. A third order polynomial function\n# is used to interpolate positions and a hermite based curve (third order) is used to interpolate the\n# orientations. To excute a single straight line trajectory to reach a desired pelvis pose, set only\n# one trajectory point with zero velocity and its time to be equal to the desired trajectory time.\n# Note that the pelvis position is limited keep the robot\'s balance (center of mass has to remain\n# inside the support polygon). A message with a unique id equals to 0 will be interpreted as invalid\n# and will not be processed by the controller. This rule does not apply to the fields of this message.\n\n# The position/orientation trajectory information.\nihmc_msgs/SE3TrajectoryRosMessage se3_trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  ihmc_msgs.SE3TrajectoryRosMessage getSe3Trajectory();
  void setSe3Trajectory(ihmc_msgs.SE3TrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}

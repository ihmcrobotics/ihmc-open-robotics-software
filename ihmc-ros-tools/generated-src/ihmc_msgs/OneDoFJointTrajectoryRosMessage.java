package ihmc_msgs;

public interface OneDoFJointTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/OneDoFJointTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## OneDoFJointTrajectoryRosMessage\n# This class is used to build trajectory messages in jointspace. It holds all the trajectory points to\n# go through with a one-dimensional trajectory. A third order polynomial function is used to\n# interpolate between trajectory points.\n\n# QP Weight, if Too low, in the event the qp can\'t achieve all of the objectives it may stop trying to\n# achieve the desireds, if too high, it will favor this joint over other objectives. If set to NaN it\n# will use the default weight for that joint\nfloat64 weight\n\n# List of trajectory points to go through while executing the trajectory.\nihmc_msgs/TrajectoryPoint1DRosMessage[] trajectory_points\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  double getWeight();
  void setWeight(double value);
  java.util.List<ihmc_msgs.TrajectoryPoint1DRosMessage> getTrajectoryPoints();
  void setTrajectoryPoints(java.util.List<ihmc_msgs.TrajectoryPoint1DRosMessage> value);
  long getUniqueId();
  void setUniqueId(long value);
}

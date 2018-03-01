package ihmc_msgs;

public interface ArmTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ArmTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## ArmTrajectoryRosMessage\n# This message commands the controller to move an arm in jointspace to the desired joint angles while\n# going through the specified trajectory points. A third order polynomial function is used to\n# interpolate between trajectory points. The jointTrajectoryMessages can have different waypoint times\n# and different number of waypoints. If a joint trajectory message is empty, the controller will hold\n# the last desired joint position while executing the other joint trajectories. A message with a\n# unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\n# This rule does not apply to the fields of this message.\n\n# Specifies the side of the robot that will execute the trajectory.\nint8 robot_side\n\n# Trajectories for each joint.\nihmc_msgs/JointspaceTrajectoryRosMessage jointspace_trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getRobotSide();
  void setRobotSide(byte value);
  ihmc_msgs.JointspaceTrajectoryRosMessage getJointspaceTrajectory();
  void setJointspaceTrajectory(ihmc_msgs.JointspaceTrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}

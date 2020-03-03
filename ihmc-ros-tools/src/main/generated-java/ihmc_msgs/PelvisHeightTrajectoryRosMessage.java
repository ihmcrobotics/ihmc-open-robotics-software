package ihmc_msgs;

public interface PelvisHeightTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PelvisHeightTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## PelvisHeightTrajectoryRosMessage\n# This mesage commands the controller to move the pelvis to a new height in the trajectory frame while\n# going through the specified trajectory points. Sending this command will not affect the pelvis\n# horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead. A\n# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the\n# controller. This rule does not apply to the fields of this message.\n\n# Execute this trajectory in user mode. User mode tries to achieve the desired regardless of the leg\n# kinematics.\nbool enable_user_pelvis_control\n\n# If enableUserPelvisControl is true then enableUserPelvisControlDuringWalking will keep the height\n# manager in user mode while walking. If this is false the height manager will switch to controller\n# mode when walking.\nbool enable_user_pelvis_control_during_walking\n\n# The position trajectory information.\nihmc_msgs/EuclideanTrajectoryRosMessage euclidean_trajectory\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  boolean getEnableUserPelvisControl();
  void setEnableUserPelvisControl(boolean value);
  boolean getEnableUserPelvisControlDuringWalking();
  void setEnableUserPelvisControlDuringWalking(boolean value);
  ihmc_msgs.EuclideanTrajectoryRosMessage getEuclideanTrajectory();
  void setEuclideanTrajectory(ihmc_msgs.EuclideanTrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}

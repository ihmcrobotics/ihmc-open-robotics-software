package ihmc_msgs;

public interface WholeBodyTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/WholeBodyTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## WholeBodyTrajectoryRosMessage\n# Send whole body trajectories to the robot. A best effort is made to execute the trajectory while balance is kept.\n#  A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule DOES apply to the fields of this message. If setting a field to null is not an option (going through IHMC ROS API), the user can use the latter rule to select the messages to be processed by the controller.\n\nHandTrajectoryRosMessage left_hand_trajectory_message\n\nHandTrajectoryRosMessage right_hand_trajectory_message\n\nArmTrajectoryRosMessage left_arm_trajectory_message\n\nArmTrajectoryRosMessage right_arm_trajectory_message\n\nChestTrajectoryRosMessage chest_trajectory_message\n\nPelvisTrajectoryRosMessage pelvis_trajectory_message\n\nFootTrajectoryRosMessage left_foot_trajectory_message\n\nFootTrajectoryRosMessage right_foot_trajectory_message\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\n# A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  ihmc_msgs.HandTrajectoryRosMessage getLeftHandTrajectoryMessage();
  void setLeftHandTrajectoryMessage(ihmc_msgs.HandTrajectoryRosMessage value);
  ihmc_msgs.HandTrajectoryRosMessage getRightHandTrajectoryMessage();
  void setRightHandTrajectoryMessage(ihmc_msgs.HandTrajectoryRosMessage value);
  ihmc_msgs.ArmTrajectoryRosMessage getLeftArmTrajectoryMessage();
  void setLeftArmTrajectoryMessage(ihmc_msgs.ArmTrajectoryRosMessage value);
  ihmc_msgs.ArmTrajectoryRosMessage getRightArmTrajectoryMessage();
  void setRightArmTrajectoryMessage(ihmc_msgs.ArmTrajectoryRosMessage value);
  ihmc_msgs.ChestTrajectoryRosMessage getChestTrajectoryMessage();
  void setChestTrajectoryMessage(ihmc_msgs.ChestTrajectoryRosMessage value);
  ihmc_msgs.PelvisTrajectoryRosMessage getPelvisTrajectoryMessage();
  void setPelvisTrajectoryMessage(ihmc_msgs.PelvisTrajectoryRosMessage value);
  ihmc_msgs.FootTrajectoryRosMessage getLeftFootTrajectoryMessage();
  void setLeftFootTrajectoryMessage(ihmc_msgs.FootTrajectoryRosMessage value);
  ihmc_msgs.FootTrajectoryRosMessage getRightFootTrajectoryMessage();
  void setRightFootTrajectoryMessage(ihmc_msgs.FootTrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}

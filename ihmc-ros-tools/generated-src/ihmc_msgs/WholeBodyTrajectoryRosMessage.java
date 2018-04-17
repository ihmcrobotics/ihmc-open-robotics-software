package ihmc_msgs;

public interface WholeBodyTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/WholeBodyTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## WholeBodyTrajectoryRosMessage\n# Send whole body trajectories to the robot. A best effort is made to execute the trajectory while\n# balance is kept.  A message with a unique id equals to 0 will be interpreted as invalid and will not\n# be processed by the controller. This rule DOES apply to the fields of this message. If setting a\n# field to null is not an option (going through IHMC ROS API), the user can use the latter rule to\n# select the messages to be processed by the controller.\n\n# Trajectory for the left hand\nihmc_msgs/HandTrajectoryRosMessage left_hand_trajectory_message\n\n# Trajectory for the right hand\nihmc_msgs/HandTrajectoryRosMessage right_hand_trajectory_message\n\n# Trajectory for the left arm joints\nihmc_msgs/ArmTrajectoryRosMessage left_arm_trajectory_message\n\n# Trajectory for the right arm joints\nihmc_msgs/ArmTrajectoryRosMessage right_arm_trajectory_message\n\n# Trajectory for the chest\nihmc_msgs/ChestTrajectoryRosMessage chest_trajectory_message\n\n# Trajectory for the pelvis\nihmc_msgs/PelvisTrajectoryRosMessage pelvis_trajectory_message\n\n# Trajectory for the left foot\nihmc_msgs/FootTrajectoryRosMessage left_foot_trajectory_message\n\n# Trajectory for the right foot\nihmc_msgs/FootTrajectoryRosMessage right_foot_trajectory_message\n\n# Trajectory for the head\nihmc_msgs/HeadTrajectoryRosMessage head_trajectory_message\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
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
  ihmc_msgs.HeadTrajectoryRosMessage getHeadTrajectoryMessage();
  void setHeadTrajectoryMessage(ihmc_msgs.HeadTrajectoryRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}

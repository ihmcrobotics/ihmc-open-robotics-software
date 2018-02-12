package ihmc_msgs;

public interface SO3TrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/SO3TrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## SO3TrajectoryRosMessage\n# \n\n# List of trajectory points (in taskpsace) to go through while executing the trajectory. Use dataFrame\n# to define what frame the points are expressed in\nihmc_msgs/SO3TrajectoryPointRosMessage[] taskspace_trajectory_points\n\n# Frame information for this message.\nihmc_msgs/FrameInformationRosMessage frame_information\n\n# The selection matrix for each axis.\nihmc_msgs/SelectionMatrix3DRosMessage selection_matrix\n\n# The weight matrix for each axis.\nihmc_msgs/WeightMatrix3DRosMessage weight_matrix\n\n# Flag that tells the controller whether the use of a custom control frame is requested.\nbool use_custom_control_frame\n\n# Pose of custom control frame. This is the frame attached to the rigid body that the taskspace\n# trajectory is defined for.\ngeometry_msgs/Transform control_frame_pose\n\n# Properties for queueing trajectories.\nihmc_msgs/QueueableRosMessage queueing_properties\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  java.util.List<ihmc_msgs.SO3TrajectoryPointRosMessage> getTaskspaceTrajectoryPoints();
  void setTaskspaceTrajectoryPoints(java.util.List<ihmc_msgs.SO3TrajectoryPointRosMessage> value);
  ihmc_msgs.FrameInformationRosMessage getFrameInformation();
  void setFrameInformation(ihmc_msgs.FrameInformationRosMessage value);
  ihmc_msgs.SelectionMatrix3DRosMessage getSelectionMatrix();
  void setSelectionMatrix(ihmc_msgs.SelectionMatrix3DRosMessage value);
  ihmc_msgs.WeightMatrix3DRosMessage getWeightMatrix();
  void setWeightMatrix(ihmc_msgs.WeightMatrix3DRosMessage value);
  boolean getUseCustomControlFrame();
  void setUseCustomControlFrame(boolean value);
  geometry_msgs.Transform getControlFramePose();
  void setControlFramePose(geometry_msgs.Transform value);
  ihmc_msgs.QueueableRosMessage getQueueingProperties();
  void setQueueingProperties(ihmc_msgs.QueueableRosMessage value);
  long getUniqueId();
  void setUniqueId(long value);
}

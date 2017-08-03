package ihmc_msgs;

public interface PelvisHeightTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PelvisHeightTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## PelvisHeightTrajectoryRosMessage\n# This mesage commands the controller to move the pelvis to a new height in the trajectory frame while\n# going through the specified trajectory points. Sending this command will not affect the pelvis\n# horizontal position. To control the pelvis 3D position use the PelvisTrajectoryMessage instead. A\n# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the\n# controller. This rule does not apply to the fields of this message.\n\n# List of trajectory points (in taskpsace) to go through while executing the trajectory.\nihmc_msgs/EuclideanTrajectoryPointRosMessage[] taskspace_trajectory_points\n\n# Frame information for this message.\nihmc_msgs/FrameInformationRosMessage frame_information\n\n# Flag that tells the controller whether the use of a custom control frame is requested.\nbool use_custom_control_frame\n\n# Pose of custom control frame. This is the frame attached to the rigid body that the taskspace\n# trajectory is defined for.\ngeometry_msgs/Transform control_frame_pose\n\n# When OVERRIDE is chosen:  - The time of the first trajectory point can be zero, in which case the\n# controller will start directly at the first trajectory point. Otherwise the controller will prepend\n# a first trajectory point at the current desired position.  When QUEUE is chosen:  - The message must\n# carry the ID of the message it should be queued to.  - The very first message of a list of queued\n# messages has to be an OVERRIDE message.  - The trajectory point times are relative to the the last\n# trajectory point time of the previous message.  - The controller will queue the joint trajectory\n# messages as a per joint basis. The first trajectory point has to be greater than zero.\nuint8 execution_mode\n\n# Only needed when using QUEUE mode, it refers to the message Id to which this message should be\n# queued to. It is used by the controller to ensure that no message has been lost on the way. If a\n# message appears to be missing (previousMessageId different from the last message ID received by the\n# controller), the motion is aborted. If previousMessageId == 0, the controller will not check for the\n# ID of the last received message.\nint64 previous_message_id\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"execution_mode\" enum values:\nuint8 OVERRIDE=0 # This message will override the previous.\nuint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.\n\n";
  static final byte OVERRIDE = 0;
  static final byte QUEUE = 1;
  java.util.List<ihmc_msgs.EuclideanTrajectoryPointRosMessage> getTaskspaceTrajectoryPoints();
  void setTaskspaceTrajectoryPoints(java.util.List<ihmc_msgs.EuclideanTrajectoryPointRosMessage> value);
  ihmc_msgs.FrameInformationRosMessage getFrameInformation();
  void setFrameInformation(ihmc_msgs.FrameInformationRosMessage value);
  boolean getUseCustomControlFrame();
  void setUseCustomControlFrame(boolean value);
  geometry_msgs.Transform getControlFramePose();
  void setControlFramePose(geometry_msgs.Transform value);
  byte getExecutionMode();
  void setExecutionMode(byte value);
  long getPreviousMessageId();
  void setPreviousMessageId(long value);
  long getUniqueId();
  void setUniqueId(long value);
}

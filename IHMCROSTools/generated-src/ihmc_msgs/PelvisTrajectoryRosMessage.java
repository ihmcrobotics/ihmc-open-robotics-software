package ihmc_msgs;

public interface PelvisTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PelvisTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## PelvisTrajectoryRosMessage\n# This message commands the controller to move in taskspace the pelvis to the desired pose (position &\n# orientation) while going through the specified trajectory points. A third order polynomial function\n# is used to interpolate positions and a hermite based curve (third order) is used to interpolate the\n# orientations. To excute a single straight line trajectory to reach a desired pelvis pose, set only\n# one trajectory point with zero velocity and its time to be equal to the desired trajectory time.\n# Note that the pelvis position is limited keep the robot\'s balance (center of mass has to remain\n# inside the support polygon). A message with a unique id equals to 0 will be interpreted as invalid\n# and will not be processed by the controller. This rule does not apply to the fields of this message.\n\n# List of trajectory points (in taskpsace) to go through while executing the trajectory. All the\n# information contained in these trajectory points needs to be expressed in world frame.\nihmc_msgs/SE3TrajectoryPointRosMessage[] taskspace_trajectory_points\n\n# Frame information for this message.\nihmc_msgs/FrameInformationRosMessage frame_information\n\n# Flag that tells the controller whether the use of a custom control frame is requested.\nbool use_custom_control_frame\n\n# Pose of custom control frame. This is the frame attached to the rigid body that the taskspace\n# trajectory is defined for.\ngeometry_msgs/Transform control_frame_pose\n\n# When OVERRIDE is chosen:  - The time of the first trajectory point can be zero, in which case the\n# controller will start directly at the first trajectory point. Otherwise the controller will prepend\n# a first trajectory point at the current desired position.  When QUEUE is chosen:  - The message must\n# carry the ID of the message it should be queued to.  - The very first message of a list of queued\n# messages has to be an OVERRIDE message.  - The trajectory point times are relative to the the last\n# trajectory point time of the previous message.  - The controller will queue the joint trajectory\n# messages as a per joint basis. The first trajectory point has to be greater than zero.\nuint8 execution_mode\n\n# Only needed when using QUEUE mode, it refers to the message Id to which this message should be\n# queued to. It is used by the controller to ensure that no message has been lost on the way. If a\n# message appears to be missing (previousMessageId different from the last message ID received by the\n# controller), the motion is aborted. If previousMessageId == 0, the controller will not check for the\n# ID of the last received message.\nint64 previous_message_id\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"execution_mode\" enum values:\nuint8 OVERRIDE=0 # This message will override the previous.\nuint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.\n\n";
  static final byte OVERRIDE = 0;
  static final byte QUEUE = 1;
  java.util.List<ihmc_msgs.SE3TrajectoryPointRosMessage> getTaskspaceTrajectoryPoints();
  void setTaskspaceTrajectoryPoints(java.util.List<ihmc_msgs.SE3TrajectoryPointRosMessage> value);
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

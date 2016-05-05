package ihmc_msgs;

public interface ArmTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ArmTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## ArmTrajectoryRosMessage\n# This message commands the controller to move an arm in jointspace to the desired joint angles while\n# going through the specified trajectory points. A third order polynomial function is used to\n# interpolate between trajectory points. The jointTrajectoryMessages can have different waypoint times\n# and different number of waypoints. If a joint trajectory message is empty, the controller will hold\n# the last desired joint position while executing the other joint trajectories. A message with a\n# unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\n# This rule does not apply to the fields of this message.\n\n# Specifies the side of the robot that will execute the trajectory.\nuint8 robot_side\n\n# List of trajectory points per joint. The expected joint ordering is from the closest joint to the\n# chest to the closest joint to the hand.\nihmc_msgs/OneDoFJointTrajectoryRosMessage[] joint_trajectory_messages\n\n# When OVERRIDE is chosen:  - The time of the first trajectory point can be zero, in which case the\n# controller will start directly at the first trajectory point. Otherwise the controller will prepend\n# a first trajectory point at the current desired position.  When QUEUE is chosen:  - The message must\n# carry the ID of the message it should be queued to.  - The very first message of a list of queued\n# messages has to be an OVERRIDE message.  - The trajectory point times are relative to the the last\n# trajectory point time of the previous message.  - The controller will queue the joint trajectory\n# messages as a per joint basis. The first trajectory point has to be greater than zero.\nuint8 execution_mode\n\n# Only needed when using QUEUE mode, it refers to the message Id to which this message should be\n# queued to. It is used by the controller to ensure that no message has been lost on the way. If a\n# message appears to be missing (previousMessageId different from the last message ID received by the\n# controller), the motion is aborted. If previousMessageId == 0, the controller will not check for the\n# ID of the last received message.\nint64 previous_message_id\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n# This message utilizes \"enums\". Enum value information for this message follows.\n\n# \"robot_side\" enum values:\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\n\n# \"execution_mode\" enum values:\nuint8 OVERRIDE=0 # This message will override the previous.\nuint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.\n\n";
  static final byte LEFT = 0;
  static final byte RIGHT = 1;
  static final byte OVERRIDE = 0;
  static final byte QUEUE = 1;
  byte getRobotSide();
  void setRobotSide(byte value);
  java.util.List<ihmc_msgs.OneDoFJointTrajectoryRosMessage> getJointTrajectoryMessages();
  void setJointTrajectoryMessages(java.util.List<ihmc_msgs.OneDoFJointTrajectoryRosMessage> value);
  byte getExecutionMode();
  void setExecutionMode(byte value);
  long getPreviousMessageId();
  void setPreviousMessageId(long value);
  long getUniqueId();
  void setUniqueId(long value);
}

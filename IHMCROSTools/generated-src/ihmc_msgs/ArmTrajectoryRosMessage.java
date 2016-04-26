package ihmc_msgs;

public interface ArmTrajectoryRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/ArmTrajectoryRosMessage";
  static final java.lang.String _DEFINITION = "## ArmTrajectoryRosMessage\n# This message commands the controller to move an arm in jointspace to the desired joint angles while going through the specified trajectory points. A third order polynomial function is used to interpolate between trajectory points. The jointTrajectoryMessages can have different waypoint times and different number of waypoints. If a joint trajectory message is empty, the controller will hold the last desired joint position while executing the other joint trajectories. A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller. This rule does not apply to the fields of this message.\n\n# Specifies the side of the robot that will execute the trajectory.\n# Options for robotSide\nuint8 LEFT=0 # refers to the LEFT side of a robot\nuint8 RIGHT=1 # refers to the RIGHT side of a robot\nuint8 robot_side\n\n# List of trajectory points per joint. The expected joint ordering is from the closest joint to the chest to the closest joint to the hand.\nOneDoFJointTrajectoryRosMessage[] joint_trajectory_messages\n\n# When OVERRIDE is chosen:\n#  - The time of the first trajectory point can be zero, in which case the controller will start directly at the first trajectory point. Otherwise the controller will prepend a first trajectory point at the current desired position.\n#  When QUEUE is chosen:\n#  - The message must carry the ID of the message it should be queued to.\n#  - The very first message of a list of queued messages has to be an OVERRIDE message.\n#  - The trajectory point times are relative to the the last trajectory point time of the previous message.\n#  - The controller will queue the joint trajectory messages as a per joint basis. The first trajectory point has to be greater than zero.\n# Options for executionMode\nuint8 OVERRIDE=0 # This message will override the previous.\nuint8 QUEUE=1 # The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.\nuint8 execution_mode\n\n# Only needed when using QUEUE mode, it refers to the message Id to which this message should be queued to. It is used by the controller to ensure that no message has been lost on the way. If a message appears to be missing (previousMessageId different from the last message ID received by the controller), the motion is aborted. If previousMessageId == 0, the controller will not check for the ID of the last received message.\nint64 previous_message_id\n\n# A unique id for the current message. This can be a timestamp or sequence number.\n# Only the unique id in the top level message is used, the unique id in nested messages is ignored.\n# Use /output/last_received_message for feedback about when the last message was received.\n# A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
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

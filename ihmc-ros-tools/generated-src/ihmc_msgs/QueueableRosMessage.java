package ihmc_msgs;

public interface QueueableRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/QueueableRosMessage";
  static final java.lang.String _DEFINITION = "## QueueableRosMessage\n# \n\n# When OVERRIDE is chosen:  - The time of the first trajectory point can be zero, in which case the\n# controller will start directly at the first trajectory point. Otherwise the controller will prepend\n# a first trajectory point at the current desired position.  When QUEUE is chosen:  - The message must\n# carry the ID of the message it should be queued to.  - The very first message of a list of queued\n# messages has to be an OVERRIDE message.  - The trajectory point times are relative to the the last\n# trajectory point time of the previous message.  - The controller will queue the joint trajectory\n# messages as a per joint basis. The first trajectory point has to be greater than zero.\nint8 execution_mode\n\n# Only needed when using QUEUE mode, it refers to the message Id to which this message should be\n# queued to. It is used by the controller to ensure that no message has been lost on the way. If a\n# message appears to be missing (previousMessageId different from the last message ID received by the\n# controller), the motion is aborted. If previousMessageId == 0, the controller will not check for the\n# ID of the last received message.\nint64 previous_message_id\n\n# The time to delay this message on the controller side before being executed.\nfloat64 execution_delay_time\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  byte getExecutionMode();
  void setExecutionMode(byte value);
  long getPreviousMessageId();
  void setPreviousMessageId(long value);
  double getExecutionDelayTime();
  void setExecutionDelayTime(double value);
  long getUniqueId();
  void setUniqueId(long value);
}

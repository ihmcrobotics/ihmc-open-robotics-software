package ihmc_msgs;

public interface PauseWalkingRosMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PauseWalkingRosMessage";
  static final java.lang.String _DEFINITION = "## PauseWalkingRosMessage\n# This message pauses the execution of a list of footsteps. If this message is sent in the middle of\n# executing a footstep, the robot will finish the step and pause when back in double support. A\n# message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the\n# controller.\n\n# True to pause walking, false to unpause and resume an existing plan.\nbool pause\n\n# A unique id for the current message. This can be a timestamp or sequence number. Only the unique id\n# in the top level message is used, the unique id in nested messages is ignored. Use\n# /output/last_received_message for feedback about when the last message was received. A message with\n# a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.\nint64 unique_id\n\n\n";
  boolean getPause();
  void setPause(boolean value);
  long getUniqueId();
  void setUniqueId(long value);
}

package ihmc_msgs;

public interface PauseCommandMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PauseCommandMessage";
  static final java.lang.String _DEFINITION = "## PauseCommandMessage\n# This message pauses the execution of a list of footsteps. If this message is\n# sent in the middle of executing a footstep, the robot will finish the step and\n# pause when back in double support.\n\nbool pause\n\nint64 unique_id\n\n\n";
  boolean getPause();
  void setPause(boolean value);
  long getUniqueId();
  void setUniqueId(long value);
}

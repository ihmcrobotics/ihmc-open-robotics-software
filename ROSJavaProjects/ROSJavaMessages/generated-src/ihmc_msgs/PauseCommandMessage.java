package ihmc_msgs;

public interface PauseCommandMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PauseCommandMessage";
  static final java.lang.String _DEFINITION = "## PauseCommandMessage\r\n# This message pauses the execution of a list of footsteps. If this message is\r\n# sent in the middle of executing a footstep, the robot will finish the step and\r\n# pause when back in double support.\r\n\r\nbool pause\r\n\r\n\r\n";
  boolean getPause();
  void setPause(boolean value);
}

package ihmc_msgs;

public interface PauseCommandMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PauseCommandMessage";
  static final java.lang.String _DEFINITION = "## PauseCommandMessage\n# This message pauses the execution of a list of footsteps. If\n# sent in the middle of executing a footstep, the robot will finish\n# the step and pause when in double support.\n\nbool pause\n\n";
  boolean getPause();
  void setPause(boolean value);
}

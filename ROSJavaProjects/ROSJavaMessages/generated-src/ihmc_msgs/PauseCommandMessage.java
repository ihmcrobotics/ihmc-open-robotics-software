package ihmc_msgs;

public interface PauseCommandMessage extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ihmc_msgs/PauseCommandMessage";
  static final java.lang.String _DEFINITION = "# PauseCommandMessage\n\nbool pause\n\n";
  boolean getPause();
  void setPause(boolean value);
}

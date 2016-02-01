package pr2_msgs;

public interface SetPeriodicCmdRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/SetPeriodicCmdRequest";
  static final java.lang.String _DEFINITION = "pr2_msgs/PeriodicCmd command\n";
  pr2_msgs.PeriodicCmd getCommand();
  void setCommand(pr2_msgs.PeriodicCmd value);
}

package pr2_msgs;

public interface SetLaserTrajCmdRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/SetLaserTrajCmdRequest";
  static final java.lang.String _DEFINITION = "pr2_msgs/LaserTrajCmd command\n";
  pr2_msgs.LaserTrajCmd getCommand();
  void setCommand(pr2_msgs.LaserTrajCmd value);
}

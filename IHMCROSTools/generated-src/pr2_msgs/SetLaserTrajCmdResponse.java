package pr2_msgs;

public interface SetLaserTrajCmdResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/SetLaserTrajCmdResponse";
  static final java.lang.String _DEFINITION = "time start_time";
  org.ros.message.Time getStartTime();
  void setStartTime(org.ros.message.Time value);
}

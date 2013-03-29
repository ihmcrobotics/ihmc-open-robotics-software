package pr2_msgs;

public interface SetPeriodicCmdResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pr2_msgs/SetPeriodicCmdResponse";
  static final java.lang.String _DEFINITION = "time start_time";
  org.ros.message.Time getStartTime();
  void setStartTime(org.ros.message.Time value);
}

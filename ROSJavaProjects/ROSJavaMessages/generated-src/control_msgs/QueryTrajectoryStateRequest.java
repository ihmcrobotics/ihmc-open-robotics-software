package control_msgs;

public interface QueryTrajectoryStateRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "control_msgs/QueryTrajectoryStateRequest";
  static final java.lang.String _DEFINITION = "time time\n";
  org.ros.message.Time getTime();
  void setTime(org.ros.message.Time value);
}

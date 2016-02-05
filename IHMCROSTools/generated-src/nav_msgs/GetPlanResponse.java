package nav_msgs;

public interface GetPlanResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nav_msgs/GetPlanResponse";
  static final java.lang.String _DEFINITION = "nav_msgs/Path plan";
  nav_msgs.Path getPlan();
  void setPlan(nav_msgs.Path value);
}

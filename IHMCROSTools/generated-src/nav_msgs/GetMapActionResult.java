package nav_msgs;

public interface GetMapActionResult extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nav_msgs/GetMapActionResult";
  static final java.lang.String _DEFINITION = "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\nHeader header\nactionlib_msgs/GoalStatus status\nGetMapResult result\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  actionlib_msgs.GoalStatus getStatus();
  void setStatus(actionlib_msgs.GoalStatus value);
  nav_msgs.GetMapResult getResult();
  void setResult(nav_msgs.GetMapResult value);
}

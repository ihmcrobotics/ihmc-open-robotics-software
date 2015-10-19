package actionlib_msgs;

public interface GoalStatusArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "actionlib_msgs/GoalStatusArray";
  static final java.lang.String _DEFINITION = "# Stores the statuses for goals that are currently being tracked\n# by an action server\nHeader header\nGoalStatus[] status_list\n\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<actionlib_msgs.GoalStatus> getStatusList();
  void setStatusList(java.util.List<actionlib_msgs.GoalStatus> value);
}

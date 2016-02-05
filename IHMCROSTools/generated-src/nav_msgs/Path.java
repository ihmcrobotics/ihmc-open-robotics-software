package nav_msgs;

public interface Path extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "nav_msgs/Path";
  static final java.lang.String _DEFINITION = "#An array of poses that represents a Path for a robot to follow\nHeader header\ngeometry_msgs/PoseStamped[] poses\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<geometry_msgs.PoseStamped> getPoses();
  void setPoses(java.util.List<geometry_msgs.PoseStamped> value);
}

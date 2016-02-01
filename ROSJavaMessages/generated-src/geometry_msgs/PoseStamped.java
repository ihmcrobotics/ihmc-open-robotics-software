package geometry_msgs;

public interface PoseStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/PoseStamped";
  static final java.lang.String _DEFINITION = "# A Pose with reference coordinate frame and timestamp\nHeader header\nPose pose\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Pose getPose();
  void setPose(geometry_msgs.Pose value);
}

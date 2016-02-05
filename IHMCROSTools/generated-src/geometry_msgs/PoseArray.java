package geometry_msgs;

public interface PoseArray extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/PoseArray";
  static final java.lang.String _DEFINITION = "# An array of poses with a header for global reference.\n\nHeader header\n\nPose[] poses\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  java.util.List<geometry_msgs.Pose> getPoses();
  void setPoses(java.util.List<geometry_msgs.Pose> value);
}

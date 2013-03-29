package geometry_msgs;

public interface WrenchStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/WrenchStamped";
  static final java.lang.String _DEFINITION = "# A wrench with reference coordinate frame and timestamp\nHeader header\nWrench wrench\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Wrench getWrench();
  void setWrench(geometry_msgs.Wrench value);
}

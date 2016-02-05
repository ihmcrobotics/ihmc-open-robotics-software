package geometry_msgs;

public interface PointStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/PointStamped";
  static final java.lang.String _DEFINITION = "# This represents a Point with reference coordinate frame and timestamp\nHeader header\nPoint point\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Point getPoint();
  void setPoint(geometry_msgs.Point value);
}

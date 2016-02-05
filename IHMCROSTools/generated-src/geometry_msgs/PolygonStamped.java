package geometry_msgs;

public interface PolygonStamped extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/PolygonStamped";
  static final java.lang.String _DEFINITION = "# This represents a Polygon with reference coordinate frame and timestamp\nHeader header\nPolygon polygon\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  geometry_msgs.Polygon getPolygon();
  void setPolygon(geometry_msgs.Polygon value);
}

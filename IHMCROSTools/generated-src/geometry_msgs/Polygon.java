package geometry_msgs;

public interface Polygon extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "geometry_msgs/Polygon";
  static final java.lang.String _DEFINITION = "#A specification of a polygon where the first and last points are assumed to be connected\nPoint32[] points\n";
  java.util.List<geometry_msgs.Point32> getPoints();
  void setPoints(java.util.List<geometry_msgs.Point32> value);
}

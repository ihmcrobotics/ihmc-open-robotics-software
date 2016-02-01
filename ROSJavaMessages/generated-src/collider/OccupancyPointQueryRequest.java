package collider;

public interface OccupancyPointQueryRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyPointQueryRequest";
  static final java.lang.String _DEFINITION = "geometry_msgs/Point point\n";
  geometry_msgs.Point getPoint();
  void setPoint(geometry_msgs.Point value);
}

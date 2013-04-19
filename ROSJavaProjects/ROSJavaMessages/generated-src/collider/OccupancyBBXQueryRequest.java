package collider;

public interface OccupancyBBXQueryRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyBBXQueryRequest";
  static final java.lang.String _DEFINITION = "# minimum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point min\n# maximum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point max\n";
  geometry_msgs.Point getMin();
  void setMin(geometry_msgs.Point value);
  geometry_msgs.Point getMax();
  void setMax(geometry_msgs.Point value);
}

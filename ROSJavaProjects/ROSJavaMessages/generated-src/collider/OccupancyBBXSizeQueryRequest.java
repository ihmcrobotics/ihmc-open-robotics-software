package collider;

public interface OccupancyBBXSizeQueryRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyBBXSizeQueryRequest";
  static final java.lang.String _DEFINITION = "# center of the query box in global frame\ngeometry_msgs/Point center\n# Size of the axis-aligned bounding box around the center (complete edge length!)\ngeometry_msgs/Point size\n";
  geometry_msgs.Point getCenter();
  void setCenter(geometry_msgs.Point value);
  geometry_msgs.Point getSize();
  void setSize(geometry_msgs.Point value);
}

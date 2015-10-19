package octomap_msgs;

public interface BoundingBoxQueryRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "octomap_msgs/BoundingBoxQueryRequest";
  static final java.lang.String _DEFINITION = "# Clear a region specified by a global axis-aligned bounding box in stored OctoMap\n\n# minimum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point min\n# maximum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point max\n";
  geometry_msgs.Point getMin();
  void setMin(geometry_msgs.Point value);
  geometry_msgs.Point getMax();
  void setMax(geometry_msgs.Point value);
}

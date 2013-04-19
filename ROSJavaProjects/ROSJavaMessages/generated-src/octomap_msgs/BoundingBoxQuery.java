package octomap_msgs;

public interface BoundingBoxQuery extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "octomap_msgs/BoundingBoxQuery";
  static final java.lang.String _DEFINITION = "# Clear a region specified by a global axis-aligned bounding box in stored OctoMap\n\n# minimum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point min\n# maximum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point max\n---\n\n";
}

package collider;

public interface OccupancyBBXQuery extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyBBXQuery";
  static final java.lang.String _DEFINITION = "# minimum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point min\n# maximum corner point of axis-aligned bounding box in global frame\ngeometry_msgs/Point max\n---\n# centers of all occupied voxels\ngeometry_msgs/Point[] occupied\n\n# centers of all free voxels\ngeometry_msgs/Point[] free\n\n# resolution of the octree = side length of all returned voxels\nfloat64 resolution\n";
}

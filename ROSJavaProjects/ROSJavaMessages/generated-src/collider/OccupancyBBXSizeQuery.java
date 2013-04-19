package collider;

public interface OccupancyBBXSizeQuery extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyBBXSizeQuery";
  static final java.lang.String _DEFINITION = "# center of the query box in global frame\ngeometry_msgs/Point center\n# Size of the axis-aligned bounding box around the center (complete edge length!)\ngeometry_msgs/Point size\n---\n# centers of all occupied voxels\ngeometry_msgs/Point[] occupied\n\n# centers of all free voxels\ngeometry_msgs/Point[] free\n\n# resolution of the octree = side length of all returned voxels\nfloat64 resolution\n";
}

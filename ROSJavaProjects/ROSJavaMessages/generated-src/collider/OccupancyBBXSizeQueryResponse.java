package collider;

public interface OccupancyBBXSizeQueryResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "collider/OccupancyBBXSizeQueryResponse";
  static final java.lang.String _DEFINITION = "# centers of all occupied voxels\ngeometry_msgs/Point[] occupied\n\n# centers of all free voxels\ngeometry_msgs/Point[] free\n\n# resolution of the octree = side length of all returned voxels\nfloat64 resolution";
  java.util.List<geometry_msgs.Point> getOccupied();
  void setOccupied(java.util.List<geometry_msgs.Point> value);
  java.util.List<geometry_msgs.Point> getFree();
  void setFree(java.util.List<geometry_msgs.Point> value);
  double getResolution();
  void setResolution(double value);
}

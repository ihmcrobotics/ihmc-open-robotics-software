package pcl_msgs;

public interface PolygonMesh extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "pcl_msgs/PolygonMesh";
  static final java.lang.String _DEFINITION = "# Separate header for the polygonal surface\nHeader header\n# Vertices of the mesh as a point cloud\nsensor_msgs/PointCloud2 cloud\n# List of polygons\nVertices[] polygons\n";
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  sensor_msgs.PointCloud2 getCloud();
  void setCloud(sensor_msgs.PointCloud2 value);
  java.util.List<pcl_msgs.Vertices> getPolygons();
  void setPolygons(java.util.List<pcl_msgs.Vertices> value);
}

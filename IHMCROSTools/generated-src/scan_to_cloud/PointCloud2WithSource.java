package scan_to_cloud;

public interface PointCloud2WithSource extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "scan_to_cloud/PointCloud2WithSource";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud2 cloud\ngeometry_msgs/Point translation\ngeometry_msgs/Quaternion orientation\n";
  sensor_msgs.PointCloud2 getCloud();
  void setCloud(sensor_msgs.PointCloud2 value);
  geometry_msgs.Point getTranslation();
  void setTranslation(geometry_msgs.Point value);
  geometry_msgs.Quaternion getOrientation();
  void setOrientation(geometry_msgs.Quaternion value);
}

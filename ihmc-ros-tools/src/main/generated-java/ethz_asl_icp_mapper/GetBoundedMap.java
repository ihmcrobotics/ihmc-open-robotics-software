package ethz_asl_icp_mapper;

public interface GetBoundedMap extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/GetBoundedMap";
  static final java.lang.String _DEFINITION = "geometry_msgs/Pose mapCenter\ngeometry_msgs/Point topRightCorner\ngeometry_msgs/Point bottomLeftCorner\n---\nsensor_msgs/PointCloud2 boundedMap\n";
}

package ethz_asl_icp_mapper;

public interface GetBoundedMapResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/GetBoundedMapResponse";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud2 boundedMap";
  sensor_msgs.PointCloud2 getBoundedMap();
  void setBoundedMap(sensor_msgs.PointCloud2 value);
}

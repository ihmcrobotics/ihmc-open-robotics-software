package ethz_asl_icp_mapper;

public interface MatchCloudsRequest extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "ethz_asl_icp_mapper/MatchCloudsRequest";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud2 reference\nsensor_msgs/PointCloud2 readings\n";
  sensor_msgs.PointCloud2 getReference();
  void setReference(sensor_msgs.PointCloud2 value);
  sensor_msgs.PointCloud2 getReadings();
  void setReadings(sensor_msgs.PointCloud2 value);
}

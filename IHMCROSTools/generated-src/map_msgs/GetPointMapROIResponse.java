package map_msgs;

public interface GetPointMapROIResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/GetPointMapROIResponse";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud2 sub_map";
  sensor_msgs.PointCloud2 getSubMap();
  void setSubMap(sensor_msgs.PointCloud2 value);
}

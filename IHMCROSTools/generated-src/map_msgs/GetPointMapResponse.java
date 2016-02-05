package map_msgs;

public interface GetPointMapResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/GetPointMapResponse";
  static final java.lang.String _DEFINITION = "sensor_msgs/PointCloud2 map";
  sensor_msgs.PointCloud2 getMap();
  void setMap(sensor_msgs.PointCloud2 value);
}

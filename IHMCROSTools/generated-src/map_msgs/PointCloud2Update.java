package map_msgs;

public interface PointCloud2Update extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "map_msgs/PointCloud2Update";
  static final java.lang.String _DEFINITION = "uint32 ADD=0\nuint32 DELETE=1\nHeader header\nuint32 type          # type of update, one of ADD or DELETE\nsensor_msgs/PointCloud2 points\n";
  static final int ADD = 0;
  static final int DELETE = 1;
  std_msgs.Header getHeader();
  void setHeader(std_msgs.Header value);
  int getType();
  void setType(int value);
  sensor_msgs.PointCloud2 getPoints();
  void setPoints(sensor_msgs.PointCloud2 value);
}
